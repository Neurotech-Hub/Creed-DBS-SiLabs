// @file Creed DBS app
// @brief Core application logic.

// Standard library includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "sl_sleeptimer.h"

#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "pin_config.h"
// #include "em_gpio.h"

#include "spidrv.h"
#include "sl_spidrv_instances.h"

#include "nvm3.h"
#include "nvm3_default.h"

// Add near other includes
#include "em_iadc.h"
#include "em_cmu.h"

#include "sl_power_manager.h"

// TIMING
#define SLEEP_TIMER_TICK_US 30
// Onset coordination, datasheet-grounded values. These busy-waits run in
// timer IRQ context, so keep them tight: overly generous values (600 µs +
// 2000 µs diagnostic) starved the BLE stack and dropped connections.
// LT6020-1 tON = 100 µs typ (datasheet); +50% margin.
#define OP_AMP_WAKE_SETTLE_US 150
// ADG1236 tON ~200 ns + charge-injection settle; one sleeptimer tick.
#define SWITCH_CONNECT_SETTLE_US 30
// DIAGNOSTIC A/B for the interpulse artifact at SHDN+2 ms: 1 = hold SHDN
// (LT6020 EN) high across the whole train (amp never shuts down between
// pulses). If the artifact vanishes, it is the LT6020 shutdown bias
// wind-down coupling through the open ADG1236. 0 = per-pulse SHDN cycling.
#define STIM_HOLD_SHDN 0
// Empirical, recalibrated for FW5 sequencing + 1 MHz SPI (bench 2026-07-16:
// P=480 µs measured 504/2480 with old offsets 93/36 -> ~117 µs constant
// per-segment overhead: tick quantization + IRQ latency + DAC write).
#define STIM_OFFSET_US 117
#define CHARGE_BALANCE_OFFSET_US 116
#define ADV_INTERVAL_MS 2000
sl_sleeptimer_timer_handle_t timer_stimulation;	   // based on frequency / intra-burst rate
sl_sleeptimer_timer_handle_t timer_pulse;		   // based on amplitude
sl_sleeptimer_timer_handle_t timer_charge_balance; // balance amplitude
sl_sleeptimer_timer_handle_t timer_burst_period;   // burst mode: time between burst onsets
sl_sleeptimer_timer_handle_t timer_burst_duration; // burst mode: length of each burst

// STIM
#define CHARGE_BALANCE_RATIO 5

// HARDWARE
#define LED0 (&sl_led_inst)
#define NVM3_KEY_CAP_ID 0x00000001UL
#define NVM3_KEY_AMPLITUDE 0x00000002UL
#define NVM3_KEY_FREQUENCY 0x00000003UL
#define NVM3_KEY_PULSE_DURATION 0x00000004UL
#define NVM3_KEY_STIM_MODE 0x00000005UL
#define NVM3_KEY_BURST_PERIOD_MS 0x00000006UL
#define NVM3_KEY_INTRA_BURST_FREQ_HZ 0x00000007UL
#define NVM3_KEY_BURST_DURATION_MS 0x00000008UL

// Protocol parameter ranges (MouseCap spec)
#define STIM_MODE_MIN 0
#define STIM_MODE_MAX 1
#define AMPLITUDE_MIN 0
#define AMPLITUDE_MAX 100
#define FREQUENCY_MIN 80
#define FREQUENCY_MAX 160
#define PULSE_DURATION_MIN 90
#define PULSE_DURATION_MAX 600
#define BURST_PERIOD_MS_MIN 1
#define BURST_PERIOD_MS_MAX 3600000
#define INTRA_BURST_FREQ_MIN 80
#define INTRA_BURST_FREQ_MAX 160
#define BURST_DURATION_MS_MIN 1
#define BURST_DURATION_MS_MAX 120000
#define CAP_ID_MIN 0
#define CAP_ID_MAX 99

// MouseCap firmware version (wire = major×10 + minor; FW5 = v0.5)
// v0.5: Deterministic pulse chain — synchronous op-amp settle, EM1 held for
//       the stim train, zero-current switch transitions, burst boundaries no
//       longer truncate in-flight pulses.
#define FIRMWARE_VERSION_MAJOR 0
#define FIRMWARE_VERSION_MINOR 5
#define FIRMWARE_VERSION_WIRE ((FIRMWARE_VERSION_MAJOR * 10) + FIRMWARE_VERSION_MINOR)

uint8_t cap_id;

// BLE
#define ATT_MTU_MAX 247 // Max ATT MTU (payload per packet = MTU - 3)
#define COMMAND_STR_MAX_SIZE 128 // node_rx/node_tx value size; fits one ATT PDU after MTU exchange
static uint8_t advertising_set_handle = 0xff;
static uint16_t negotiated_mtu = 23; // updated on sl_bt_evt_gatt_mtu_exchanged
char commandStr[COMMAND_STR_MAX_SIZE];
#define APP_BUFFER_SIZE 3
uint8_t rx_buffer[APP_BUFFER_SIZE];
uint8_t tx_buffer[APP_BUFFER_SIZE];

// App
// Transmission and reception buffers
static uint8_t activateStim = 0;
static int amplitude = 0;
static int frequency = 0;
static int pulse_duration = 0;
static int stim_mode = 0;
static int burst_period_ms = 30000;
static int intra_burst_freq_hz = 130;
static int burst_duration_ms = 10000;

// Functions (needs .h)
extern uint32_t sl_sleeptimer_get_timer_frequency(void);
void handleNodeRxChange(uint8_t *data, size_t len);
void compileCommandString(char *commandStr);
void setTxBufferGain(void);
void setTxBufferVolts(float outputVolts);
void transferTxBuffer(void);

// SPI/DAC
#define SPI_HANDLE sl_spidrv_inst_handle
static const float refVolts = 1.25;
static const uint16_t maxADCValue = 65535;

// Add near other global variables
static bool advertising_enabled = false;

// LED regimes: ready 1 Hz chirp, connected solid ON, stim 10 s heartbeat only
typedef enum {
	LED_REGIME_READY,
	LED_REGIME_CONNECTED,
	LED_REGIME_STIM,
} led_regime_t;

#define LED_BOOT_CHIRP_INTERVAL_MS       1000
#define LED_BOOT_CHIRP_DURATION_MS       50
#define LED_STIM_HEARTBEAT_INTERVAL_MS   10000
#define LED_STIM_HEARTBEAT_DURATION_MS   50
#define LED_STIM_HEARTBEAT_DEFER_MS      10
#define LED_STIM_HEARTBEAT_DEFER_MAX_MS  200

static led_regime_t led_regime = LED_REGIME_READY;
static bool ble_connected = false;
static bool advertising_active = false;
static bool led_user_on = true;
static bool led_chirp_phase_on = false;
static bool led_stim_pulse_active = false;
static bool led_stim_heartbeat_active = false;
static uint32_t led_stim_defer_accum_ms = 0;

static sl_sleeptimer_timer_handle_t led_chirp_timer;
static sl_sleeptimer_timer_handle_t led_stim_timer;

static void led_apply_output(bool on);
static void led_refresh(void);
static void led_set_regime(led_regime_t regime);
static void led_enter_stim_idle(void);
static void led_update_stim_heartbeat(void);
static void enter_ready_to_connect(void);
static void on_magnet_detected(void);
static bool stim_chain_busy(void);
static void led_stim_heartbeat_start(void);
static void led_stim_heartbeat_stop(void);
static void on_led_boot_chirp_timeout(sl_sleeptimer_timer_handle_t *handle, void *data);
static void on_led_stim_heartbeat_timeout(sl_sleeptimer_timer_handle_t *handle, void *data);

void gpio_init(void);

#define IADC_REFERENCE_VOLTAGE 3.3f

void nvm3_init(void)
{
	Ecode_t result = nvm3_initDefault();
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

void update_cap_id(uint8_t new_cap_id)
{
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_CAP_ID,
									&new_cap_id, sizeof(new_cap_id));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

void update_amplitude(int new_amplitude)
{
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_AMPLITUDE,
									&new_amplitude, sizeof(new_amplitude));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

void update_frequency(int new_frequency)
{
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_FREQUENCY,
									&new_frequency, sizeof(new_frequency));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

void update_pulse_duration(int new_pulse_duration)
{
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_PULSE_DURATION,
									&new_pulse_duration, sizeof(new_pulse_duration));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

void update_stim_mode(int new_stim_mode)
{
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_STIM_MODE,
									&new_stim_mode, sizeof(new_stim_mode));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

void update_burst_period_ms(int new_burst_period_ms)
{
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_BURST_PERIOD_MS,
									&new_burst_period_ms, sizeof(new_burst_period_ms));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

void update_intra_burst_freq_hz(int new_intra_burst_freq_hz)
{
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_INTRA_BURST_FREQ_HZ,
									&new_intra_burst_freq_hz, sizeof(new_intra_burst_freq_hz));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

void update_burst_duration_ms(int new_burst_duration_ms)
{
	if (new_burst_duration_ms > burst_period_ms)
	{
		new_burst_duration_ms = burst_period_ms;
	}
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_BURST_DURATION_MS,
									&new_burst_duration_ms, sizeof(new_burst_duration_ms));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error
	}
}

uint8_t read_cap_id(void)
{
	uint8_t cap_id = 0x00;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_CAP_ID, &cap_id,
								   sizeof(cap_id));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error or set default value
		cap_id = 0x00;
	}
	return cap_id;
}

int read_amplitude(void)
{
	int amplitude = 0;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_AMPLITUDE,
								   &amplitude, sizeof(amplitude));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error or set default value
		amplitude = 0;
	}
	return amplitude;
}

int read_frequency(void)
{
	int frequency = 0;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_FREQUENCY,
								   &frequency, sizeof(frequency));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error or set default value
		frequency = 0;
	}
	return frequency;
}

int read_pulse_duration(void)
{
	int pulse_duration = 0;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_PULSE_DURATION,
								   &pulse_duration, sizeof(pulse_duration));
	if (result != ECODE_NVM3_OK)
	{
		// Handle error or set default value
		pulse_duration = 0;
	}
	return pulse_duration;
}

int read_stim_mode(void)
{
	int mode = 0;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_STIM_MODE, &mode,
								   sizeof(mode));
	if (result != ECODE_NVM3_OK)
	{
		mode = 0;
	}
	return mode;
}

int read_burst_period_ms(void)
{
	int period = 30000;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_BURST_PERIOD_MS,
								   &period, sizeof(period));
	if (result != ECODE_NVM3_OK)
	{
		period = 30000;
	}
	return period;
}

int read_intra_burst_freq_hz(void)
{
	int freq = 130;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_INTRA_BURST_FREQ_HZ,
								   &freq, sizeof(freq));
	if (result != ECODE_NVM3_OK)
	{
		freq = 130;
	}
	return freq;
}

int read_burst_duration_ms(void)
{
	int duration = 10000;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_BURST_DURATION_MS,
								   &duration, sizeof(duration));
	if (result != ECODE_NVM3_OK)
	{
		duration = 10000;
	}
	return duration;
}

// scale based on 0-100% for amplitude, -V=I
float calcStimVolts(void)
{
	float stimVolts = refVolts - refVolts * (amplitude / 100.0);
	return stimVolts;
}

float calcChargeBalanceVolts(void)
{
	float stimVolts = calcStimVolts();
	float chargeBalanceVolts = refVolts + ((refVolts - stimVolts) / CHARGE_BALANCE_RATIO);
	return chargeBalanceVolts;
}

// void transfer_callback(SPIDRV_HandleData_t *handle, Ecode_t transfer_status,
//		int items_transferred) {
//	(void) handle;
//	(void) items_transferred;
//
//	// Post semaphore to signal to application task that transfer is successful
//	if (transfer_status == ECODE_EMDRV_SPIDRV_OK) {
//		transfer_complete = true;
//	}
// }

void spidrv_app_init(void)
{
	setTxBufferGain();
	setTxBufferVolts(refVolts);
}

void setTxBufferGain(void)
{
	// gain x2, internal ref off
	tx_buffer[0] = 0x41;
	tx_buffer[1] = 0x80;
	tx_buffer[2] = 0x00;
	transferTxBuffer();
}

void setTxBufferVolts(float outputVolts)
{
	uint16_t ADC_bits =
		(uint16_t)(((outputVolts / refVolts) / 2) * maxADCValue); // divide by 2 for gain
	tx_buffer[0] = (1 << 4) | (ADC_bits >> 12);					  // Set upper nibble to 1, then add top 4 bits of ADC_bits
	tx_buffer[1] = (ADC_bits >> 4) & 0xFF;						  // Middle 8 bits of ADC_bits
	tx_buffer[2] = (ADC_bits << 4) & 0xF0;						  // Last 4 bits of ADC_bits shifted to the top, lower 4 bits are 0
	transferTxBuffer();
}

void transferTxBuffer(void)
{
	Ecode_t ecode;
	ecode = SPIDRV_MTransferB(SPI_HANDLE, tx_buffer, rx_buffer,
							  APP_BUFFER_SIZE); // transfer_callback
	EFM_ASSERT(ecode == ECODE_OK);
}

static void enable_charge_balancing(void)
{
	float chargeBalanceVolts = calcChargeBalanceVolts();
	setTxBufferVolts(chargeBalanceVolts);
}

static uint32_t getPulseDurationTicks(uint32_t pulse_duration_us)
{
	uint32_t timer_frequency = sl_sleeptimer_get_timer_frequency();
	return (pulse_duration_us * timer_frequency) / 1000000;
}

static void on_pulse_timeout(sl_sleeptimer_timer_handle_t *handle, void *data);

// EM1 requirement held for the entire stim train (start_stimulation ->
// stop_stimulation). Field-validated units never slept below EM1, so this
// replicates their timing environment exactly: every sleeptimer edge in the
// pulse chain fires with EM1 latency, never EM2 wake latency. Temporary
// power trade-off (~+0.5 mA during stim); revisit tighter windowing once the
// waveform is validated. Guarded flag keeps add/remove balanced.
static bool em1_requirement_active = false;

static void stim_em1_acquire(void)
{
	if (!em1_requirement_active)
	{
		em1_requirement_active = true;
		sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
	}
}

static void stim_em1_release(void)
{
	if (em1_requirement_active)
	{
		em1_requirement_active = false;
		sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
	}
}

// Busy-wait settle (field-validated behavior): keeps the SHDN -> electrode
// enable interval deterministic instead of exposing it to sleeptimer jitter.
static void delay_microseconds(uint32_t us)
{
	uint32_t ticks = (us * sl_sleeptimer_get_timer_frequency()) / 1000000;
	uint32_t start_tick = sl_sleeptimer_get_tick_count();

	while ((sl_sleeptimer_get_tick_count() - start_tick) < ticks)
	{
		// Do nothing, just wait
	}
}

// Zero-current transition sequencing: the ADG1236 switches only open/close
// while the DAC commands refVolts (zero Howland current), and the pulse edges
// are produced by DAC steps into an already-connected load. This avoids
// connecting a saturated open-circuit current source (onset transient) and
// disconnecting mid-current (turn-off transient).
static void enable_stimulation_output(void)
{
	float stimVolts = calcStimVolts();

	GPIO_PinOutSet(SHDN_PORT, SHDN_PIN);
	// DAC is at refVolts here (zero current): give the amp a generous wake
	// settle, connect the electrodes at zero, let the switch settle, then
	// drive the pulse edge with the DAC step.
	delay_microseconds(OP_AMP_WAKE_SETTLE_US);
	GPIO_PinOutSet(ELEC0_OUT_PORT, ELEC0_OUT_PIN);
	GPIO_PinOutSet(ELEC1_OUT_PORT, ELEC1_OUT_PIN);
	delay_microseconds(SWITCH_CONNECT_SETTLE_US);
	setTxBufferVolts(stimVolts);

	uint32_t nonzero_pulse_duration =
		(pulse_duration > STIM_OFFSET_US) ? (pulse_duration - STIM_OFFSET_US) : 0;
	uint32_t pulse_duration_ticks = getPulseDurationTicks(
		nonzero_pulse_duration);

	sl_sleeptimer_start_timer(&timer_pulse, pulse_duration_ticks,
							  on_pulse_timeout, NULL, 0, 0);
}

static void disable_output(void)
{
	// Return to zero current while still connected, let the output settle,
	// then disconnect and shut the amp down with the electrodes isolated.
	setTxBufferVolts(refVolts);
	delay_microseconds(SLEEP_TIMER_TICK_US);
	GPIO_PinOutClear(ELEC0_OUT_PORT, ELEC0_OUT_PIN);
	GPIO_PinOutClear(ELEC1_OUT_PORT, ELEC1_OUT_PIN);
#if !STIM_HOLD_SHDN
	GPIO_PinOutClear(SHDN_PORT, SHDN_PIN);
#endif
}

static void on_charge_balance_timeout(sl_sleeptimer_timer_handle_t *handle,
									  void *data)
{
	(void)handle;
	(void)data;
	disable_output(); // Disable output after charge balancing
}

static void on_pulse_timeout(sl_sleeptimer_timer_handle_t *handle, void *data)
{
	(void)handle;
	(void)data;

	enable_charge_balancing();

	uint32_t adj_pulse_duration = pulse_duration * CHARGE_BALANCE_RATIO;
	uint32_t nonzero_pulse_duration =
		(adj_pulse_duration > CHARGE_BALANCE_OFFSET_US) ? (adj_pulse_duration - CHARGE_BALANCE_OFFSET_US) : 0;
	uint32_t pulse_duration_ticks = getPulseDurationTicks(
		nonzero_pulse_duration);

	// Start charge balancing using CHARGE_BALANCE_RATIO multiplier
	sl_sleeptimer_start_timer(&timer_charge_balance, pulse_duration_ticks,
							  on_charge_balance_timeout, NULL, 0, 0);
}

static void on_stimulation_timeout(sl_sleeptimer_timer_handle_t *handle,
								   void *data);

static void on_burst_duration_end(sl_sleeptimer_timer_handle_t *handle, void *data);

static void on_burst_period(sl_sleeptimer_timer_handle_t *handle, void *data);

static void on_burst_onset(void);

static void stop_pulse_chain(void)
{
	sl_sleeptimer_stop_timer(&timer_stimulation);
	sl_sleeptimer_stop_timer(&timer_pulse);
	sl_sleeptimer_stop_timer(&timer_charge_balance);
}

static void start_continuous_stimulation(void)
{
	sl_status_t status;
	bool isRunning = false;

	status = sl_sleeptimer_is_timer_running(&timer_stimulation, &isRunning);
	if (status == SL_STATUS_OK && !isRunning && frequency > 0)
	{
		sl_sleeptimer_start_periodic_timer(&timer_stimulation,
										   sl_sleeptimer_ms_to_tick(1000 / frequency),
										   on_stimulation_timeout, NULL, 0, 0);
	}
}

static void on_burst_duration_end(sl_sleeptimer_timer_handle_t *handle, void *data)
{
	(void)handle;
	(void)data;

	// Stop scheduling new pulses only; an in-flight pulse completes its own
	// chain (pulse -> charge balance -> disable_output), preserving charge
	// balance instead of truncating mid-phase.
	sl_sleeptimer_stop_timer(&timer_stimulation);
}

static void on_burst_onset(void)
{
	sl_sleeptimer_stop_timer(&timer_burst_duration);
	sl_sleeptimer_stop_timer(&timer_stimulation);

	if (intra_burst_freq_hz <= 0)
	{
		return;
	}

	sl_sleeptimer_start_periodic_timer(&timer_stimulation,
									   sl_sleeptimer_ms_to_tick(1000 / intra_burst_freq_hz),
									   on_stimulation_timeout, NULL, 0, 0);
	sl_sleeptimer_start_timer_ms(&timer_burst_duration, (uint32_t)burst_duration_ms,
								 on_burst_duration_end, NULL, 0, 0);
}

static void on_burst_period(sl_sleeptimer_timer_handle_t *handle, void *data)
{
	(void)handle;
	(void)data;

	on_burst_onset();
}

static void start_burst_cycle(void)
{
	bool isRunning = false;

	if (sl_sleeptimer_is_timer_running(&timer_burst_period, &isRunning) == SL_STATUS_OK
		&& isRunning)
	{
		return;
	}

	on_burst_onset();
	sl_sleeptimer_start_periodic_timer(&timer_burst_period,
									   sl_sleeptimer_ms_to_tick((uint32_t)burst_period_ms),
									   on_burst_period, NULL, 0, 0);
}

void start_stimulation(void)
{
	stim_em1_acquire();
	if (stim_mode == 0)
	{
		start_continuous_stimulation();
	}
	else
	{
		start_burst_cycle();
	}
	led_update_stim_heartbeat();
}

static void start_advertising(void)
{
	if (advertising_set_handle == 0xff)
	{
		return;
	}

	sl_status_t sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
												   sl_bt_legacy_advertiser_connectable);
	app_assert_status(sc);
	if (sc == SL_STATUS_OK)
	{
		advertising_active = true;
	}
}

void stop_stimulation(void)
{
	led_stim_heartbeat_stop();
	sl_sleeptimer_stop_timer(&timer_burst_period);
	sl_sleeptimer_stop_timer(&timer_burst_duration);
	stop_pulse_chain();
	disable_output();
	// End of train: amp always shuts down, regardless of STIM_HOLD_SHDN.
	GPIO_PinOutClear(SHDN_PORT, SHDN_PIN);
	stim_em1_release();
}

static void on_stimulation_timeout(sl_sleeptimer_timer_handle_t *handle,
								   void *data)
{
	(void)handle;
	(void)data;

	enable_stimulation_output();
}

/**************************************************************************/ /**
																			  * Application Init.
																			  *****************************************************************************/
SL_WEAK void app_init(void)
{
	/////////////////////////////////////////////////////////////////////////////
	// Put your additional application init code here!                         //
	// This is called once during start-up.                                    //
	/////////////////////////////////////////////////////////////////////////////
	GPIO_PinModeSet(SHDN_PORT, SHDN_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(MAG_PORT, MAG_PIN, gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(ELEC0_OUT_PORT, ELEC0_OUT_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(ELEC1_OUT_PORT, ELEC1_OUT_PIN, gpioModePushPull, 0);

	nvm3_init();
	cap_id = read_cap_id();
	amplitude = read_amplitude();
	frequency = read_frequency();
	pulse_duration = read_pulse_duration();
	stim_mode = read_stim_mode();
	burst_period_ms = read_burst_period_ms();
	intra_burst_freq_hz = read_intra_burst_freq_hz();
	burst_duration_ms = read_burst_duration_ms();
	if (burst_duration_ms > burst_period_ms)
	{
		burst_duration_ms = burst_period_ms;
	}

	spidrv_app_init();
	gpio_init();
	iadc_init();

	led_set_regime(LED_REGIME_READY);
}

/**************************************************************************/ /**
																			  * Application Process Action.
																			  *****************************************************************************/
SL_WEAK void app_process_action(void)
{
	/////////////////////////////////////////////////////////////////////////////
	// Put your additional application code here!                              //
	// This is called infinitely.                                              //
	// Do not call blocking functions from here!                               //
	/////////////////////////////////////////////////////////////////////////////

	// Check if we should start advertising
	if (advertising_enabled && advertising_set_handle != 0xff)
	{
		advertising_enabled = false;
		start_advertising();
	}
}

/**************************************************************************/ /**
																			  * Bluetooth stack event handler.
																			  * This overrides the dummy weak implementation.
																			  *
																			  * @param[in] evt Event coming from the Bluetooth stack.
																			  *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
	sl_status_t sc;
	int16_t pwr_min, pwr_max;

	switch (SL_BT_MSG_ID(evt->header))
	{
	// -------------------------------
	// This event indicates the device has started and the radio is ready.
	// Do not call any stack command before receiving this boot event!
	case sl_bt_evt_system_boot_id:
		// Set TX power to reasonable level
		sc = sl_bt_system_set_tx_power(0, 0, &pwr_min, &pwr_max);
		app_assert_status(sc);

		// Allow large ATT payloads; stack requests MTU exchange on connect when > 23
		{
			uint16_t max_mtu_out;
			sc = sl_bt_gatt_server_set_max_mtu(ATT_MTU_MAX, &max_mtu_out);
			app_assert_status(sc);
		}

		// Create an advertising set.
		sc = sl_bt_advertiser_create_set(&advertising_set_handle);
		app_assert_status(sc);

		// Generate data for advertising
		sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
												   sl_bt_advertiser_general_discoverable);
		app_assert_status(sc);

		// Set advertising interval to 100ms.
		sc = sl_bt_advertiser_set_timing(advertising_set_handle,
										 ADV_INTERVAL_MS,
										 ADV_INTERVAL_MS,
										 0,
										 0);
		app_assert_status(sc);

		// Start advertising immediately — 1 Hz chirp means ready to connect
		start_advertising();
		break;

		// -------------------------------
		// This event indicates that a new connection was opened.
	case sl_bt_evt_connection_opened_id:
		advertising_active = false;
		ble_connected = true;
		led_user_on = true;
		led_set_regime(LED_REGIME_CONNECTED);
		led_update_stim_heartbeat();
		break;

		// -------------------------------
		// This event indicates that a connection was closed.
	case sl_bt_evt_connection_closed_id:
		ble_connected = false;
		if (activateStim)
		{
			led_enter_stim_idle();
			start_stimulation();
		}
		else
		{
			enter_ready_to_connect();
		}
		break;

		// -------------------------------
		// This event indicates that the value of an attribute in the local GATT
		// database was changed by a remote GATT client.
	case sl_bt_evt_gatt_server_attribute_value_id:
		if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_node_rx)
		{
			// Use dynamic size based on actual data
			uint8_t buffer[COMMAND_STR_MAX_SIZE];
			size_t len = 0;
			sc = sl_bt_gatt_server_read_attribute_value(
				gattdb_node_rx, 0, sizeof(buffer), &len, buffer);

			if (sc == SL_STATUS_OK)
			{
				handleNodeRxChange(buffer, len);
				compileCommandString(commandStr);
				size_t tx_len = strlen(commandStr);
				sc = sl_bt_gatt_server_write_attribute_value(
					gattdb_node_tx, 0, tx_len, (uint8_t *)commandStr);
			}
			else
			{
				// Handle error
			}
		}
		break;

		// -------------------------------
		// This event occurs when the remote device enabled or disabled the
		// notification.
	case sl_bt_evt_gatt_server_characteristic_status_id:
		break;

	case sl_bt_evt_gatt_mtu_exchanged_id:
		negotiated_mtu = evt->data.evt_gatt_mtu_exchanged.mtu;
		break;

		///////////////////////////////////////////////////////////////////////////
		// Add additional event handlers here as your application requires!      //
		///////////////////////////////////////////////////////////////////////////

		// -------------------------------
		// Default event handler.
	default:
		break;
	}
}

static const char *const PROTO_KEYS[] = {
	"BP", "IF", "BD", "FW", "M", "A", "F", "P", "G", "N", "L", NULL
};

static int clamp_int(int value, int min_val, int max_val)
{
	if (value < min_val)
	{
		return min_val;
	}
	if (value > max_val)
	{
		return max_val;
	}
	return value;
}

static bool parse_token(const char *token, const char **out_key, int *out_value)
{
	for (int i = 0; PROTO_KEYS[i] != NULL; i++)
	{
		const char *key = PROTO_KEYS[i];
		size_t key_len = strlen(key);

		if (strncmp(token, key, key_len) == 0)
		{
			*out_key = key;
			*out_value = atoi(token + key_len);
			return true;
		}
	}
	return false;
}

static void apply_param(const char *key, int value)
{
	if (strcmp(key, "M") == 0)
	{
		int mode = clamp_int(value, STIM_MODE_MIN, STIM_MODE_MAX);
		if (mode != stim_mode)
		{
			stim_mode = mode;
			update_stim_mode(stim_mode);
		}
	}
	else if (strcmp(key, "A") == 0)
	{
		int amp = clamp_int(value, AMPLITUDE_MIN, AMPLITUDE_MAX);
		if (amp != amplitude)
		{
			amplitude = amp;
			update_amplitude(amplitude);
		}
	}
	else if (strcmp(key, "F") == 0)
	{
		int freq = clamp_int(value, FREQUENCY_MIN, FREQUENCY_MAX);
		if (freq != frequency)
		{
			frequency = freq;
			update_frequency(frequency);
		}
	}
	else if (strcmp(key, "P") == 0)
	{
		int pulse = clamp_int(value, PULSE_DURATION_MIN, PULSE_DURATION_MAX);
		if (pulse != pulse_duration)
		{
			pulse_duration = pulse;
			update_pulse_duration(pulse_duration);
		}
	}
	else if (strcmp(key, "BP") == 0)
	{
		int period = clamp_int(value, BURST_PERIOD_MS_MIN, BURST_PERIOD_MS_MAX);
		if (period != burst_period_ms)
		{
			burst_period_ms = period;
			update_burst_period_ms(burst_period_ms);
			if (burst_duration_ms > burst_period_ms)
			{
				burst_duration_ms = burst_period_ms;
				update_burst_duration_ms(burst_duration_ms);
			}
		}
	}
	else if (strcmp(key, "IF") == 0)
	{
		int freq = clamp_int(value, INTRA_BURST_FREQ_MIN, INTRA_BURST_FREQ_MAX);
		if (freq != intra_burst_freq_hz)
		{
			intra_burst_freq_hz = freq;
			update_intra_burst_freq_hz(intra_burst_freq_hz);
		}
	}
	else if (strcmp(key, "BD") == 0)
	{
		int duration = clamp_int(value, BURST_DURATION_MS_MIN, BURST_DURATION_MS_MAX);
		if (duration > burst_period_ms)
		{
			duration = burst_period_ms;
		}
		if (duration != burst_duration_ms)
		{
			burst_duration_ms = duration;
			update_burst_duration_ms(burst_duration_ms);
		}
	}
	else if (strcmp(key, "G") == 0)
	{
		activateStim = (uint8_t)clamp_int(value, 0, 1);
	}
	else if (strcmp(key, "N") == 0)
	{
		int id = clamp_int(value, CAP_ID_MIN, CAP_ID_MAX);
		if (id != cap_id)
		{
			cap_id = (uint8_t)id;
			update_cap_id(cap_id);
		}
	}
	else if (strcmp(key, "L") == 0)
	{
		if (value && ble_connected)
		{
			led_user_on = !led_user_on;
			led_refresh();
		}
	}
	// FW is read-only (device → app); ignore if received
}

void compileCommandString(char *commandStr)
{
	if (commandStr == NULL)
	{
		return;
	}

	memset(commandStr, 0, COMMAND_STR_MAX_SIZE);
	uint32_t voltage_mv = read_battery_voltage();

	if (stim_mode == 0)
	{
		snprintf(commandStr, COMMAND_STR_MAX_SIZE,
				 "_A%u,F%u,P%u,M0,G%u,N%u,V%lu,FW%u",
				 amplitude, frequency, pulse_duration, activateStim, cap_id,
				 voltage_mv, FIRMWARE_VERSION_WIRE);
	}
	else
	{
		snprintf(commandStr, COMMAND_STR_MAX_SIZE,
				 "_A%u,P%u,M1,BP%u,IF%u,BD%u,G%u,N%u,V%lu,FW%u",
				 amplitude, pulse_duration, burst_period_ms, intra_burst_freq_hz,
				 burst_duration_ms, activateStim, cap_id, voltage_mv,
				 FIRMWARE_VERSION_WIRE);
	}
}

void handleNodeRxChange(uint8_t *data, size_t len)
{
	if (len == 0 || data[0] != '_')
	{
		return;
	}

	if (len == 2 && data[1] == '1')
	{
		return;
	}
	if (len == 3 && data[1] == 'L' && data[2] == '1')
	{
		if (ble_connected)
		{
			led_user_on = !led_user_on;
			led_refresh();
		}
		return;
	}

	char str[len + 1];
	memset(str, 0, len + 1);
	memcpy(str, data, len);
	str[len] = '\0';

	char *token = strtok(str + 1, ",");
	while (token != NULL)
	{
		const char *key = NULL;
		int value = 0;

		if (parse_token(token, &key, &value))
		{
			apply_param(key, value);
		}

		token = strtok(NULL, ",");
	}

	if (activateStim)
	{
		stop_stimulation();
		start_stimulation();
	}
	else
	{
		stop_stimulation();
	}
}

static void led_apply_output(bool on)
{
	if (on)
	{
		sl_led_turn_on(LED0);
	}
	else
	{
		sl_led_turn_off(LED0);
	}
}

static bool stim_chain_busy(void)
{
	bool running = false;

	if (sl_sleeptimer_is_timer_running(&timer_pulse, &running) == SL_STATUS_OK
		&& running)
	{
		return true;
	}
	if (sl_sleeptimer_is_timer_running(&timer_charge_balance, &running) == SL_STATUS_OK
		&& running)
	{
		return true;
	}
	return false;
}

static void led_refresh(void)
{
	if (led_regime == LED_REGIME_CONNECTED)
	{
		led_apply_output(led_user_on);
	}
}

static void led_stop_boot_chirp(void)
{
	sl_sleeptimer_stop_timer(&led_chirp_timer);
	led_chirp_phase_on = false;
}

static void led_start_boot_chirp(void)
{
	led_stop_boot_chirp();
	led_chirp_phase_on = true;
	sl_sleeptimer_start_timer_ms(&led_chirp_timer,
								 LED_BOOT_CHIRP_INTERVAL_MS,
								 on_led_boot_chirp_timeout,
								 NULL,
								 0,
								 0);
}

static void led_set_regime(led_regime_t regime)
{
	led_regime = regime;
	led_stop_boot_chirp();

	if (regime == LED_REGIME_READY)
	{
		led_apply_output(false);
		led_start_boot_chirp();
	}
	else if (regime == LED_REGIME_STIM)
	{
		led_apply_output(false);
	}
	else
	{
		led_refresh();
	}
}

static void led_update_stim_heartbeat(void)
{
	if (activateStim && !advertising_active && led_regime != LED_REGIME_READY)
	{
		led_stim_heartbeat_start();
	}
	else
	{
		led_stim_heartbeat_stop();
	}
}

static void led_enter_stim_idle(void)
{
	led_stop_boot_chirp();
	led_apply_output(false);
	led_regime = LED_REGIME_STIM;
	advertising_active = false;
	led_update_stim_heartbeat();
}

static void enter_ready_to_connect(void)
{
	advertising_enabled = true;
	led_stim_heartbeat_stop();
	led_set_regime(LED_REGIME_READY);
}

static void on_magnet_detected(void)
{
	enter_ready_to_connect();
}

static void on_led_boot_chirp_timeout(sl_sleeptimer_timer_handle_t *handle, void *data)
{
	(void)handle;
	(void)data;

	if (led_regime != LED_REGIME_READY || led_stim_pulse_active)
	{
		return;
	}

	if (led_chirp_phase_on)
	{
		led_apply_output(true);
		led_chirp_phase_on = false;
		sl_sleeptimer_start_timer_ms(&led_chirp_timer,
									 LED_BOOT_CHIRP_DURATION_MS,
									 on_led_boot_chirp_timeout,
									 NULL,
									 0,
									 0);
	}
	else
	{
		led_apply_output(false);
		led_chirp_phase_on = true;
		sl_sleeptimer_start_timer_ms(&led_chirp_timer,
									 LED_BOOT_CHIRP_INTERVAL_MS,
									 on_led_boot_chirp_timeout,
									 NULL,
									 0,
									 0);
	}
}

static void led_stim_heartbeat_stop(void)
{
	led_stim_heartbeat_active = false;
	led_stim_pulse_active = false;
	led_stim_defer_accum_ms = 0;
	sl_sleeptimer_stop_timer(&led_stim_timer);
	if (led_regime == LED_REGIME_CONNECTED)
	{
		led_refresh();
	}
	else
	{
		led_apply_output(false);
	}
}

static void led_stim_heartbeat_start(void)
{
	if (led_stim_heartbeat_active)
	{
		return;
	}
	led_stim_heartbeat_active = true;
	led_stim_pulse_active = false;
	led_stim_defer_accum_ms = 0;
	sl_sleeptimer_start_timer_ms(&led_stim_timer,
								 LED_STIM_HEARTBEAT_INTERVAL_MS,
								 on_led_stim_heartbeat_timeout,
								 NULL,
								 0,
								 0);
}

static void on_led_stim_heartbeat_timeout(sl_sleeptimer_timer_handle_t *handle, void *data)
{
	(void)handle;
	(void)data;

	if (!led_stim_heartbeat_active)
	{
		return;
	}

	if (led_stim_pulse_active)
	{
		led_stim_pulse_active = false;
		if (led_regime == LED_REGIME_CONNECTED)
		{
			led_refresh();
		}
		else
		{
			led_apply_output(false);
		}
		sl_sleeptimer_start_timer_ms(&led_stim_timer,
									 LED_STIM_HEARTBEAT_INTERVAL_MS,
									 on_led_stim_heartbeat_timeout,
									 NULL,
									 0,
									 0);
		return;
	}

	if (stim_chain_busy()
		&& led_stim_defer_accum_ms < LED_STIM_HEARTBEAT_DEFER_MAX_MS)
	{
		led_stim_defer_accum_ms += LED_STIM_HEARTBEAT_DEFER_MS;
		sl_sleeptimer_start_timer_ms(&led_stim_timer,
									 LED_STIM_HEARTBEAT_DEFER_MS,
									 on_led_stim_heartbeat_timeout,
									 NULL,
									 0,
									 0);
		return;
	}
	led_stim_defer_accum_ms = 0;

	led_stim_pulse_active = true;
	if (ble_connected && led_user_on)
	{
		led_apply_output(false);
	}
	else
	{
		led_apply_output(true);
	}
	sl_sleeptimer_start_timer_ms(&led_stim_timer,
								 LED_STIM_HEARTBEAT_DURATION_MS,
								 on_led_stim_heartbeat_timeout,
								 NULL,
								 0,
								 0);
}

void gpio_init(void)
{
	// Configure interrupt for MAG_PIN with filter
	GPIO_ExtIntConfig(MAG_PORT, MAG_PIN, MAG_PIN, false, true, true);

	// Enable wake-up from EM2/EM3
	GPIO_EM4EnablePinWakeup(MAG_PIN << _GPIO_EM4WUEN_EM4WUEN_SHIFT, 0);

	// Clear and enable interrupts
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

void GPIO_EVEN_IRQHandler(void)
{
	// Clear interrupt flag
	uint32_t flags = GPIO_IntGet();
	GPIO_IntClear(flags);

	if (flags & (1 << MAG_PIN))
	{
		on_magnet_detected();
	}
}

void GPIO_ODD_IRQHandler(void)
{
	// Clear interrupt flag
	uint32_t flags = GPIO_IntGet();
	GPIO_IntClear(flags);

	if (flags & (1 << MAG_PIN))
	{
		on_magnet_detected();
	}
}

void iadc_init(void)
{
	// Declare initialization structures
	IADC_Init_t init = IADC_INIT_DEFAULT;
	IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
	IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
	IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

	// Enable IADC clock
	CMU_ClockEnable(cmuClock_IADC0, true);

	// Select clock for IADC
	CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO); // FSRCO - 20MHz

	// Modify init structs
	init.warmup = iadcWarmupNormal;

	// Set the prescale value and clock
	init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, 20000000, 0);

	// Configuration 0 for single conversion
	initAllConfigs.configs[0].reference = iadcCfgReferenceVddx; // Use VDD as reference
	initAllConfigs.configs[0].vRef = 3300;						// 3.3V reference
	initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
																	   10000000, // 10MHz ADC clock
																	   0,
																	   iadcCfgModeNormal,
																	   init.srcClkPrescale);

	// Single input configuration
	initSingleInput.posInput = iadcPosInputPortCPin2; // PC02 - VBATT
	initSingleInput.negInput = iadcNegInputGnd;		  // Single-ended mode

	// Initialize IADC
	IADC_init(IADC0, &init, &initAllConfigs);
	IADC_initSingle(IADC0, &initSingle, &initSingleInput);

	// Allocate the analog bus for ADC0 inputs
	GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0; // For PC02
}

uint32_t read_battery_voltage(void)
{
	// Start conversion
	IADC_command(IADC0, iadcCmdStartSingle);

	// Wait for conversion to complete
	while ((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK | _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV)
		;

	// Get ADC result
	IADC_Result_t result = IADC_pullSingleFifoResult(IADC0);

	// Convert to millivolts (12-bit ADC)
	uint32_t voltage_mv = (result.data * 3300) / 0xFFF;

	return voltage_mv;
}
