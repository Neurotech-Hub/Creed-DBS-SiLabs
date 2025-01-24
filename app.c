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

// TIMING
#define SLEEP_TIMER_TICK_US 30
#define OP_AMP_INIT_TICKS 4
#define STIM_OFFSET_US 93			// empirical
#define CHARGE_BALANCE_OFFSET_US 36 // empirical
#define ADV_INTERVAL_MS 2000
sl_sleeptimer_timer_handle_t timer_stimulation;	   // based on frequency
sl_sleeptimer_timer_handle_t timer_pulse;		   // based on amplitude
sl_sleeptimer_timer_handle_t timer_charge_balance; // balance amplitude

// STIM
#define CHARGE_BALANCE_RATIO 5

// HARDWARE
#define LED0 (&sl_led_inst)
#define NVM3_KEY_CAP_ID 0x00000001UL
#define NVM3_KEY_AMPLITUDE 0x00000002UL
#define NVM3_KEY_FREQUENCY 0x00000003UL
#define NVM3_KEY_PULSE_DURATION 0x00000004UL
uint8_t cap_id;

// BLE
#define COMMAND_STR_MAX_SIZE 20 // Maximum possible MTU size
static uint8_t advertising_set_handle = 0xff;
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

// Functions (needs .h)
extern uint32_t sl_sleeptimer_get_timer_frequency(void);
void handleNodeRxChange(uint8_t *data, size_t len);
void compileCommandString(char *commandStr);
void setTxBufferGain(void);
void setTxBufferVolts(float outputVolts);
void transferTxBuffer(void);
void delay_microseconds(uint32_t us);

// SPI/DAC
#define SPI_HANDLE sl_spidrv_inst_handle
static const float refVolts = 1.25;
static const uint16_t maxADCValue = 65535;

// Add near other global variables
static bool advertising_enabled = false;

// Add near other defines at top
#define LED_FLASH_INTERVAL_MS 2000 // Flash every 2 seconds during advertising
#define LED_FLASH_DURATION_MS 50   // Flash duration of 50ms

// Add near other global variables
static sl_sleeptimer_timer_handle_t led_flash_timer;
static bool led_is_flashing = false;

// Add these function declarations near the top of the file, after the other function declarations
void gpio_init(void);
static void on_led_flash_timeout(sl_sleeptimer_timer_handle_t *handle, void *data);

// Add near other defines
#define IADC_REFERENCE_VOLTAGE 3.3f

// Add near other global variables
static uint8_t txCharSwitch = 1; // Default to first set of values

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

void delay_microseconds(uint32_t us)
{
	// Convert microseconds to timer ticks
	uint32_t ticks = (us * sl_sleeptimer_get_timer_frequency()) / 1000000;

	// Get the current tick count
	uint32_t start_tick = sl_sleeptimer_get_tick_count();

	// Wait until the specified number of ticks have passed
	while ((sl_sleeptimer_get_tick_count() - start_tick) < ticks)
	{
		// Do nothing, just wait
	}
}

static void enable_charge_balancing(void)
{
	float chargeBalanceVolts = calcChargeBalanceVolts();
	setTxBufferVolts(chargeBalanceVolts);
}

static void enable_stimulation_output(void)
{
	float stimVolts = calcStimVolts();
	GPIO_PinOutSet(SHDN_PORT, SHDN_PIN);
	setTxBufferVolts(stimVolts);
	delay_microseconds(SLEEP_TIMER_TICK_US * OP_AMP_INIT_TICKS);
	GPIO_PinOutSet(ELEC0_OUT_PORT, ELEC0_OUT_PIN);
	GPIO_PinOutSet(ELEC1_OUT_PORT, ELEC1_OUT_PIN);
}

static void disable_output(void)
{
	GPIO_PinOutClear(ELEC0_OUT_PORT, ELEC0_OUT_PIN);
	GPIO_PinOutClear(ELEC1_OUT_PORT, ELEC1_OUT_PIN);
	GPIO_PinOutClear(SHDN_PORT, SHDN_PIN);
	setTxBufferVolts(refVolts);
}

static void on_charge_balance_timeout(sl_sleeptimer_timer_handle_t *handle,
									  void *data)
{
	(void)handle;
	(void)data;
	disable_output(); // Disable output after charge balancing
}

static uint32_t getPulseDurationTicks(uint32_t pulse_duration)
{
	// Calculate the number of ticks for pulse_duration in µs
	uint32_t timer_frequency = sl_sleeptimer_get_timer_frequency();
	uint32_t pulse_duration_ticks = (pulse_duration * timer_frequency) / 1000000;
	return pulse_duration_ticks;
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
								   void *data)
{
	(void)handle;
	(void)data;

	enable_stimulation_output(); // Start the stimulation pulse

	uint32_t nonzero_pulse_duration =
		(pulse_duration > STIM_OFFSET_US) ? (pulse_duration - STIM_OFFSET_US) : 0;
	uint32_t pulse_duration_ticks = getPulseDurationTicks(
		nonzero_pulse_duration);

	// Start the pulse timer
	sl_sleeptimer_start_timer(&timer_pulse, pulse_duration_ticks,
							  on_pulse_timeout, NULL, 0, 0);
}

void start_stimulation(void)
{
	sl_status_t status;
	bool isRunning = false;

	// Check if the timer is running
	status = sl_sleeptimer_is_timer_running(&timer_stimulation, &isRunning);

	if (status == SL_STATUS_OK && !isRunning)
	{
		// Timer is not running, so start it
		sl_sleeptimer_start_periodic_timer(&timer_stimulation,
										   sl_sleeptimer_ms_to_tick(1000 / frequency),
										   on_stimulation_timeout, NULL, 0, 0);
	}
}

void stop_stimulation(void)
{
	// Stop the stimulation timer
	sl_sleeptimer_stop_timer(&timer_stimulation);
	// Stop the pulse timer if running
	sl_sleeptimer_stop_timer(&timer_pulse);
	// Stop the charge balancing timer if running
	sl_sleeptimer_stop_timer(&timer_charge_balance);
	disable_output();
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

	spidrv_app_init();
	gpio_init();
	iadc_init();

	// Ensure LED starts off
	sl_led_turn_off(LED0);
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

		// Start advertising
		sl_status_t sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
													   sl_bt_legacy_advertiser_connectable);

		app_assert_status(sc);

		// Start LED flashing
		led_is_flashing = true;
		sl_sleeptimer_start_timer_ms(&led_flash_timer,
									 LED_FLASH_INTERVAL_MS,
									 on_led_flash_timeout,
									 NULL,
									 0,
									 0);
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

		// Don't start advertising yet - wait for MAG_PIN interrupt
		break;

		// -------------------------------
		// This event indicates that a new connection was opened.
	case sl_bt_evt_connection_opened_id:
		break;

		// -------------------------------
		// This event indicates that a connection was closed.
	case sl_bt_evt_connection_closed_id:
		// Stop LED flashing
		sl_sleeptimer_stop_timer(&led_flash_timer);
		sl_led_turn_off(LED0);

		if (activateStim)
		{
			start_stimulation();
		}
		// No else needed - we'll wait for magnet to re-enable advertising
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
				handleNodeRxChange(buffer, len);  // This updates txCharSwitch
				compileCommandString(commandStr); // This uses the updated txCharSwitch
				sc = sl_bt_gatt_server_write_attribute_value(
					gattdb_node_tx, 0, sizeof(commandStr), (uint8_t *)commandStr);
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

void compileCommandString(char *commandStr)
{
	if (commandStr != NULL)
	{
		memset(commandStr, 0, COMMAND_STR_MAX_SIZE);

		switch (txCharSwitch)
		{
		case 1:
			sprintf(commandStr, "_A%u,F%u,P%u", amplitude, frequency, pulse_duration);
			break;
		case 2:
			sprintf(commandStr, "_G%u,N%u,V%lu", activateStim, cap_id, read_battery_voltage());
			break;
		default:
			sprintf(commandStr, "invalid tx value");
			break;
		}
	}
}

void handleNodeRxChange(uint8_t *data, size_t len)
{
	if (len == 0 || data[0] != '_')
	{
		return;
	}

	// Check for switch commands first - only check first two bytes
	if (len >= 2) // Changed from len == 2
	{
		if (data[1] == '1')
		{
			txCharSwitch = 1;
			return;
		}
		else if (data[1] == '2')
		{
			txCharSwitch = 2;
			return;
		}
	}

	// Convert data to null-terminated string
	char str[len + 1];		 // +1 for null terminator
	memset(str, 0, len + 1); // Initialize buffer to zero
	memcpy(str, data, len);
	str[len] = '\0';

	// Tokenize the string
	char *token = strtok(str + 1, ","); // Start from str + 1 to skip '_'

	while (token != NULL)
	{
		char settingType = token[0];
		int value = atoi(token + 1); // Convert string to integer

		switch (settingType)
		{
		case 'A':
			if (value != amplitude)
			{
				amplitude = value;
				update_amplitude(amplitude);
			}
			break;
		case 'F':
			if (value != frequency)
			{
				frequency = value;
				update_frequency(frequency);
			}
			break;
		case 'P':
			if (value != pulse_duration)
			{
				pulse_duration = value;
				update_pulse_duration(pulse_duration);
			}
			break;
		case 'L':
			if (value)
			{
				sl_led_toggle(LED0);
			}
			break;
		case 'G':
			activateStim = value;
			break;
		case 'N':
			if (value != cap_id)
			{
				cap_id = value;
				update_cap_id(cap_id);
			}
			break;
		default:
			// Handle unknown setting
			break;
		}

		token = strtok(NULL, ","); // Get next token
	}
	// act on changes
	if (activateStim)
	{
		stop_stimulation(); // makes sure timers restart
		start_stimulation();
	}
	else
	{
		stop_stimulation();
	}
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
		advertising_enabled = true;
	}
}

void GPIO_ODD_IRQHandler(void)
{
	// Clear interrupt flag
	uint32_t flags = GPIO_IntGet();
	GPIO_IntClear(flags);

	if (flags & (1 << MAG_PIN))
	{
		advertising_enabled = true;
	}
}

static void on_led_flash_timeout(sl_sleeptimer_timer_handle_t *handle, void *data)
{
	(void)handle;
	(void)data;

	if (led_is_flashing)
	{
		sl_led_turn_on(LED0);
		// Schedule LED turn off
		sl_sleeptimer_start_timer_ms(&led_flash_timer,
									 LED_FLASH_DURATION_MS,
									 on_led_flash_timeout,
									 NULL,
									 0,
									 0);
		led_is_flashing = false;
	}
	else
	{
		sl_led_turn_off(LED0);
		// Schedule next flash if still advertising
		if (advertising_set_handle != 0xff)
		{
			sl_sleeptimer_start_timer_ms(&led_flash_timer,
										 LED_FLASH_INTERVAL_MS,
										 on_led_flash_timeout,
										 NULL,
										 0,
										 0);
			led_is_flashing = true;
		}
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
