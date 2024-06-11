/***************************************************************************
 * @file Creed DBS app
 * @brief Core application logic.
 **************************************************************************/

// Standard library includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "sl_sleeptimer.h"
#include "sl_bluetooth.h"

#include "em_cmu.h"
#include "em_timer.h"
#include "em_common.h"
#include "em_emu.h"
#include "app_assert.h"
#include "gatt_db.h"
#include "app.h"
#include "pin_config.h"

#include "spidrv.h"
#include "sl_spidrv_instances.h"

#include "nvm3.h"
#include "nvm3_default.h"

// TIMING
#define OP_AMP_SETTLE 100 // µs
#define ADV_INTERVAL_MS 1000
sl_sleeptimer_timer_handle_t timer_stimulation; // based on frequency
sl_sleeptimer_timer_handle_t timer_pulse; // based on amplitude
sl_sleeptimer_timer_handle_t timer_charge_balance; // balance amplitude

// STIM
#define CHARGE_BALANCE_RATIO 1

// HARDWARE
#define LED0	(&sl_led_inst)
#define NVM3_KEY_CAP_ID        0x00000001UL
#define NVM3_KEY_AMPLITUDE     0x00000002UL
#define NVM3_KEY_FREQUENCY     0x00000003UL
#define NVM3_KEY_PULSE_DURATION 0x00000004UL
uint8_t cap_id;

// BLE
#define COMMAND_STR_MAX_SIZE 26 // should match nodeTx characteristic size
static uint8_t advertising_set_handle = 0xff;
char commandStr[COMMAND_STR_MAX_SIZE];

// App
static uint8_t activateStim = 0;
static int amplitude = 0;
static int frequency = 0;
static int pulse_duration = 0;

// Functions
void
handleNodeRxChange(uint8_t *data, size_t len);
void
compileCommandString(char *commandStr);

// SPI/DAC
#define SPI_HANDLE                  sl_spidrv_inst_handle
// size of transmission and reception buffers
#define APP_BUFFER_SIZE             3

// Transmission and reception buffers
uint8_t rx_buffer[APP_BUFFER_SIZE];
uint8_t tx_buffer[APP_BUFFER_SIZE];
static const float refVolts = 1.25;
static const uint16_t maxADCValue = 65535;

// !! put this in .h
void setTxBufferGain(void);
void setTxBufferVolts(float outputVolts);
void transferTxBuffer(void);
void delay_microseconds(uint32_t us);

#define TOGGLE_DELAY_MS 100
#define APP_BUFFER_SIZE 3

bool transfer_complete = false; // Flag for SPI transfer completion
bool spi_transfer_pending = false; // Flag for pending SPI transfer

uint8_t tx_buffer[APP_BUFFER_SIZE];
uint8_t rx_buffer[APP_BUFFER_SIZE];

//extern uint32_t sl_sleeptimer_get_timer_frequency(void);

void nvm3_init(void) {
	Ecode_t result = nvm3_initDefault();
	if (result != ECODE_NVM3_OK) {
		// Handle error
	}
}

void update_cap_id(uint8_t new_cap_id) {
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_CAP_ID,
			&new_cap_id, sizeof(new_cap_id));
	if (result != ECODE_NVM3_OK) {
		// Handle error
	}
}

void update_amplitude(int new_amplitude) {
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_AMPLITUDE,
			&new_amplitude, sizeof(new_amplitude));
	if (result != ECODE_NVM3_OK) {
		// Handle error
	}
}

void update_frequency(int new_frequency) {
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_FREQUENCY,
			&new_frequency, sizeof(new_frequency));
	if (result != ECODE_NVM3_OK) {
		// Handle error
	}
}

void update_pulse_duration(int new_pulse_duration) {
	Ecode_t result = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_PULSE_DURATION,
			&new_pulse_duration, sizeof(new_pulse_duration));
	if (result != ECODE_NVM3_OK) {
		// Handle error
	}
}

uint8_t read_cap_id(void) {
	uint8_t cap_id = 0x00;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_CAP_ID, &cap_id,
			sizeof(cap_id));
	if (result != ECODE_NVM3_OK) {
		// Handle error or set default value
		cap_id = 0x00;
	}
	return cap_id;
}

int read_amplitude(void) {
	int amplitude = 0;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_AMPLITUDE,
			&amplitude, sizeof(amplitude));
	if (result != ECODE_NVM3_OK) {
		// Handle error or set default value
		amplitude = 0;
	}
	return amplitude;
}

int read_frequency(void) {
	int frequency = 0;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_FREQUENCY,
			&frequency, sizeof(frequency));
	if (result != ECODE_NVM3_OK) {
		// Handle error or set default value
		frequency = 0;
	}
	return frequency;
}

int read_pulse_duration(void) {
	int pulse_duration = 0;
	Ecode_t result = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_PULSE_DURATION,
			&pulse_duration, sizeof(pulse_duration));
	if (result != ECODE_NVM3_OK) {
		// Handle error or set default value
		pulse_duration = 0;
	}
	return pulse_duration;
}

// scale based on 0-100% for amplitude, -V=I
float calcStimVolts(void) {
	float stimVolts = refVolts - refVolts * (amplitude / 100.0);
	return stimVolts;
}

float calcChargeBalanceVolts(void) {
	float stimVolts = calcStimVolts();
	float chargeBalanceVolts = refVolts
			+ ((refVolts - stimVolts) / CHARGE_BALANCE_RATIO);
	return chargeBalanceVolts;
}

void transfer_callback(SPIDRV_HandleData_t *handle, Ecode_t transfer_status,
		int items_transferred) {
	(void) handle;
	(void) items_transferred;

	// Post semaphore to signal to application task that transfer is successful
	if (transfer_status == ECODE_EMDRV_SPIDRV_OK) {
		transfer_complete = true;
	}
}

void setTxBufferGain(void) {
	// gain x2, internal ref off
	tx_buffer[0] = 0x41;
	tx_buffer[1] = 0x80;
	tx_buffer[2] = 0x00;
	transferTxBuffer();
}

void setTxBufferVolts(float outputVolts) {
	uint16_t ADC_bits =
			(uint16_t) (((outputVolts / refVolts) / 2) * maxADCValue); // divide by 2 for gain
	tx_buffer[0] = (1 << 4) | (ADC_bits >> 12); // Set upper nibble to 1, then add top 4 bits of ADC_bits
	tx_buffer[1] = (ADC_bits >> 4) & 0xFF;       // Middle 8 bits of ADC_bits
	tx_buffer[2] = (ADC_bits << 4) & 0xF0; // Last 4 bits of ADC_bits shifted to the top, lower 4 bits are 0
	transferTxBuffer();
}

void transferTxBuffer(void) {
	Ecode_t ecode;
	transfer_complete = false;
	ecode = SPIDRV_MTransferB(SPI_HANDLE, tx_buffer, rx_buffer,
	APP_BUFFER_SIZE); // transfer_callback
	EFM_ASSERT(ecode == ECODE_OK);
}

void init_timer(void) {
	// Enable clock for TIMER0
	CMU_ClockEnable(cmuClock_TIMER0, true);

	// Configure TIMER0 for one-shot mode with prescaler
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.enable = false;
	timerInit.oneShot = true;
	timerInit.prescale = timerPrescale1;

	TIMER_Init(TIMER0, &timerInit);
}

void start_timer(uint32_t us) {
	delay_ticks = (SystemCoreClock / 1000000) * us;

	// Set the TIMER top value
	TIMER_TopSet(TIMER0, delay_ticks);

	// Clear the counter
	TIMER_CounterSet(TIMER0, 0);

	// Clear any pending interrupts
	TIMER_IntClear(TIMER0, TIMER_IF_OF);

	// Start the timer
	TIMER_Enable(TIMER0, true);
}

bool timer_expired(void) {
	return (TIMER_IntGet(TIMER0) & TIMER_IF_OF) != 0;
}

void clear_timer(void) {
	// Clear the interrupt flag
	TIMER_IntClear(TIMER0, TIMER_IF_OF);

	// Disable the timer
	TIMER_Enable(TIMER0, false);
}

static void enable_charge_balancing(void) {
	float chargeBalanceVolts = calcChargeBalanceVolts();
	setTxBufferVolts(chargeBalanceVolts);
}

static void enable_stimulation_output(void) {
	float stimVolts = calcStimVolts();
	GPIO_PinOutSet(SHDN_PORT, SHDN_PIN);
	setTxBufferVolts(stimVolts);
}

static void disable_output(void) {
	GPIO_PinOutClear(ELEC0_OUT_PORT, ELEC0_OUT_PIN);
	GPIO_PinOutClear(ELEC1_OUT_PORT, ELEC1_OUT_PIN);
	GPIO_PinOutClear(SHDN_PORT, SHDN_PIN);
	setTxBufferVolts(refVolts);
}

static void on_stimulation_timeout(sl_sleeptimer_timer_handle_t *handle,
		void *data) {
	(void) handle;
	(void) data;

	// Transition to the state to enable stimulation
	current_state = STATE_ENABLE_STIMULATION;
}

void start_stimulation(void) {
	sl_status_t status;
	bool isRunning = false;

	status = sl_sleeptimer_is_timer_running(&timer_stimulation, &isRunning);

	if (status == SL_STATUS_OK && !isRunning) {
		sl_sleeptimer_start_periodic_timer(&timer_stimulation,
				sl_sleeptimer_ms_to_tick(1000 / frequency),
				on_stimulation_timeout, NULL, 0, 0);
		current_state = STATE_ENABLE_STIMULATION;
	}
}

void stop_stimulation(void) {
	sl_sleeptimer_stop_timer(&timer_stimulation);
	disable_output();
	current_state = STATE_IDLE;
}

void stop_ble_advertising(void) {
	sl_status_t status = sl_bt_advertiser_stop(advertising_set_handle);
	if (status != SL_STATUS_OK) {
	}
}

//void MAG_PIN_interrupt_handler(uint8_t pin) {
//	// Disable the interrupt
//	GPIO_IntDisable(1 << pin);
//
//	// Re-enable BLE functionality
//	sl_bt_legacy_advertiser_start(advertising_set_handle,
//			sl_bt_legacy_advertiser_connectable);
//
//	// Clear the interrupt flag
//	GPIO_IntClear(1 << pin);
//}
//
//void GPIO_EVEN_IRQHandler(void) {
//	uint32_t flags = GPIO_IntGet();
//	GPIO_IntClear(flags);
//
//	if (flags & (1 << MAG_PIN)) {
//		MAG_PIN_interrupt_handler(MAG_PIN);
//	}
//}
//
//void init_mag_pin_interrupt(void) {
//	// Configure MAG_PIN as input with pull-up
//	GPIO_PinModeSet(MAG_PORT, MAG_PIN, gpioModeInputPull, 1);
//
//	// Enable interrupt on MAG_PIN
//	GPIO_ExtIntConfig(MAG_PORT, MAG_PIN, MAG_PIN, false, true, true);
//
//	// Clear any pending interrupts
//	GPIO_IntClear(1 << MAG_PIN);
//
//	// Enable the interrupt
//	GPIO_IntEnable(1 << MAG_PIN);
//}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void) {
	/////////////////////////////////////////////////////////////////////////////
	// Put your additional application init code here!                         //
	// This is called once during start-up.                                    //
	/////////////////////////////////////////////////////////////////////////////
	sl_led_turn_on(LED0);
	GPIO_PinModeSet(SHDN_PORT, SHDN_PIN, gpioModePushPull, 0);
//	GPIO_PinModeSet(MAG_PORT, MAG_PIN, gpioModeInputPull, 1);
	GPIO_PinModeSet(ELEC0_OUT_PORT, ELEC0_OUT_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(ELEC1_OUT_PORT, ELEC1_OUT_PIN, gpioModePushPull, 0);

	nvm3_init();
	cap_id = read_cap_id();
	amplitude = read_amplitude();
	frequency = read_frequency();
	pulse_duration = read_pulse_duration();

	// DAC init
	setTxBufferGain();
	setTxBufferVolts(refVolts);

	init_timer();

	uint32_t sleeptimerFreq = sl_sleeptimer_get_timer_frequency();

	// Calculate the resolution in microseconds
	float sleeptimerResolution_us = (1.0 / sleeptimerFreq) * 1000000;

	// Variable to check using the debugger
	volatile float resolution_us = sleeptimerResolution_us;

	sl_led_turn_off(LED0);

//	// Enable GPIO clock
//	CMU_ClockEnable(cmuClock_GPIO, true);
//
//	// Register the interrupt handler
//	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
//	NVIC_SetPriority(GPIO_EVEN_IRQn, 1);
//
//	// Initialize MAG_PIN interrupt
//	init_mag_pin_interrupt();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void) {
	/////////////////////////////////////////////////////////////////////////////
	// Put your additional application code here!                              //
	// This is called infinitely.                                              //
	// Do not call blocking functions from here!                               //
	/////////////////////////////////////////////////////////////////////////////
	switch (current_state) {
	case STATE_IDLE:
		// Do nothing
		break;

	case STATE_ENABLE_STIMULATION:
		enable_stimulation_output();
		start_timer(OP_AMP_SETTLE);
		current_state = STATE_WAIT_FOR_OP_AMP_SETTLE;
		break;

	case STATE_WAIT_FOR_OP_AMP_SETTLE:
		if (timer_expired()) {
			clear_timer();
			setTxBufferVolts(calcStimVolts());
			start_timer(5); // 5 us delay
			current_state = STATE_SET_STIM_VOLTAGE;
		}
		break;

	case STATE_SET_STIM_VOLTAGE:
		if (timer_expired()) {
			clear_timer();
			GPIO_PinOutSet(ELEC0_OUT_PORT, ELEC0_OUT_PIN);
			GPIO_PinOutSet(ELEC1_OUT_PORT, ELEC1_OUT_PIN);
			start_timer(pulse_duration);
			current_state = STATE_WAIT_FOR_STIM_PULSE;
		}
		break;

	case STATE_WAIT_FOR_STIM_PULSE:
		if (timer_expired()) {
			clear_timer();
			enable_charge_balancing();
			start_timer(CHARGE_BALANCE_RATIO * pulse_duration);
			current_state = STATE_WAIT_FOR_CHARGE_BALANCING;
		}
		break;

	case STATE_ENABLE_CHARGE_BALANCING:
		enable_charge_balancing();
		start_timer(CHARGE_BALANCE_RATIO * pulse_duration);
		current_state = STATE_WAIT_FOR_CHARGE_BALANCING;
		break;

	case STATE_WAIT_FOR_CHARGE_BALANCING:
		if (timer_expired()) {
			clear_timer();
			disable_output();
			current_state = STATE_DISABLE_OUTPUT;
		}
		break;

	case STATE_DISABLE_OUTPUT:
		disable_output();
		current_state = STATE_IDLE;
		break;
	}
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt) {
	sl_status_t sc;

	switch (SL_BT_MSG_ID(evt->header)) {
	// -------------------------------
	// This event indicates the device has started and the radio is ready.
	// Do not call any stack command before receiving this boot event!
	case sl_bt_evt_system_boot_id:
		// Create an advertising set.
		sc = sl_bt_advertiser_create_set(&advertising_set_handle);
		app_assert_status(sc);

		// Generate data for advertising
		sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
				sl_bt_advertiser_general_discoverable);
		app_assert_status(sc);

		// Set advertising interval to 100ms.
		sc = sl_bt_advertiser_set_timing(advertising_set_handle,
		ADV_INTERVAL_MS, // min. adv. interval (milliseconds * 1.6)
				ADV_INTERVAL_MS, // max. adv. interval (milliseconds * 1.6)
				0,   // adv. duration
				0);  // max. num. adv. events
		app_assert_status(sc);

		// Start advertising and enable connections.
		sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
				sl_bt_legacy_advertiser_connectable);
		app_assert_status(sc);
		break;

		// -------------------------------
		// This event indicates that a new connection was opened.
	case sl_bt_evt_connection_opened_id:
		// update for initial sync
		compileCommandString(commandStr);
		sc = sl_bt_gatt_server_write_attribute_value(
		gattdb_node_tx, 0, sizeof(commandStr), (uint8_t*) commandStr);
//		stop_stimulation();
//		sl_led_turn_off(LED0); // known state
		break;

		// -------------------------------
		// This event indicates that a connection was closed.
	case sl_bt_evt_connection_closed_id:
//		sl_led_turn_off(LED0); // known state
//		if (activateStim) {
//			start_stimulation();
//		}

		// Generate data for advertising
		sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
				sl_bt_advertiser_general_discoverable);
		app_assert_status(sc);

		// Restart advertising after client has disconnected.
		sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
				sl_bt_legacy_advertiser_connectable);
		app_assert_status(sc);
		break;

		// -------------------------------
		// This event indicates that the value of an attribute in the local GATT
		// database was changed by a remote GATT client.
	case sl_bt_evt_gatt_server_attribute_value_id:
		// The value of the gattdb_led_control characteristic was changed.

		// Check if the attribute written is gattdb_node_rx
		if (evt->data.evt_gatt_server_attribute_value.attribute
				== gattdb_node_rx) {
			// Read the updated value
			uint8_t buffer[26]; // !! set to char size, can't use #define
			size_t len = 0;
			sc = sl_bt_gatt_server_read_attribute_value(
			gattdb_node_rx, 0, sizeof(buffer), &len, buffer);

			if (sc == SL_STATUS_OK) {
				handleNodeRxChange(buffer, len);
				compileCommandString(commandStr);
				sc = sl_bt_gatt_server_write_attribute_value(
				gattdb_node_tx, 0, sizeof(commandStr), (uint8_t*) commandStr);
			} else {
				// Handle error
			}
		}
		break;

		// -------------------------------
		// This event occurs when the remote device enabled or disabled the
		// notification.
	case sl_bt_evt_gatt_server_characteristic_status_id:
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

void compileCommandString(char *commandStr) {
	if (commandStr != NULL) {
		// Initialize the entire buffer with null characters
		memset(commandStr, 0, COMMAND_STR_MAX_SIZE);
		// Format the string with the global variable values
		sprintf(commandStr, "_A%u,F%u,P%u,G%u,N%u", amplitude, frequency,
				pulse_duration, activateStim, cap_id);
	}
}

void handleNodeRxChange(uint8_t *data, size_t len) {
	if (len == 0 || data[0] != '_') {
		return; // No valid data or does not start with '_'
	}

	// Convert data to null-terminated string
	char str[len + 1]; // +1 for null terminator
	memset(str, 0, len + 1); // Initialize buffer to zero
	memcpy(str, data, len);
	str[len] = '\0';

	// Tokenize the string
	char *token = strtok(str + 1, ","); // Start from str + 1 to skip '_'

	while (token != NULL) {
		char settingType = token[0];
		int value = atoi(token + 1); // Convert string to integer

		switch (settingType) {
		case 'A':
			if (value != amplitude) {
				amplitude = value;
				update_amplitude(amplitude);
			}
			break;
		case 'F':
			if (value != frequency) {
				frequency = value;
				update_frequency(frequency);
			}
			break;
		case 'P':
			if (value != pulse_duration) {
				pulse_duration = value;
				update_pulse_duration(pulse_duration);
			}
			break;
		case 'L':
			if (value) {
				sl_led_toggle(LED0);
			}
			break;
		case 'G':
			activateStim = value;
			break;
		case 'N':
			if (value != cap_id) {
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
	if (activateStim) {
		stop_stimulation(); // makes sure timers restart
		start_stimulation();
	} else {
		stop_stimulation();
	}
}
