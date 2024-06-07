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

#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "pin_config.h"
//#include "em_gpio.h"

#include "spidrv.h"
#include "sl_spidrv_instances.h"

// HARDWARE
#define LED0	(&sl_led_inst)

// BLE
#define COMMAND_STR_MAX_SIZE 20 // should match nodeTx characteristic size
static uint8_t advertising_set_handle = 0xff;

// App
static uint8_t activateOnDisconnect = 0;
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
//uint16_t ADC_bits;
//float outputVolts = 1.8;
static const float refVolts = 1.25;
static const uint16_t maxADCValue = 65535;

// !! put this in .h
void setTxBufferGain(void);
void setTxBufferVolts(float outputVolts);
void transferTxBuffer(void);
void delay_microseconds(uint32_t us);

#define TOGGLE_DELAY_MS 100
#define APP_BUFFER_SIZE 3

sl_sleeptimer_timer_handle_t timer_blink;
sl_sleeptimer_timer_handle_t timer_stimulation;
sl_sleeptimer_timer_handle_t timer_pulse;

bool toggle_blink_timeout = false;
bool should_blink = false; // Global flag to control blinking
bool stimulation_enabled = false; // Global flag to control stimulation
bool transfer_complete = false; // Flag for SPI transfer completion
bool spi_transfer_pending = false; // Flag for pending SPI transfer

uint8_t tx_buffer[APP_BUFFER_SIZE];
uint8_t rx_buffer[APP_BUFFER_SIZE];

extern uint32_t sl_sleeptimer_get_timer_frequency(void);

void transfer_callback(SPIDRV_HandleData_t *handle, Ecode_t transfer_status, int items_transferred) {
    (void)handle;
    (void)items_transferred;

    // Post semaphore to signal to application task that transfer is successful
    if (transfer_status == ECODE_EMDRV_SPIDRV_OK) {
        transfer_complete = true;
    }
}

void spidrv_app_init(void) {
    setTxBufferGain();
}

void setTxBufferGain(void) {
    // gain x2, internal ref off
    tx_buffer[0] = 0x41;
    tx_buffer[1] = 0x80;
    tx_buffer[2] = 0x00;
    spi_transfer_pending = true;
}

void setTxBufferVolts(float outputVolts) {
    uint16_t ADC_bits = (uint16_t)(((outputVolts / refVolts) / 2) * maxADCValue); // divide by 2 for gain
    tx_buffer[0] = (1 << 4) | (ADC_bits >> 12); // Set upper nibble to 1, then add top 4 bits of ADC_bits
    tx_buffer[1] = (ADC_bits >> 4) & 0xFF;       // Middle 8 bits of ADC_bits
    tx_buffer[2] = (ADC_bits << 4) & 0xF0;       // Last 4 bits of ADC_bits shifted to the top, lower 4 bits are 0
    spi_transfer_pending = true;
}

void transferTxBuffer(void) {
    Ecode_t ecode;
    transfer_complete = false;
    ecode = SPIDRV_MTransfer(SPI_HANDLE, tx_buffer, rx_buffer, APP_BUFFER_SIZE, transfer_callback);
    EFM_ASSERT(ecode == ECODE_OK);
}

void delay_microseconds(uint32_t us) {
    // Convert microseconds to timer ticks
    uint32_t ticks = (us * sl_sleeptimer_get_timer_frequency()) / 1000000;

    // Get the current tick count
    uint32_t start_tick = sl_sleeptimer_get_tick_count();

    // Wait until the specified number of ticks have passed
    while ((sl_sleeptimer_get_tick_count() - start_tick) < ticks) {
        // Do nothing, just wait
    }
}

static void enable_stimulation_output(void) {
    setTxBufferVolts(1.8);
    GPIO_PinOutSet(SHDN_PORT, SHDN_PIN);
    GPIO_PinOutSet(ELEC0_OUT_PORT, ELEC0_OUT_PIN);
    GPIO_PinOutSet(ELEC1_OUT_PORT, ELEC1_OUT_PIN);
    sl_led_turn_on(LED0);
}

static void disable_stimulation_output(void) {
    GPIO_PinOutClear(ELEC1_OUT_PORT, ELEC1_OUT_PIN);
    GPIO_PinOutClear(SHDN_PORT, SHDN_PIN);
    setTxBufferVolts(refVolts);
    sl_led_turn_off(LED0);
}

static void on_blink_timeout(sl_sleeptimer_timer_handle_t *handle, void *data) {
    (void)handle;
    (void)data;
    toggle_blink_timeout = true;
}

static void on_pulse_timeout(sl_sleeptimer_timer_handle_t *handle, void *data) {
    (void)handle;
    (void)data;
    stimulation_enabled = false;
    disable_stimulation_output(); // Disable stimulation once
}

static void on_stimulation_timeout(sl_sleeptimer_timer_handle_t *handle, void *data) {
    (void)handle;
    (void)data;
    stimulation_enabled = true;
    enable_stimulation_output(); // Enable stimulation once

    // Calculate the number of ticks for pulse_duration in µs
    uint32_t timer_frequency = sl_sleeptimer_get_timer_frequency();
    uint32_t pulse_duration_ticks = (pulse_duration * timer_frequency) / 1000000;

    sl_sleeptimer_start_timer(&timer_pulse, pulse_duration_ticks, on_pulse_timeout, NULL, 0, 0);
}

void start_blinking(void) {
    sl_status_t status;
    bool isRunning = false;

    should_blink = true;
    // Check if the timer is running
    status = sl_sleeptimer_is_timer_running(&timer_blink, &isRunning);

    if (status == SL_STATUS_OK && !isRunning) {
        // Timer is not running, so start it
        sl_sleeptimer_start_periodic_timer_ms(&timer_blink, TOGGLE_DELAY_MS, on_blink_timeout, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
    }
}

void stop_blinking(void) {
    should_blink = false;
    // Stop the timer
    sl_sleeptimer_stop_timer(&timer_blink);
}

void start_stimulation(void) {
    sl_status_t status;
    bool isRunning = false;

    // Check if the timer is running
    status = sl_sleeptimer_is_timer_running(&timer_stimulation, &isRunning);

    if (status == SL_STATUS_OK && !isRunning) {
        // Timer is not running, so start it
        sl_sleeptimer_start_periodic_timer(&timer_stimulation, sl_sleeptimer_ms_to_tick(1000 / frequency), on_stimulation_timeout, NULL, 0, 0);
    }
}

void stop_stimulation(void) {
    // Stop the stimulation timer
    sl_sleeptimer_stop_timer(&timer_stimulation);
    // Stop the pulse timer if running
    sl_sleeptimer_stop_timer(&timer_pulse);
    // Ensure stimulation is turned off
    stimulation_enabled = false;
    disable_stimulation_output();
}

void process_actions(void) {
    if (toggle_blink_timeout && should_blink) {
        sl_led_toggle(LED0);
        toggle_blink_timeout = false;
    }

    if (spi_transfer_pending) {
        transferTxBuffer();
        spi_transfer_pending = false;
    }
}


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void) {
	/////////////////////////////////////////////////////////////////////////////
	// Put your additional application init code here!                         //
	// This is called once during start-up.                                    //
	/////////////////////////////////////////////////////////////////////////////
	GPIO_PinModeSet(SHDN_PORT, SHDN_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(MAG_PORT, MAG_PIN, gpioModeInputPull, 1);
	GPIO_PinModeSet(ELEC0_OUT_PORT, ELEC0_OUT_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(ELEC1_OUT_PORT, ELEC1_OUT_PIN, gpioModePushPull, 0);

	spidrv_app_init();
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
	process_actions();
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
		sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, // min. adv. interval (milliseconds * 1.6)
				160, // max. adv. interval (milliseconds * 1.6)
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
		stop_stimulation();
		stop_blinking();
		sl_led_turn_off(LED0); // known state
		break;

		// -------------------------------
		// This event indicates that a connection was closed.
	case sl_bt_evt_connection_closed_id:
		sl_led_turn_off(LED0); // known state
		if (activateOnDisconnect) {
//			start_blinking();
			start_stimulation();
		}

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
			uint8_t buffer[20]; // Adjust the size as needed
			size_t len = 0;
			sc = sl_bt_gatt_server_read_attribute_value(
			gattdb_node_rx, 0, sizeof(buffer), &len, buffer);

			if (sc == SL_STATUS_OK) {
				// Trigger your function here and pass the buffer and len as parameters
				handleNodeRxChange(buffer, len);

				// get command string and set nodeTx
				char commandStr[COMMAND_STR_MAX_SIZE];
				compileCommandString(commandStr);
//				 Write attribute in the local GATT database.
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
		sprintf(commandStr, "_A%u,F%u,P%u,G%u", amplitude, frequency,
				pulse_duration, activateOnDisconnect);
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
			amplitude = value;
			break;
		case 'F':
			frequency = value;
			break;
		case 'P':
			pulse_duration = value;
			break;
		case 'L':
			if (value) {
				sl_led_toggle(LED0);
			}
			break;
		case 'G':
			activateOnDisconnect = value;
			break;
		default:
			// Handle unknown setting
			break;
		}

		token = strtok(NULL, ","); // Get next token
	}
}
