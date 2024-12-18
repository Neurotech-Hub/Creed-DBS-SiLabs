// @file
// @brief Application interface provided to main().

#ifndef APP_H
#define APP_H

#include "sl_sleeptimer.h"

// Application Init.
void app_init(void);

// Application Process Action.
void app_process_action(void);

// GPIO Initialization and Configuration
void gpio_init(void);

// NVM3 Functions
void nvm3_init(void);
void update_cap_id(uint8_t new_cap_id);
void update_amplitude(int new_amplitude);
void update_frequency(int new_frequency);
void update_pulse_duration(int new_pulse_duration);
uint8_t read_cap_id(void);
int read_amplitude(void);
int read_frequency(void);
int read_pulse_duration(void);

// Stimulation Control Functions
void start_stimulation(void);
void stop_stimulation(void);

// SPI/DAC Functions
void spidrv_app_init(void);
void setTxBufferGain(void);
void setTxBufferVolts(float outputVolts);
void transferTxBuffer(void);
void delay_microseconds(uint32_t us);

// BLE Command Functions
void handleNodeRxChange(uint8_t *data, size_t len);
void compileCommandString(char *commandStr);

#endif // APP_H
