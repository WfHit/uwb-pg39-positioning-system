/**
 * @file    hardware_interface.h
 * @brief   Hardware abstraction layer for UWB positioning system
 * 
 * This file provides a unified interface to all hardware components including
 * UWB radio, UART communication, flash storage, OLED display, and timing.
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#ifndef HARDWARE_INTERFACE_H_
#define HARDWARE_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include "../platform/flash/flash_config.h"
#include "../platform/oled/oled_display.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * TYPE DEFINITIONS
 *============================================================================*/

/**
 * @brief Hardware status structure
 */
typedef struct {
    bool initialized;           /* Overall system initialization status */
    bool uwb_ready;             /* UWB radio status */
    bool uart_ready;            /* UART communication status */
    bool flash_ready;           /* Flash memory status */
    bool display_ready;         /* OLED display status */
    uint32_t uptime_ms;         /* System uptime in milliseconds */
    uint32_t last_error;        /* Last error code */
} hardware_status_t;

/*============================================================================
 * HARDWARE INITIALIZATION
 *============================================================================*/

/**
 * @brief Initialize all hardware components
 * @return true if successful, false otherwise
 */
bool hardware_interface_init(void);

/**
 * @brief Check if hardware is initialized
 * @return true if initialized, false otherwise
 */
bool hardware_interface_is_initialized(void);

/*============================================================================
 * UWB RADIO INTERFACE
 *============================================================================*/

/**
 * @brief Initialize UWB radio
 * @return true if successful, false otherwise
 */
bool hardware_interface_uwb_init(void);

/**
 * @brief Send UWB frame
 * @param data Pointer to data buffer
 * @param length Length of data to send
 * @return true if successful, false otherwise
 */
bool hardware_interface_uwb_send(const uint8_t* data, uint16_t length);

/**
 * @brief Receive UWB frame
 * @param data Pointer to receive buffer
 * @param length Pointer to buffer length (in/out parameter)
 * @param timeout_ms Timeout in milliseconds
 * @return true if frame received, false on timeout or error
 */
bool hardware_interface_uwb_receive(uint8_t* data, uint16_t* length, uint32_t timeout_ms);

/**
 * @brief Get UWB signal quality (RSSI)
 * @return RSSI value in dBm
 */
int8_t hardware_interface_uwb_get_rssi(void);

/*============================================================================
 * UART COMMUNICATION INTERFACE
 *============================================================================*/

/**
 * @brief Initialize UART communication
 * @return true if successful, false otherwise
 */
bool hardware_interface_uart_init(void);

/**
 * @brief Send data via UART
 * @param data Pointer to data buffer
 * @param length Length of data to send
 * @return true if successful, false otherwise
 */
bool hardware_interface_uart_send(const uint8_t* data, uint16_t length);

/**
 * @brief Receive data via UART
 * @param data Pointer to receive buffer
 * @param length Pointer to buffer length (in/out parameter)
 * @param timeout_ms Timeout in milliseconds
 * @return true if data received, false on timeout or error
 */
bool hardware_interface_uart_receive(uint8_t* data, uint16_t* length, uint32_t timeout_ms);

/*============================================================================
 * TIMING INTERFACE
 *============================================================================*/

/**
 * @brief Get current timestamp in milliseconds
 * @return Current timestamp
 */
uint32_t hardware_interface_get_timestamp_ms(void);

/**
 * @brief Get current timestamp in microseconds
 * @return Current timestamp
 */
uint64_t hardware_interface_get_timestamp_us(void);

/**
 * @brief Delay for specified milliseconds
 * @param delay_ms Delay time in milliseconds
 */
void hardware_interface_delay_ms(uint32_t delay_ms);

/**
 * @brief Delay for specified microseconds
 * @param delay_us Delay time in microseconds
 */
void hardware_interface_delay_us(uint32_t delay_us);

/*============================================================================
 * DISPLAY INTERFACE
 *============================================================================*/

/**
 * @brief Update display with current system status
 */
void hardware_interface_display_update(void);

/**
 * @brief Display measurement result
 * @param distance Measured distance in meters
 * @param anchor_id ID of anchor measured
 */
void hardware_interface_display_measurement(float distance, uint8_t anchor_id);

/**
 * @brief Display error message
 * @param error_msg Error message string
 */
void hardware_interface_display_error(const char* error_msg);

/**
 * @brief Display network status
 * @param anchor_count Number of discovered anchors
 * @param tag_count Number of active tags
 */
void hardware_interface_display_network_status(uint8_t anchor_count, uint8_t tag_count);

/*============================================================================
 * CONFIGURATION INTERFACE
 *============================================================================*/

/**
 * @brief Get system configuration
 * @return Pointer to configuration structure
 */
const uwb_config_t* hardware_interface_get_config(void);

/**
 * @brief Set system configuration
 * @param config Pointer to configuration structure
 * @return true if successful, false otherwise
 */
bool hardware_interface_set_config(const uwb_config_t* config);

/**
 * @brief Get calibration data
 * @return Pointer to calibration structure
 */
const uwb_calibration_t* hardware_interface_get_calibration(void);

/**
 * @brief Set calibration data
 * @param calibration Pointer to calibration structure
 * @return true if successful, false otherwise
 */
bool hardware_interface_set_calibration(const uwb_calibration_t* calibration);

/*============================================================================
 * STATUS AND DIAGNOSTICS
 *============================================================================*/

/**
 * @brief Get hardware status
 * @return Hardware status structure
 */
hardware_status_t hardware_interface_get_status(void);

/**
 * @brief Perform system self-test
 * @return true if all tests pass, false otherwise
 */
bool hardware_interface_self_test(void);

/**
 * @brief System reset
 */
void hardware_interface_system_reset(void);

/*============================================================================
 * LEGACY COMPATIBILITY FUNCTIONS
 *============================================================================*/

/* Legacy function mappings for backward compatibility */
#define hw_peripherals_init()           hardware_interface_init()
#define hw_system_reset()               hardware_interface_system_reset()
#define hw_get_system_tick()            hardware_interface_get_timestamp_ms()
#define hw_get_high_precision_timestamp() hardware_interface_get_timestamp_us()
#define hw_delay_ms(ms)                 hardware_interface_delay_ms(ms)
#define hw_delay_us(us)                 hardware_interface_delay_us(us)
#define hw_uart_transmit(data, len)     hardware_interface_uart_send(data, len)

/* Legacy LED and button functions */
void hw_led_on(void);
void hw_led_off(void);
void hw_led_toggle(void);
bool hw_button_pressed(void);

/* Legacy flash functions */
bool hw_flash_read(uint32_t address, uint8_t* data, uint16_t length);
bool hw_flash_write(uint32_t address, const uint8_t* data, uint16_t length);
bool hw_flash_erase_page(uint32_t address);

/* Legacy UWB functions */
bool dwm3000_init(void);
bool dwm3000_send_frame(const uint8_t* data, uint16_t length);
bool dwm3000_receive_frame(uint8_t* buffer, uint16_t buffer_size, int8_t* rssi, uint32_t timeout_ms);
bool dwm3000_is_ready(void);
void dwm3000_set_config(uint8_t channel, uint8_t prf, uint8_t data_rate);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_INTERFACE_H_ */
