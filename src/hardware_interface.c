/**
 * @file    hardware_interface.c
 * @brief   Hardware Interface Implementation for UWB Positioning System
 * 
 * This file implements the hardware abstraction layer that integrates
 * all platform-specific components including flash, OLED, UWB radio, and UART.
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "hardware_interface.h"
#include "platform/port.h"
#include "platform/flash/flash_config.h"
#include "platform/oled/oled_display.h"
#include "platform/spi.h"
#include "platform/delay/Delay.h"
#include <deca_device_api.h>
#include <string.h>

/*============================================================================
 * PRIVATE VARIABLES
 *============================================================================*/

static bool hardware_initialized = false;
static uwb_config_t system_config;
static uwb_calibration_t system_calibration;
static uwb_display_data_t display_data;

/*============================================================================
 * HARDWARE INITIALIZATION
 *============================================================================*/

/**
 * @brief Initialize all hardware components
 */
bool hardware_interface_init(void)
{
    /* Initialize system clocks first */
    RCC_Configuration();
    SysTick_Configuration();
    
    /* Initialize GPIO configuration */
    GPIO_Configuration();
    
    /* Initialize all peripherals */
    peripherals_init();
    
    /* Initialize flash memory system */
    if (flash_config_init() != FLASH_SUCCESS) {
        return false;
    }
    
    /* Load system configuration from flash */
    if (flash_load_uwb_config(&system_config) != FLASH_SUCCESS) {
        /* Use default configuration if flash is empty */
        flash_reset_uwb_config();
        flash_load_uwb_config(&system_config);
    }
    
    /* Load calibration data from flash */
    if (flash_load_calibration(&system_calibration) != FLASH_SUCCESS) {
        /* Use default calibration if flash is empty */
        flash_reset_calibration();
        flash_load_calibration(&system_calibration);
    }
    
    /* Initialize OLED display */
    if (!oled_init()) {
        return false;
    }
    
    /* Display startup screen */
    oled_clear_screen();
    oled_draw_string(0, 0, "UWB PG3.9", OLED_FONT_MEDIUM, false);
    oled_draw_string(0, 2, "Initializing...", OLED_FONT_SMALL, false);
    
    /* Initialize UWB radio */
    if (!hardware_interface_uwb_init()) {
        oled_display_error_screen("UWB Init Failed");
        return false;
    }
    
    /* Initialize UART communication */
    if (!hardware_interface_uart_init()) {
        oled_display_error_screen("UART Init Failed");
        return false;
    }
    
    /* Initialize display data structure */
    memset(&display_data, 0, sizeof(uwb_display_data_t));
    display_data.device_mode = system_config.device_mode;
    display_data.device_id = system_config.device_id;
    display_data.uwb_initialized = true;
    display_data.flash_ok = true;
    display_data.uart_ok = true;
    
    /* Show ready screen */
    oled_clear_screen();
    oled_draw_string(0, 0, "UWB PG3.9", OLED_FONT_MEDIUM, false);
    oled_draw_string(0, 2, "Ready", OLED_FONT_SMALL, false);
    
    char str[32];
    sprintf(str, "%s ID:%d", 
            (system_config.device_mode == 0) ? "Tag" : "Anchor", 
            system_config.device_id);
    oled_draw_string(0, 3, str, OLED_FONT_SMALL, false);
    
    Delay_ms(2000);
    
    hardware_initialized = true;
    return true;
}

/**
 * @brief Check if hardware is initialized
 */
bool hardware_interface_is_initialized(void)
{
    return hardware_initialized;
}

/*============================================================================
 * UWB RADIO INTERFACE
 *============================================================================*/

/**
 * @brief Initialize UWB radio
 */
bool hardware_interface_uwb_init(void)
{
    /* Reset UWB chip */
    reset_DWIC();
    wakeup_device_with_io();
    
    /* Initialize DecaWave driver */
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        return false;
    }
    
    /* Configure UWB parameters from stored configuration */
    dwt_config_t uwb_config = {
        .chan = system_config.channel,
        .rxCode = 9,
        .txCode = 9,
        .dataRate = system_config.data_rate,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdSeq = DWT_SFD_DW_8,
        .sfdTO = (129 + 8 - 8),
        .smartPowerEn = 1
    };
    
    dwt_configure(&uwb_config);
    
    /* Set antenna delays from calibration */
    dwt_setrxantennadelay(system_calibration.antenna_delay_rx);
    dwt_settxantennadelay(system_calibration.antenna_delay_tx);
    
    return true;
}

/**
 * @brief Send UWB frame
 */
bool hardware_interface_uwb_send(const uint8_t* data, uint16_t length)
{
    if (!hardware_initialized || data == NULL || length == 0) {
        return false;
    }
    
    dwt_writetxdata(length, (uint8_t*)data, 0);
    dwt_writetxfctrl(length, 0, 1);
    
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR) {
        return false;
    }
    
    /* Wait for transmission to complete */
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
        /* Could add timeout here */
    }
    
    /* Clear status */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    
    return true;
}

/**
 * @brief Receive UWB frame
 */
bool hardware_interface_uwb_receive(uint8_t* data, uint16_t* length, uint32_t timeout_ms)
{
    if (!hardware_initialized || data == NULL || length == NULL) {
        return false;
    }
    
    uint32_t start_time = HAL_GetTick();
    
    /* Enable receiver */
    dwt_setrxaftertxdelay(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    /* Wait for frame reception */
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG)) {
        if ((HAL_GetTick() - start_time) > timeout_ms) {
            dwt_forcetrxoff();
            return false;
        }
        
        /* Check for errors */
        if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_ALL_RX_ERR) {
            dwt_forcetrxoff();
            return false;
        }
    }
    
    /* Get frame length */
    *length = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    
    /* Read frame data */
    dwt_readrxdata(data, *length, 0);
    
    /* Clear status */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR);
    
    return true;
}

/**
 * @brief Get UWB signal quality
 */
int8_t hardware_interface_uwb_get_rssi(void)
{
    if (!hardware_initialized) {
        return -127;
    }
    
    /* Read receive signal power */
    uint32_t rx_power = dwt_read32bitreg(RX_FQUAL_ID);
    
    /* Convert to dBm (simplified calculation) */
    int8_t rssi = (int8_t)(rx_power >> 8) - 115;
    
    return rssi;
}

/*============================================================================
 * UART INTERFACE
 *============================================================================*/

/**
 * @brief Initialize UART communication
 */
bool hardware_interface_uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    /* Enable USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
    /* Configure USART1 pins */
    /* TX (PA9) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* RX (PA10) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure USART1 */
    USART_InitStructure.USART_BaudRate = system_config.uart_baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
    
    return true;
}

/**
 * @brief Send data via UART
 */
bool hardware_interface_uart_send(const uint8_t* data, uint16_t length)
{
    if (!hardware_initialized || data == NULL || length == 0) {
        return false;
    }
    
    for (uint16_t i = 0; i < length; i++) {
        /* Wait for transmit buffer to be empty */
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        
        /* Send byte */
        USART_SendData(USART1, data[i]);
    }
    
    /* Wait for transmission to complete */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    
    return true;
}

/**
 * @brief Receive data via UART
 */
bool hardware_interface_uart_receive(uint8_t* data, uint16_t* length, uint32_t timeout_ms)
{
    if (!hardware_initialized || data == NULL || length == NULL) {
        return false;
    }
    
    uint32_t start_time = HAL_GetTick();
    uint16_t received = 0;
    
    while (received < *length && (HAL_GetTick() - start_time) < timeout_ms) {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
            data[received++] = USART_ReceiveData(USART1);
        }
    }
    
    *length = received;
    return (received > 0);
}

/*============================================================================
 * TIMING INTERFACE
 *============================================================================*/

/**
 * @brief Get current timestamp in milliseconds
 */
uint32_t hardware_interface_get_timestamp_ms(void)
{
    return HAL_GetTick();
}

/**
 * @brief Get current timestamp in microseconds
 */
uint64_t hardware_interface_get_timestamp_us(void)
{
    return (uint64_t)HAL_GetTick() * 1000;
}

/**
 * @brief Delay for specified milliseconds
 */
void hardware_interface_delay_ms(uint32_t delay_ms)
{
    Delay_ms(delay_ms);
}

/**
 * @brief Delay for specified microseconds
 */
void hardware_interface_delay_us(uint32_t delay_us)
{
    Delay_us(delay_us);
}

/*============================================================================
 * DISPLAY INTERFACE
 *============================================================================*/

/**
 * @brief Update display with current system status
 */
void hardware_interface_display_update(void)
{
    if (!hardware_initialized) {
        return;
    }
    
    /* Update display data with current system status */
    display_data.signal_strength = hardware_interface_uwb_get_rssi();
    
    /* Display main screen */
    oled_display_main_screen(&display_data);
}

/**
 * @brief Display measurement result
 */
void hardware_interface_display_measurement(float distance, uint8_t anchor_id)
{
    if (!hardware_initialized) {
        return;
    }
    
    display_data.last_distance = distance;
    display_data.measurement_count++;
    
    char str[32];
    sprintf(str, "A%d: %.2fm", anchor_id, distance);
    
    oled_clear_screen();
    oled_draw_string(0, 0, "Measurement", OLED_FONT_MEDIUM, false);
    oled_draw_string(0, 2, str, OLED_FONT_SMALL, false);
    
    sprintf(str, "RSSI: %ddBm", display_data.signal_strength);
    oled_draw_string(0, 3, str, OLED_FONT_SMALL, false);
    
    sprintf(str, "Count: %lu", display_data.measurement_count);
    oled_draw_string(0, 4, str, OLED_FONT_SMALL, false);
}

/**
 * @brief Display error message
 */
void hardware_interface_display_error(const char* error_msg)
{
    if (!hardware_initialized) {
        return;
    }
    
    display_data.error_count++;
    oled_display_error_screen(error_msg);
}

/**
 * @brief Display network status
 */
void hardware_interface_display_network_status(uint8_t anchor_count, uint8_t tag_count)
{
    if (!hardware_initialized) {
        return;
    }
    
    display_data.anchor_count = anchor_count;
    display_data.tag_count = tag_count;
    
    hardware_interface_display_update();
}

/*============================================================================
 * CONFIGURATION INTERFACE
 *============================================================================*/

/**
 * @brief Get system configuration
 */
const uwb_config_t* hardware_interface_get_config(void)
{
    return &system_config;
}

/**
 * @brief Set system configuration
 */
bool hardware_interface_set_config(const uwb_config_t* config)
{
    if (config == NULL) {
        return false;
    }
    
    memcpy(&system_config, config, sizeof(uwb_config_t));
    
    /* Save to flash */
    return (flash_save_uwb_config(&system_config) == FLASH_SUCCESS);
}

/**
 * @brief Get calibration data
 */
const uwb_calibration_t* hardware_interface_get_calibration(void)
{
    return &system_calibration;
}

/**
 * @brief Set calibration data
 */
bool hardware_interface_set_calibration(const uwb_calibration_t* calibration)
{
    if (calibration == NULL) {
        return false;
    }
    
    memcpy(&system_calibration, calibration, sizeof(uwb_calibration_t));
    
    /* Save to flash */
    return (flash_save_calibration(&system_calibration) == FLASH_SUCCESS);
}

/*============================================================================
 * STATUS AND DIAGNOSTICS
 *============================================================================*/

/**
 * @brief Get hardware status
 */
hardware_status_t hardware_interface_get_status(void)
{
    hardware_status_t status = {0};
    
    status.initialized = hardware_initialized;
    status.uwb_ready = hardware_initialized;
    status.uart_ready = hardware_initialized;
    status.flash_ready = flash_is_uwb_config_valid();
    status.display_ready = oled_get_status().initialized;
    status.uptime_ms = HAL_GetTick();
    
    return status;
}

/**
 * @brief Perform system self-test
 */
bool hardware_interface_self_test(void)
{
    if (!hardware_initialized) {
        return false;
    }
    
    bool result = true;
    
    /* Test OLED display */
    if (!oled_self_test()) {
        result = false;
    }
    
    /* Test flash memory */
    uwb_config_t test_config;
    if (flash_load_uwb_config(&test_config) != FLASH_SUCCESS) {
        result = false;
    }
    
    /* Test UWB radio */
    uint32_t device_id = dwt_readdevid();
    if (device_id != DWT_DEVICE_ID) {
        result = false;
    }
    
    return result;
}

/**
 * @brief System reset
 */
void hardware_interface_system_reset(void)
{
    NVIC_SystemReset();
}
