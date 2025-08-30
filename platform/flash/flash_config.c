/**
 * @file    flash_config.c
 * @brief   STM32F103 Internal Flash Memory Management Implementation
 *
 * This file implements flash memory operations for UWB system configuration
 * and calibration data storage.
 *
 * @author  Refactored for UWB PG3.9 project
 * @date    2024
 */

#include "flash_config.h"
#include "stm32f10x.h"
#include <string.h>

/*============================================================================
 * PRIVATE CONSTANTS
 *============================================================================*/

#define CONFIG_MAGIC_NUMBER             0x55574233  /* "UWB3" */
#define CALIBRATION_MAGIC_NUMBER        0x43414C42  /* "CALB" */
#define FLASH_TIMEOUT_MS                1000

/*============================================================================
 * PRIVATE FUNCTIONS
 *============================================================================*/

/**
 * @brief Calculate CRC32 checksum for data integrity
 */
static uint32_t calculate_crc32(const void* data, uint16_t length)
{
    const uint8_t* bytes = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= bytes[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

/*============================================================================
 * LOW LEVEL FLASH OPERATIONS
 *============================================================================*/

/**
 * @brief Unlock flash for write operations
 */
void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

/**
 * @brief Lock flash to prevent accidental writes
 */
void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * @brief Get current flash status
 */
flash_result_t flash_get_status(void)
{
    uint32_t status = FLASH->SR;

    if (status & FLASH_SR_BSY) {
        return FLASH_ERROR_BUSY;
    }
    if (status & FLASH_SR_PGERR) {
        FLASH->SR = FLASH_SR_PGERR;  /* Clear error flag */
        return FLASH_ERROR_PROGRAM;
    }
    if (status & FLASH_SR_WRPRTERR) {
        FLASH->SR = FLASH_SR_WRPRTERR;  /* Clear error flag */
        return FLASH_ERROR_WRITE_PROTECTED;
    }

    return FLASH_SUCCESS;
}

/**
 * @brief Wait for flash operation to complete
 */
flash_result_t flash_wait_done(uint16_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();

    while (FLASH->SR & FLASH_SR_BSY) {
        if ((HAL_GetTick() - start_time) > timeout_ms) {
            return FLASH_ERROR_TIMEOUT;
        }
    }

    return flash_get_status();
}

/**
 * @brief Erase a flash page
 */
flash_result_t flash_erase_page(uint32_t page_addr)
{
    flash_result_t result;

    /* Check address alignment */
    if (page_addr % STM32_FLASH_PAGE_SIZE != 0) {
        return FLASH_ERROR_ALIGNMENT;
    }

    /* Check address range */
    if (page_addr < USER_FLASH_START_ADDR ||
        page_addr >= (USER_FLASH_START_ADDR + USER_FLASH_SIZE)) {
        return FLASH_ERROR_INVALID_ADDR;
    }

    /* Wait for any ongoing operation */
    result = flash_wait_done(FLASH_TIMEOUT_MS);
    if (result != FLASH_SUCCESS) {
        return result;
    }

    /* Unlock flash */
    flash_unlock();

    /* Set page erase bit */
    FLASH->CR |= FLASH_CR_PER;

    /* Set page address */
    FLASH->AR = page_addr;

    /* Start erase operation */
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait for completion */
    result = flash_wait_done(FLASH_TIMEOUT_MS);

    /* Clear page erase bit */
    FLASH->CR &= ~FLASH_CR_PER;

    /* Lock flash */
    flash_lock();

    return result;
}

/**
 * @brief Write a halfword to flash
 */
flash_result_t flash_write_halfword(uint32_t addr, uint16_t data)
{
    flash_result_t result;

    /* Check address alignment */
    if (addr % 2 != 0) {
        return FLASH_ERROR_ALIGNMENT;
    }

    /* Check if location is already programmed */
    if (*(volatile uint16_t*)addr != 0xFFFF) {
        return FLASH_ERROR_PROGRAM;
    }

    /* Wait for any ongoing operation */
    result = flash_wait_done(FLASH_TIMEOUT_MS);
    if (result != FLASH_SUCCESS) {
        return result;
    }

    /* Unlock flash */
    flash_unlock();

    /* Set programming bit */
    FLASH->CR |= FLASH_CR_PG;

    /* Write data */
    *(volatile uint16_t*)addr = data;

    /* Wait for completion */
    result = flash_wait_done(FLASH_TIMEOUT_MS);

    /* Clear programming bit */
    FLASH->CR &= ~FLASH_CR_PG;

    /* Lock flash */
    flash_lock();

    /* Verify write */
    if (result == FLASH_SUCCESS) {
        if (*(volatile uint16_t*)addr != data) {
            result = FLASH_ERROR_PROGRAM;
        }
    }

    return result;
}

/**
 * @brief Read a halfword from flash
 */
uint16_t flash_read_halfword(uint32_t addr)
{
    return *(volatile uint16_t*)addr;
}

/**
 * @brief Write a byte to flash (uses halfword programming)
 */
flash_result_t flash_write_byte(uint32_t addr, uint8_t data)
{
    uint32_t aligned_addr = addr & ~1;
    uint16_t halfword;

    /* Read existing halfword */
    halfword = flash_read_halfword(aligned_addr);

    /* Modify the appropriate byte */
    if (addr & 1) {
        /* High byte */
        halfword = (halfword & 0x00FF) | ((uint16_t)data << 8);
    } else {
        /* Low byte */
        halfword = (halfword & 0xFF00) | data;
    }

    return flash_write_halfword(aligned_addr, halfword);
}

/**
 * @brief Read a byte from flash
 */
uint8_t flash_read_byte(uint32_t addr)
{
    return *(volatile uint8_t*)addr;
}

/**
 * @brief Write a buffer to flash
 */
flash_result_t flash_write_buffer(uint32_t addr, const uint8_t* buffer, uint16_t length)
{
    flash_result_t result = FLASH_SUCCESS;

    for (uint16_t i = 0; i < length && result == FLASH_SUCCESS; i += 2) {
        uint16_t data;

        if (i + 1 < length) {
            /* Write full halfword */
            data = buffer[i] | (buffer[i + 1] << 8);
        } else {
            /* Write partial halfword (pad with 0xFF) */
            data = buffer[i] | 0xFF00;
        }

        result = flash_write_halfword(addr + i, data);
    }

    return result;
}

/**
 * @brief Read a buffer from flash
 */
flash_result_t flash_read_buffer(uint32_t addr, uint8_t* buffer, uint16_t length)
{
    memcpy(buffer, (void*)addr, length);
    return FLASH_SUCCESS;
}

/*============================================================================
 * HIGH LEVEL CONFIGURATION MANAGEMENT
 *============================================================================*/

/**
 * @brief Initialize flash configuration system
 */
flash_result_t flash_config_init(void)
{
    /* Enable flash clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FLITF, ENABLE);

    /* No additional initialization needed for STM32F103 internal flash */
    return FLASH_SUCCESS;
}

/**
 * @brief Save UWB configuration to flash
 */
flash_result_t flash_save_uwb_config(const uwb_config_t* config)
{
    flash_result_t result;
    uwb_config_t config_copy;

    if (config == NULL) {
        return FLASH_ERROR_INVALID_ADDR;
    }

    /* Make a copy and calculate checksum */
    memcpy(&config_copy, config, sizeof(uwb_config_t));
    config_copy.magic_number = CONFIG_MAGIC_NUMBER;
    config_copy.checksum = calculate_crc32(&config_copy, sizeof(uwb_config_t) - sizeof(uint32_t));

    /* Erase the configuration page */
    result = flash_erase_page(CONFIG_FLASH_ADDR);
    if (result != FLASH_SUCCESS) {
        return result;
    }

    /* Write configuration */
    result = flash_write_buffer(CONFIG_FLASH_ADDR, (uint8_t*)&config_copy, sizeof(uwb_config_t));

    return result;
}

/**
 * @brief Load UWB configuration from flash
 */
flash_result_t flash_load_uwb_config(uwb_config_t* config)
{
    uwb_config_t temp_config;
    uint32_t calculated_checksum;

    if (config == NULL) {
        return FLASH_ERROR_INVALID_ADDR;
    }

    /* Read configuration from flash */
    flash_read_buffer(CONFIG_FLASH_ADDR, (uint8_t*)&temp_config, sizeof(uwb_config_t));

    /* Verify magic number */
    if (temp_config.magic_number != CONFIG_MAGIC_NUMBER) {
        return FLASH_ERROR_PROGRAM;
    }

    /* Verify checksum */
    calculated_checksum = calculate_crc32(&temp_config, sizeof(uwb_config_t) - sizeof(uint32_t));
    if (calculated_checksum != temp_config.checksum) {
        return FLASH_ERROR_PROGRAM;
    }

    /* Copy valid configuration */
    memcpy(config, &temp_config, sizeof(uwb_config_t));

    return FLASH_SUCCESS;
}

/**
 * @brief Reset UWB configuration to defaults
 */
flash_result_t flash_reset_uwb_config(void)
{
    uwb_config_t default_config = {
        .magic_number = CONFIG_MAGIC_NUMBER,
        .device_mode = 0,           /* Tag mode by default */
        .device_id = 1,
        .zone_id = 0,
        .channel = 5,               /* UWB channel 5 */
        .prf = 64,                  /* 64 MHz PRF */
        .data_rate = 1,             /* 850 kbps */
        .ranging_algorithm = 0,     /* DS-TWR */
        .anchor_position_x = 0.0f,
        .anchor_position_y = 0.0f,
        .anchor_position_z = 0.0f,
        .antenna_delay_rx = 16436,  /* Default antenna delay */
        .antenna_delay_tx = 16436,
        .tx_power = 0x0E082848,     /* Default TX power */
        .uart_baudrate = 115200,
        .uart_protocol_version = 1,
        .measurement_interval_ms = 200,     /* 5 Hz */
        .discovery_interval_ms = 5000,      /* 5 seconds */
        .max_anchors_per_measurement = 4,
        .checksum = 0
    };

    return flash_save_uwb_config(&default_config);
}

/**
 * @brief Check if UWB configuration is valid
 */
bool flash_is_uwb_config_valid(void)
{
    uwb_config_t config;
    return (flash_load_uwb_config(&config) == FLASH_SUCCESS);
}

/**
 * @brief Save calibration data to flash
 */
flash_result_t flash_save_calibration(const uwb_calibration_t* calibration)
{
    flash_result_t result;
    uwb_calibration_t cal_copy;

    if (calibration == NULL) {
        return FLASH_ERROR_INVALID_ADDR;
    }

    /* Make a copy and calculate checksum */
    memcpy(&cal_copy, calibration, sizeof(uwb_calibration_t));
    cal_copy.magic_number = CALIBRATION_MAGIC_NUMBER;
    cal_copy.checksum = calculate_crc32(&cal_copy, sizeof(uwb_calibration_t) - sizeof(uint32_t));

    /* Erase the calibration page */
    result = flash_erase_page(CALIBRATION_FLASH_ADDR);
    if (result != FLASH_SUCCESS) {
        return result;
    }

    /* Write calibration */
    result = flash_write_buffer(CALIBRATION_FLASH_ADDR, (uint8_t*)&cal_copy, sizeof(uwb_calibration_t));

    return result;
}

/**
 * @brief Load calibration data from flash
 */
flash_result_t flash_load_calibration(uwb_calibration_t* calibration)
{
    uwb_calibration_t temp_cal;
    uint32_t calculated_checksum;

    if (calibration == NULL) {
        return FLASH_ERROR_INVALID_ADDR;
    }

    /* Read calibration from flash */
    flash_read_buffer(CALIBRATION_FLASH_ADDR, (uint8_t*)&temp_cal, sizeof(uwb_calibration_t));

    /* Verify magic number */
    if (temp_cal.magic_number != CALIBRATION_MAGIC_NUMBER) {
        return FLASH_ERROR_PROGRAM;
    }

    /* Verify checksum */
    calculated_checksum = calculate_crc32(&temp_cal, sizeof(uwb_calibration_t) - sizeof(uint32_t));
    if (calculated_checksum != temp_cal.checksum) {
        return FLASH_ERROR_PROGRAM;
    }

    /* Copy valid calibration */
    memcpy(calibration, &temp_cal, sizeof(uwb_calibration_t));

    return FLASH_SUCCESS;
}

/**
 * @brief Reset calibration data to defaults
 */
flash_result_t flash_reset_calibration(void)
{
    uwb_calibration_t default_cal = {
        .magic_number = CALIBRATION_MAGIC_NUMBER,
        .range_bias = 0.0f,
        .range_std = 0.1f,
        .power_cal = {0, 0, 0, 0, 0, 0, 0, 0},
        .temp_coeff = 0.0f,
        .ref_temperature = 25.0f,
        .xtal_trim = 0,
        .checksum = 0
    };

    return flash_save_calibration(&default_cal);
}

/**
 * @brief Check if calibration data is valid
 */
bool flash_is_calibration_valid(void)
{
    uwb_calibration_t calibration;
    return (flash_load_calibration(&calibration) == FLASH_SUCCESS);
}

/**
 * @brief Calculate checksum for data integrity
 */
uint32_t flash_calculate_checksum(const void* data, uint16_t length)
{
    return calculate_crc32(data, length);
}

/**
 * @brief Erase all user data
 */
flash_result_t flash_erase_user_data(void)
{
    flash_result_t result = FLASH_SUCCESS;

    /* Erase all user data pages */
    for (uint32_t page = 0; page < 4 && result == FLASH_SUCCESS; page++) {
        uint32_t page_addr = USER_FLASH_START_ADDR + (page * STM32_FLASH_PAGE_SIZE);
        result = flash_erase_page(page_addr);
    }

    return result;
}

/**
 * @brief Get flash memory information
 */
void flash_get_info(uint32_t* total_size, uint32_t* free_size, uint32_t* used_size)
{
    if (total_size) {
        *total_size = USER_FLASH_SIZE;
    }

    if (used_size) {
        *used_size = 0;
        if (flash_is_uwb_config_valid()) {
            *used_size += STM32_FLASH_PAGE_SIZE;
        }
        if (flash_is_calibration_valid()) {
            *used_size += STM32_FLASH_PAGE_SIZE;
        }
    }

    if (free_size && used_size) {
        *free_size = USER_FLASH_SIZE - *used_size;
    }
}
