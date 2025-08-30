/**
 * @file    flash_config.h
 * @brief   STM32F103 Internal Flash Memory Management for UWB System
 *
 * This file provides functions for reading and writing to the STM32F103
 * internal flash memory for configuration storage and data persistence.
 *
 * @attention
 * Refactored for UWB PG3.9 positioning system.
 *
 * @author  Refactored for UWB PG3.9 project
 * @date    2024
 */

#ifndef __FLASH_CONFIG_H__
#define __FLASH_CONFIG_H__

#include "port.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * FLASH CONFIGURATION
 *============================================================================*/

/* STM32F103CB Flash configuration */
#define STM32_FLASH_SIZE_KB             128         /* Flash size in KB */
#define STM32_FLASH_WRITE_ENABLE        1           /* Enable flash write operations */
#define STM32_FLASH_BASE_ADDR           0x08000000  /* Flash base address */

/* Flash page size for STM32F103 */
#define STM32_FLASH_PAGE_SIZE           1024        /* 1KB per page */
#define STM32_FLASH_TOTAL_PAGES         (STM32_FLASH_SIZE_KB * 1024 / STM32_FLASH_PAGE_SIZE)

/* Flash unlock keys */
#define FLASH_KEY1                      0x45670123
#define FLASH_KEY2                      0xCDEF89AB

/* User data storage area (last 4 pages reserved for user data) */
#define USER_FLASH_START_PAGE           (STM32_FLASH_TOTAL_PAGES - 4)
#define USER_FLASH_START_ADDR           (STM32_FLASH_BASE_ADDR + (USER_FLASH_START_PAGE * STM32_FLASH_PAGE_SIZE))
#define USER_FLASH_SIZE                 (4 * STM32_FLASH_PAGE_SIZE)

/* Configuration storage addresses */
#define CONFIG_FLASH_ADDR               USER_FLASH_START_ADDR
#define CALIBRATION_FLASH_ADDR          (CONFIG_FLASH_ADDR + STM32_FLASH_PAGE_SIZE)
#define LOG_FLASH_ADDR                  (CALIBRATION_FLASH_ADDR + STM32_FLASH_PAGE_SIZE)

/*============================================================================
 * ERROR CODES
 *============================================================================*/

typedef enum {
    FLASH_SUCCESS = 0,
    FLASH_ERROR_BUSY,
    FLASH_ERROR_PROGRAM,
    FLASH_ERROR_WRITE_PROTECTED,
    FLASH_ERROR_TIMEOUT,
    FLASH_ERROR_ALIGNMENT,
    FLASH_ERROR_INVALID_ADDR
} flash_result_t;

/*============================================================================
 * UWB SYSTEM CONFIGURATION STRUCTURE
 *============================================================================*/

/**
 * @brief UWB system configuration stored in flash
 */
typedef struct {
    uint32_t magic_number;              /* Configuration validity marker */
    uint8_t device_mode;                /* Tag or Anchor mode */
    uint8_t device_id;                  /* Unique device identifier */
    uint8_t zone_id;                    /* Zone identifier for multi-zone systems */
    uint8_t channel;                    /* UWB channel (1-7) */
    uint8_t prf;                        /* Pulse repetition frequency */
    uint8_t data_rate;                  /* Data rate configuration */
    uint8_t ranging_algorithm;          /* DS-TWR or HDS-TWR */

    /* Anchor-specific configuration */
    float anchor_position_x;            /* Anchor X coordinate (meters) */
    float anchor_position_y;            /* Anchor Y coordinate (meters) */
    float anchor_position_z;            /* Anchor Z coordinate (meters) */

    /* Calibration data */
    uint16_t antenna_delay_rx;          /* RX antenna delay calibration */
    uint16_t antenna_delay_tx;          /* TX antenna delay calibration */
    int8_t tx_power;                    /* TX power setting */

    /* Communication settings */
    uint32_t uart_baudrate;             /* UART communication speed */
    uint8_t uart_protocol_version;      /* Protocol version */

    /* System settings */
    uint16_t measurement_interval_ms;   /* Measurement interval for tags */
    uint16_t discovery_interval_ms;     /* Discovery interval for tags */
    uint8_t max_anchors_per_measurement; /* Maximum anchors per measurement */

    uint32_t checksum;                  /* Configuration checksum */
} __attribute__((packed)) uwb_config_t;

/**
 * @brief Calibration data structure
 */
typedef struct {
    uint32_t magic_number;              /* Calibration validity marker */

    /* Range calibration */
    float range_bias;                   /* Range measurement bias correction */
    float range_std;                    /* Range measurement standard deviation */

    /* Power calibration */
    int8_t power_cal[8];                /* Power calibration per channel */

    /* Temperature compensation */
    float temp_coeff;                   /* Temperature coefficient */
    float ref_temperature;              /* Reference temperature */

    /* Crystal calibration */
    int16_t xtal_trim;                  /* Crystal trim value */

    uint32_t checksum;                  /* Calibration checksum */
} __attribute__((packed)) uwb_calibration_t;

/*============================================================================
 * FUNCTION PROTOTYPES - LOW LEVEL FLASH OPERATIONS
 *============================================================================*/

/**
 * @brief Low-level flash control functions
 */
void flash_unlock(void);
void flash_lock(void);
flash_result_t flash_get_status(void);
flash_result_t flash_wait_done(uint16_t timeout_ms);
flash_result_t flash_erase_page(uint32_t page_addr);

/**
 * @brief Basic read/write functions
 */
flash_result_t flash_write_halfword(uint32_t addr, uint16_t data);
uint16_t flash_read_halfword(uint32_t addr);
flash_result_t flash_write_byte(uint32_t addr, uint8_t data);
uint8_t flash_read_byte(uint32_t addr);

/**
 * @brief Buffer read/write functions
 */
flash_result_t flash_write_buffer(uint32_t addr, const uint8_t* buffer, uint16_t length);
flash_result_t flash_read_buffer(uint32_t addr, uint8_t* buffer, uint16_t length);

/*============================================================================
 * FUNCTION PROTOTYPES - HIGH LEVEL CONFIGURATION MANAGEMENT
 *============================================================================*/

/**
 * @brief Initialize flash configuration system
 */
flash_result_t flash_config_init(void);

/**
 * @brief UWB configuration management
 */
flash_result_t flash_save_uwb_config(const uwb_config_t* config);
flash_result_t flash_load_uwb_config(uwb_config_t* config);
flash_result_t flash_reset_uwb_config(void);
bool flash_is_uwb_config_valid(void);

/**
 * @brief Calibration data management
 */
flash_result_t flash_save_calibration(const uwb_calibration_t* calibration);
flash_result_t flash_load_calibration(uwb_calibration_t* calibration);
flash_result_t flash_reset_calibration(void);
bool flash_is_calibration_valid(void);

/**
 * @brief Utility functions
 */
uint32_t flash_calculate_checksum(const void* data, uint16_t length);
flash_result_t flash_erase_user_data(void);
void flash_get_info(uint32_t* total_size, uint32_t* free_size, uint32_t* used_size);

/*============================================================================
 * BACKWARD COMPATIBILITY MACROS
 *============================================================================*/

/* Legacy function name mappings */
#define STMFLASH_Unlock()               flash_unlock()
#define STMFLASH_Lock()                 flash_lock()
#define STMFLASH_GetStatus()            flash_get_status()
#define STMFLASH_WaitDone(time)         flash_wait_done(time)
#define STMFLASH_ErasePage(addr)        flash_erase_page(addr)
#define STMFLASH_WriteHalfWord(addr, data) flash_write_halfword(addr, data)
#define STMFLASH_ReadHalfWord(addr)     flash_read_halfword(addr)
#define STMFLASH_WriteByte(addr, data)  flash_write_byte(addr, data)
#define STMFLASH_ReadByte(addr)         flash_read_byte(addr)

/* Legacy constants */
#define STM32_FLASH_SIZE                STM32_FLASH_SIZE_KB
#define STM32_FLASH_WREN                STM32_FLASH_WRITE_ENABLE
#define STM32_FLASH_BASE                STM32_FLASH_BASE_ADDR

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_CONFIG_H__ */
