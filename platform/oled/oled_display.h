/**
 * @file    oled_display.h
 * @brief   SSD1306 OLED Display Driver for UWB Positioning System
 * 
 * This file provides functions for controlling a 0.96" OLED display (128x64)
 * using SSD1306 controller via I2C interface for system status display.
 * 
 * @attention
 * Hardware connections:
 * - VCC: 3.3V or 5V power supply
 * - GND: Ground
 * - SCL: PB12 (I2C Clock)
 * - SDA: PB13 (I2C Data)
 * - RST: PA8 (Reset, optional)
 * 
 * @author  Refactored for UWB PG3.9 project
 * @date    2024
 */

#ifndef __OLED_DISPLAY_H__
#define __OLED_DISPLAY_H__

#include "port.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * DISPLAY CONFIGURATION
 *============================================================================*/

/* Display dimensions */
#define OLED_WIDTH                      128
#define OLED_HEIGHT                     64
#define OLED_PAGES                      8           /* 64 / 8 = 8 pages */

/* I2C configuration */
#define OLED_I2C_ADDRESS                0x78        /* 7-bit address shifted */
#define OLED_I2C_SPEED                  400000      /* 400kHz */

/* GPIO pin definitions */
#define OLED_SCL_PIN                    GPIO_Pin_12
#define OLED_SCL_PORT                   GPIOB
#define OLED_SDA_PIN                    GPIO_Pin_13
#define OLED_SDA_PORT                   GPIOB
#define OLED_RST_PIN                    GPIO_Pin_8
#define OLED_RST_PORT                   GPIOA

/* Font sizes */
#define OLED_FONT_SMALL                 0           /* 6x8 pixels */
#define OLED_FONT_MEDIUM                1           /* 8x16 pixels */
#define OLED_FONT_LARGE                 2           /* 16x32 pixels */

/* Display modes */
#define OLED_MODE_NORMAL                0
#define OLED_MODE_INVERSE               1

/*============================================================================
 * GPIO CONTROL MACROS
 *============================================================================*/

/* I2C bit-bang control */
#define OLED_SCL_HIGH()                 GPIO_SET_HIGH(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SCL_LOW()                  GPIO_SET_LOW(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SDA_HIGH()                 GPIO_SET_HIGH(OLED_SDA_PORT, OLED_SDA_PIN)
#define OLED_SDA_LOW()                  GPIO_SET_LOW(OLED_SDA_PORT, OLED_SDA_PIN)

/* Reset control */
#define OLED_RST_HIGH()                 GPIO_SET_HIGH(OLED_RST_PORT, OLED_RST_PIN)
#define OLED_RST_LOW()                  GPIO_SET_LOW(OLED_RST_PORT, OLED_RST_PIN)

/* SDA input reading */
#define OLED_SDA_READ()                 GPIO_READ_PIN(OLED_SDA_PORT, OLED_SDA_PIN)

/*============================================================================
 * DATA TYPE DEFINITIONS
 *============================================================================*/

/**
 * @brief OLED command/data selector
 */
typedef enum {
    OLED_COMMAND = 0,
    OLED_DATA = 1
} oled_dc_t;

/**
 * @brief Display status structure
 */
typedef struct {
    bool initialized;
    bool display_on;
    uint8_t contrast;
    uint8_t current_page;
    uint8_t current_column;
    bool inverse_mode;
} oled_status_t;

/**
 * @brief UWB system display data
 */
typedef struct {
    /* Device information */
    uint8_t device_mode;            /* Tag or Anchor */
    uint8_t device_id;
    bool uwb_initialized;
    
    /* Network status */
    uint8_t anchor_count;           /* For tags: discovered anchors */
    uint8_t tag_count;              /* For anchors: active tags */
    int8_t signal_strength;         /* RSSI in dBm */
    
    /* Measurement data */
    float last_distance;            /* Last measured distance */
    uint32_t measurement_count;     /* Total measurements */
    uint32_t error_count;           /* Error count */
    
    /* Position (for anchors) */
    float position_x;
    float position_y;
    float position_z;
    
    /* System status */
    bool flash_ok;
    bool uart_ok;
    uint8_t battery_level;          /* 0-100% */
} uwb_display_data_t;

/*============================================================================
 * FUNCTION PROTOTYPES - LOW LEVEL I2C OPERATIONS
 *============================================================================*/

/**
 * @brief I2C communication functions
 */
void oled_i2c_start(void);
void oled_i2c_stop(void);
bool oled_i2c_wait_ack(void);
void oled_i2c_send_ack(void);
void oled_i2c_send_nack(void);
void oled_i2c_send_byte(uint8_t data);
uint8_t oled_i2c_read_byte(void);

/*============================================================================
 * FUNCTION PROTOTYPES - DISPLAY CONTROL
 *============================================================================*/

/**
 * @brief Basic display control
 */
bool oled_init(void);
void oled_reset(void);
void oled_power_on(void);
void oled_power_off(void);
void oled_set_contrast(uint8_t contrast);
void oled_set_display_mode(uint8_t mode);

/**
 * @brief Command and data transmission
 */
void oled_write_command(uint8_t command);
void oled_write_data(uint8_t data);
void oled_write_data_buffer(const uint8_t* buffer, uint16_t length);

/*============================================================================
 * FUNCTION PROTOTYPES - DRAWING OPERATIONS
 *============================================================================*/

/**
 * @brief Basic drawing functions
 */
void oled_clear_screen(void);
void oled_clear_area(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
void oled_set_position(uint8_t x, uint8_t y);
void oled_draw_pixel(uint8_t x, uint8_t y, bool state);
void oled_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, bool state);
void oled_draw_rectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool fill);
void oled_draw_circle(uint8_t x, uint8_t y, uint8_t radius, bool fill);

/**
 * @brief Text display functions
 */
void oled_draw_char(uint8_t x, uint8_t y, char ch, uint8_t font_size, bool inverse);
void oled_draw_string(uint8_t x, uint8_t y, const char* str, uint8_t font_size, bool inverse);
void oled_draw_number(uint8_t x, uint8_t y, uint32_t number, uint8_t font_size, bool inverse);
void oled_draw_float(uint8_t x, uint8_t y, float number, uint8_t decimals, uint8_t font_size, bool inverse);

/**
 * @brief Advanced display functions
 */
void oled_draw_progress_bar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t percentage);
void oled_draw_signal_bars(uint8_t x, uint8_t y, int8_t rssi);
void oled_draw_battery_icon(uint8_t x, uint8_t y, uint8_t level);

/*============================================================================
 * FUNCTION PROTOTYPES - UWB SYSTEM DISPLAY
 *============================================================================*/

/**
 * @brief UWB system specific display functions
 */
void oled_display_uwb_status(const uwb_display_data_t* data);
void oled_display_tag_info(const uwb_display_data_t* data);
void oled_display_anchor_info(const uwb_display_data_t* data);
void oled_display_measurement_data(const uwb_display_data_t* data);
void oled_display_network_status(const uwb_display_data_t* data);
void oled_display_system_info(const uwb_display_data_t* data);

/**
 * @brief Screen layout functions
 */
void oled_display_main_screen(const uwb_display_data_t* data);
void oled_display_config_screen(const uwb_display_data_t* data);
void oled_display_debug_screen(const uwb_display_data_t* data);
void oled_display_error_screen(const char* error_msg);

/*============================================================================
 * FUNCTION PROTOTYPES - UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Utility and helper functions
 */
void oled_update_display(void);
void oled_test_pattern(void);
void oled_screen_saver(bool enable);
uint8_t oled_get_string_width(const char* str, uint8_t font_size);
uint8_t oled_get_char_width(char ch, uint8_t font_size);
uint8_t oled_get_font_height(uint8_t font_size);

/**
 * @brief Status and diagnostics
 */
oled_status_t oled_get_status(void);
bool oled_self_test(void);
void oled_print_info(void);

/*============================================================================
 * BACKWARD COMPATIBILITY MACROS
 *============================================================================*/

/* Legacy function name mappings */
#define OLED_Init()                     oled_init()
#define OLED_Clear()                    oled_clear_screen()
#define OLED_ShowString(x,y,str,size)   oled_draw_string(x,y,str,size,false)
#define OLED_ShowNum(x,y,num,size)      oled_draw_number(x,y,num,size,false)
#define OLED_ShowChar(x,y,chr,size)     oled_draw_char(x,y,chr,size,false)
#define OLED_Display()                  oled_update_display()

/* Legacy constants */
#define Max_Column                      OLED_WIDTH
#define Max_Row                         OLED_HEIGHT
#define SIZE                            OLED_FONT_SMALL
#define X_WIDTH                         OLED_WIDTH
#define Y_WIDTH                         OLED_HEIGHT

#ifdef __cplusplus
}
#endif

#endif /* __OLED_DISPLAY_H__ */
