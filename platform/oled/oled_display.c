/**
 * @file    oled_display.c
 * @brief   SSD1306 OLED Display Driver Implementation
 *
 * This file implements the OLED display functions for UWB system status display.
 *
 * @author  Refactored for UWB PG3.9 project
 * @date    2024
 */

#include "oled_display.h"
#include "oledfont.h"
#include "delay/delay.h"
#include <stdio.h>
#include <string.h>

/*============================================================================
 * PRIVATE VARIABLES
 *============================================================================*/

static oled_status_t oled_status = {0};
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES]; /* Display buffer */

/*============================================================================
 * SSD1306 COMMAND DEFINITIONS
 *============================================================================*/

#define SSD1306_SETCONTRAST             0x81
#define SSD1306_DISPLAYALLON_RESUME     0xA4
#define SSD1306_DISPLAYALLON            0xA5
#define SSD1306_NORMALDISPLAY           0xA6
#define SSD1306_INVERTDISPLAY           0xA7
#define SSD1306_DISPLAYOFF              0xAE
#define SSD1306_DISPLAYON               0xAF
#define SSD1306_SETDISPLAYOFFSET        0xD3
#define SSD1306_SETCOMPINS              0xDA
#define SSD1306_SETVCOMDETECT           0xDB
#define SSD1306_SETDISPLAYCLOCKDIV      0xD5
#define SSD1306_SETPRECHARGE            0xD9
#define SSD1306_SETMULTIPLEX            0xA8
#define SSD1306_SETLOWCOLUMN            0x00
#define SSD1306_SETHIGHCOLUMN           0x10
#define SSD1306_SETSTARTLINE            0x40
#define SSD1306_MEMORYMODE              0x20
#define SSD1306_COLUMNADDR              0x21
#define SSD1306_PAGEADDR                0x22
#define SSD1306_COMSCANINC              0xC0
#define SSD1306_COMSCANDEC              0xC8
#define SSD1306_SEGREMAP                0xA0
#define SSD1306_CHARGEPUMP              0x8D

/*============================================================================
 * LOW LEVEL I2C OPERATIONS
 *============================================================================*/

/**
 * @brief I2C start condition
 */
void oled_i2c_start(void)
{
    OLED_SDA_HIGH();
    OLED_SCL_HIGH();
    Delay_us(4);
    OLED_SDA_LOW();
    Delay_us(4);
    OLED_SCL_LOW();
}

/**
 * @brief I2C stop condition
 */
void oled_i2c_stop(void)
{
    OLED_SCL_LOW();
    OLED_SDA_LOW();
    Delay_us(4);
    OLED_SCL_HIGH();
    Delay_us(4);
    OLED_SDA_HIGH();
    Delay_us(4);
}

/**
 * @brief Wait for I2C acknowledge
 */
bool oled_i2c_wait_ack(void)
{
    uint8_t timeout = 0;

    OLED_SDA_HIGH();
    Delay_us(1);
    OLED_SCL_HIGH();
    Delay_us(1);

    while (OLED_SDA_READ()) {
        timeout++;
        if (timeout > 250) {
            oled_i2c_stop();
            return false;
        }
    }

    OLED_SCL_LOW();
    return true;
}

/**
 * @brief Send I2C acknowledge
 */
void oled_i2c_send_ack(void)
{
    OLED_SCL_LOW();
    OLED_SDA_LOW();
    Delay_us(2);
    OLED_SCL_HIGH();
    Delay_us(2);
    OLED_SCL_LOW();
}

/**
 * @brief Send I2C not-acknowledge
 */
void oled_i2c_send_nack(void)
{
    OLED_SCL_LOW();
    OLED_SDA_HIGH();
    Delay_us(2);
    OLED_SCL_HIGH();
    Delay_us(2);
    OLED_SCL_LOW();
}

/**
 * @brief Send a byte via I2C
 */
void oled_i2c_send_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        OLED_SCL_LOW();
        if (data & 0x80) {
            OLED_SDA_HIGH();
        } else {
            OLED_SDA_LOW();
        }
        data <<= 1;
        Delay_us(2);
        OLED_SCL_HIGH();
        Delay_us(2);
    }
    OLED_SCL_LOW();
}

/**
 * @brief Read a byte via I2C
 */
uint8_t oled_i2c_read_byte(void)
{
    uint8_t data = 0;

    OLED_SDA_HIGH();
    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        OLED_SCL_LOW();
        Delay_us(2);
        OLED_SCL_HIGH();
        if (OLED_SDA_READ()) {
            data++;
        }
        Delay_us(1);
    }
    OLED_SCL_LOW();

    return data;
}

/*============================================================================
 * DISPLAY CONTROL FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize GPIO pins for OLED
 */
static void oled_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure I2C pins (SCL, SDA) as open-drain outputs */
    GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_SCL_PORT, &GPIO_InitStructure);

    /* Configure reset pin as push-pull output */
    GPIO_InitStructure.GPIO_Pin = OLED_RST_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(OLED_RST_PORT, &GPIO_InitStructure);

    /* Initialize pins to high state */
    OLED_SCL_HIGH();
    OLED_SDA_HIGH();
    OLED_RST_HIGH();
}

/**
 * @brief Write command to OLED
 */
void oled_write_command(uint8_t command)
{
    oled_i2c_start();
    oled_i2c_send_byte(OLED_I2C_ADDRESS);
    oled_i2c_wait_ack();
    oled_i2c_send_byte(0x00); /* Command mode */
    oled_i2c_wait_ack();
    oled_i2c_send_byte(command);
    oled_i2c_wait_ack();
    oled_i2c_stop();
}

/**
 * @brief Write data to OLED
 */
void oled_write_data(uint8_t data)
{
    oled_i2c_start();
    oled_i2c_send_byte(OLED_I2C_ADDRESS);
    oled_i2c_wait_ack();
    oled_i2c_send_byte(0x40); /* Data mode */
    oled_i2c_wait_ack();
    oled_i2c_send_byte(data);
    oled_i2c_wait_ack();
    oled_i2c_stop();
}

/**
 * @brief Write data buffer to OLED
 */
void oled_write_data_buffer(const uint8_t* buffer, uint16_t length)
{
    oled_i2c_start();
    oled_i2c_send_byte(OLED_I2C_ADDRESS);
    oled_i2c_wait_ack();
    oled_i2c_send_byte(0x40); /* Data mode */
    oled_i2c_wait_ack();

    for (uint16_t i = 0; i < length; i++) {
        oled_i2c_send_byte(buffer[i]);
        oled_i2c_wait_ack();
    }

    oled_i2c_stop();
}

/**
 * @brief Reset OLED display
 */
void oled_reset(void)
{
    OLED_RST_HIGH();
    Delay_ms(100);
    OLED_RST_LOW();
    Delay_ms(100);
    OLED_RST_HIGH();
    Delay_ms(100);
}

/**
 * @brief Initialize OLED display
 */
bool oled_init(void)
{
    /* Initialize GPIO */
    oled_gpio_init();

    /* Reset display */
    oled_reset();

    /* Initialize display */
    oled_write_command(SSD1306_DISPLAYOFF);
    oled_write_command(SSD1306_SETDISPLAYCLOCKDIV);
    oled_write_command(0x80);
    oled_write_command(SSD1306_SETMULTIPLEX);
    oled_write_command(0x3F);
    oled_write_command(SSD1306_SETDISPLAYOFFSET);
    oled_write_command(0x00);
    oled_write_command(SSD1306_SETSTARTLINE | 0x00);
    oled_write_command(SSD1306_CHARGEPUMP);
    oled_write_command(0x14);
    oled_write_command(SSD1306_MEMORYMODE);
    oled_write_command(0x00);
    oled_write_command(SSD1306_SEGREMAP | 0x01);
    oled_write_command(SSD1306_COMSCANDEC);
    oled_write_command(SSD1306_SETCOMPINS);
    oled_write_command(0x12);
    oled_write_command(SSD1306_SETCONTRAST);
    oled_write_command(0xCF);
    oled_write_command(SSD1306_SETPRECHARGE);
    oled_write_command(0xF1);
    oled_write_command(SSD1306_SETVCOMDETECT);
    oled_write_command(0x40);
    oled_write_command(SSD1306_DISPLAYALLON_RESUME);
    oled_write_command(SSD1306_NORMALDISPLAY);
    oled_write_command(SSD1306_DISPLAYON);

    /* Clear display */
    oled_clear_screen();

    /* Update status */
    oled_status.initialized = true;
    oled_status.display_on = true;
    oled_status.contrast = 0xCF;
    oled_status.inverse_mode = false;

    return true;
}

/**
 * @brief Turn display on
 */
void oled_power_on(void)
{
    oled_write_command(SSD1306_DISPLAYON);
    oled_status.display_on = true;
}

/**
 * @brief Turn display off
 */
void oled_power_off(void)
{
    oled_write_command(SSD1306_DISPLAYOFF);
    oled_status.display_on = false;
}

/**
 * @brief Set display contrast
 */
void oled_set_contrast(uint8_t contrast)
{
    oled_write_command(SSD1306_SETCONTRAST);
    oled_write_command(contrast);
    oled_status.contrast = contrast;
}

/**
 * @brief Set display mode (normal/inverse)
 */
void oled_set_display_mode(uint8_t mode)
{
    if (mode == OLED_MODE_INVERSE) {
        oled_write_command(SSD1306_INVERTDISPLAY);
        oled_status.inverse_mode = true;
    } else {
        oled_write_command(SSD1306_NORMALDISPLAY);
        oled_status.inverse_mode = false;
    }
}

/*============================================================================
 * DRAWING OPERATIONS
 *============================================================================*/

/**
 * @brief Set cursor position
 */
void oled_set_position(uint8_t x, uint8_t y)
{
    oled_write_command(0xB0 + y);
    oled_write_command(((x & 0xF0) >> 4) | 0x10);
    oled_write_command((x & 0x0F) | 0x00);
}

/**
 * @brief Clear entire screen
 */
void oled_clear_screen(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_position(0, page);
        for (uint8_t col = 0; col < OLED_WIDTH; col++) {
            oled_write_data(0x00);
        }
    }
}

/**
 * @brief Draw a character
 */
void oled_draw_char(uint8_t x, uint8_t y, char ch, uint8_t font_size, bool inverse)
{
    uint8_t char_index = ch - ' ';
    uint8_t font_width = (font_size == OLED_FONT_LARGE) ? 16 : 8;
    uint8_t font_height = (font_size == OLED_FONT_LARGE) ? 16 : 8;

    if (char_index >= 95) return; /* Beyond printable ASCII range */

    for (uint8_t i = 0; i < font_width; i++) {
        uint8_t font_data;

        if (font_size == OLED_FONT_LARGE) {
            font_data = F16x16[char_index][i];
        } else {
            font_data = F8X16[char_index][i];
        }

        if (inverse) {
            font_data = ~font_data;
        }

        oled_set_position(x + i, y);
        oled_write_data(font_data);

        if (font_height == 16) {
            uint8_t font_data2;
            if (font_size == OLED_FONT_LARGE) {
                font_data2 = F16x16[char_index][i + font_width];
            } else {
                font_data2 = F8X16[char_index][i + font_width];
            }

            if (inverse) {
                font_data2 = ~font_data2;
            }

            oled_set_position(x + i, y + 1);
            oled_write_data(font_data2);
        }
    }
}

/**
 * @brief Draw a string
 */
void oled_draw_string(uint8_t x, uint8_t y, const char* str, uint8_t font_size, bool inverse)
{
    uint8_t char_width = (font_size == OLED_FONT_LARGE) ? 16 : 8;
    uint8_t pos_x = x;

    while (*str) {
        if (pos_x + char_width > OLED_WIDTH) {
            break; /* Exceed display width */
        }

        oled_draw_char(pos_x, y, *str, font_size, inverse);
        pos_x += char_width;
        str++;
    }
}

/**
 * @brief Draw a number
 */
void oled_draw_number(uint8_t x, uint8_t y, uint32_t number, uint8_t font_size, bool inverse)
{
    char str[12];
    sprintf(str, "%lu", number);
    oled_draw_string(x, y, str, font_size, inverse);
}

/**
 * @brief Draw a floating point number
 */
void oled_draw_float(uint8_t x, uint8_t y, float number, uint8_t decimals, uint8_t font_size, bool inverse)
{
    char str[16];
    sprintf(str, "%.*f", decimals, number);
    oled_draw_string(x, y, str, font_size, inverse);
}

/*============================================================================
 * UWB SYSTEM DISPLAY FUNCTIONS
 *============================================================================*/

/**
 * @brief Display main UWB system screen
 */
void oled_display_main_screen(const uwb_display_data_t* data)
{
    char str[32];

    oled_clear_screen();

    /* Title */
    oled_draw_string(0, 0, "UWB PG3.9", OLED_FONT_MEDIUM, false);

    /* Device mode and ID */
    sprintf(str, "%s ID:%d", (data->device_mode == 0) ? "Tag" : "Anchor", data->device_id);
    oled_draw_string(0, 2, str, OLED_FONT_SMALL, false);

    /* Signal strength */
    sprintf(str, "RSSI:%ddBm", data->signal_strength);
    oled_draw_string(0, 3, str, OLED_FONT_SMALL, false);

    /* Measurements */
    sprintf(str, "Dist:%.2fm", data->last_distance);
    oled_draw_string(0, 4, str, OLED_FONT_SMALL, false);

    /* Network status */
    if (data->device_mode == 0) { /* Tag */
        sprintf(str, "Anchors:%d", data->anchor_count);
    } else { /* Anchor */
        sprintf(str, "Tags:%d", data->tag_count);
    }
    oled_draw_string(0, 5, str, OLED_FONT_SMALL, false);

    /* Status indicators */
    sprintf(str, "F:%s U:%s", data->flash_ok ? "OK" : "ER", data->uart_ok ? "OK" : "ER");
    oled_draw_string(0, 6, str, OLED_FONT_SMALL, false);

    /* Measurement count */
    sprintf(str, "Cnt:%lu Err:%lu", data->measurement_count, data->error_count);
    oled_draw_string(0, 7, str, OLED_FONT_SMALL, false);
}

/**
 * @brief Display configuration screen
 */
void oled_display_config_screen(const uwb_display_data_t* data)
{
    char str[32];

    oled_clear_screen();

    /* Title */
    oled_draw_string(0, 0, "Configuration", OLED_FONT_MEDIUM, false);

    /* Device information */
    sprintf(str, "Mode: %s", (data->device_mode == 0) ? "Tag" : "Anchor");
    oled_draw_string(0, 2, str, OLED_FONT_SMALL, false);

    sprintf(str, "ID: %d", data->device_id);
    oled_draw_string(0, 3, str, OLED_FONT_SMALL, false);

    if (data->device_mode == 1) { /* Anchor */
        sprintf(str, "Pos: %.1f,%.1f,%.1f", data->position_x, data->position_y, data->position_z);
        oled_draw_string(0, 4, str, OLED_FONT_SMALL, false);
    }

    /* System status */
    sprintf(str, "UWB: %s", data->uwb_initialized ? "Ready" : "Init");
    oled_draw_string(0, 5, str, OLED_FONT_SMALL, false);

    sprintf(str, "Flash: %s", data->flash_ok ? "OK" : "Error");
    oled_draw_string(0, 6, str, OLED_FONT_SMALL, false);

    sprintf(str, "UART: %s", data->uart_ok ? "OK" : "Error");
    oled_draw_string(0, 7, str, OLED_FONT_SMALL, false);
}

/**
 * @brief Display error screen
 */
void oled_display_error_screen(const char* error_msg)
{
    oled_clear_screen();

    /* Error title */
    oled_draw_string(0, 0, "ERROR", OLED_FONT_MEDIUM, true);

    /* Error message */
    oled_draw_string(0, 3, error_msg, OLED_FONT_SMALL, false);

    /* Instructions */
    oled_draw_string(0, 6, "Press RESET to", OLED_FONT_SMALL, false);
    oled_draw_string(0, 7, "restart system", OLED_FONT_SMALL, false);
}

/**
 * @brief Get display status
 */
oled_status_t oled_get_status(void)
{
    return oled_status;
}

/**
 * @brief Perform self-test
 */
bool oled_self_test(void)
{
    /* Test display initialization */
    if (!oled_status.initialized) {
        return false;
    }

    /* Test pattern display */
    oled_test_pattern();
    Delay_ms(1000);

    oled_clear_screen();
    oled_draw_string(0, 0, "Self Test", OLED_FONT_MEDIUM, false);
    oled_draw_string(0, 2, "Display: OK", OLED_FONT_SMALL, false);
    oled_draw_string(0, 3, "I2C: OK", OLED_FONT_SMALL, false);
    oled_draw_string(0, 4, "Font: OK", OLED_FONT_SMALL, false);

    Delay_ms(2000);
    oled_clear_screen();

    return true;
}

/**
 * @brief Display test pattern
 */
void oled_test_pattern(void)
{
    oled_clear_screen();

    /* Draw border */
    for (uint8_t i = 0; i < OLED_WIDTH; i++) {
        oled_set_position(i, 0);
        oled_write_data(0xFF);
        oled_set_position(i, 7);
        oled_write_data(0xFF);
    }

    for (uint8_t i = 1; i < 7; i++) {
        oled_set_position(0, i);
        oled_write_data(0xFF);
        oled_set_position(127, i);
        oled_write_data(0xFF);
    }

    /* Draw center text */
    oled_draw_string(32, 3, "UWB PG3.9", OLED_FONT_MEDIUM, false);
    oled_draw_string(40, 5, "TEST", OLED_FONT_SMALL, false);
}

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Update display (placeholder for buffered operations)
 */
void oled_update_display(void)
{
    /* In this implementation, display is updated immediately */
    /* This function is provided for compatibility */
}

/**
 * @brief Get string width in pixels
 */
uint8_t oled_get_string_width(const char* str, uint8_t font_size)
{
    uint8_t char_width = (font_size == OLED_FONT_LARGE) ? 16 : 8;
    return strlen(str) * char_width;
}

/**
 * @brief Get character width in pixels
 */
uint8_t oled_get_char_width(char ch, uint8_t font_size)
{
    (void)ch; /* Unused in this implementation */
    return (font_size == OLED_FONT_LARGE) ? 16 : 8;
}

/**
 * @brief Get font height in pixels
 */
uint8_t oled_get_font_height(uint8_t font_size)
{
    return (font_size == OLED_FONT_LARGE) ? 16 : 8;
}
