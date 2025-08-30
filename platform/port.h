/**
 * @file    port.h
 * @brief   STM32F103 Hardware Abstraction Layer for UWB Positioning System
 *
 * This file provides hardware-specific definitions and functions for the
 * STM32F103CB microcontroller interfacing with DecaWave UWB transceiver.
 *
 * @attention
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 * Refactored for UWB PG3.9 project.
 * All rights reserved.
 *
 * @author DecaWave/Refactored
 * @date 2024
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Standard library includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* STM32 HAL includes */
#include "stm32f10x.h"

/*============================================================================
 * SYSTEM CONFIGURATION
 *============================================================================*/

/* System timing configuration */
#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC                  1000000UL   /* Microsecond timebase */

/* Enable/disable features */
#define DECAIRQ_EXTI_USEIRQ             0           /* Use polling instead of IRQ */

/*============================================================================
 * TYPE DEFINITIONS
 *============================================================================*/

/**
 * @brief UWB IC interrupt handler function pointer type
 */
typedef void (*port_dwic_isr_t)(void);

/*============================================================================
 * GPIO UTILITY MACROS
 *============================================================================*/

/**
 * @brief Fast GPIO manipulation macros for STM32F103
 * These macros provide direct register access for optimal performance
 */
#define GPIO_SET_HIGH(port, pin)        ((port)->BSRR = (pin))
#define GPIO_SET_LOW(port, pin)         ((port)->BRR = (pin))
#define GPIO_TOGGLE(port, pin)          ((port)->ODR ^= (pin))
#define GPIO_READ_PIN(port, pin)        GPIO_ReadInputDataBit((port), (pin))

/*============================================================================
 * LED INTERFACE
 *============================================================================*/

/* LED configuration */
#define LED_PIN                         GPIO_Pin_1
#define LED_GPIO_PORT                   GPIOB

/* LED control macros */
#define LED_ON()                        GPIO_SET_HIGH(LED_GPIO_PORT, LED_PIN)
#define LED_OFF()                       GPIO_SET_LOW(LED_GPIO_PORT, LED_PIN)
#define LED_TOGGLE()                    GPIO_TOGGLE(LED_GPIO_PORT, LED_PIN)

/*============================================================================
 * USER INPUT INTERFACE
 *============================================================================*/

/* User button configuration */
#define KEY_MODE_PIN                    GPIO_Pin_14
#define KEY_SET_PIN                     GPIO_Pin_15
#define KEY_GPIO_PORT                   GPIOB

/* Button reading macros */
#define Read_KEY_MODE()                 GPIO_READ_PIN(KEY_GPIO_PORT, KEY_MODE_PIN)
#define Read_KEY_SET()                  GPIO_READ_PIN(KEY_GPIO_PORT, KEY_SET_PIN)

/*============================================================================
 * UWB TRANSCEIVER INTERFACE
 *============================================================================*/

/* UWB chip control pins */
#define DW_RESET_PIN                    GPIO_Pin_2
#define DW_RESET_GPIO_PORT              GPIOA
#define DW_WAKEUP_PIN                   GPIO_Pin_1
#define DW_WAKEUP_GPIO_PORT             GPIOA
#define DW_EXTON_PIN                    GPIO_Pin_0
#define DW_EXTON_GPIO_PORT              GPIOA

/* UWB SPI interface pins */
#define DW_NSS_PIN                      GPIO_Pin_4
#define DW_NSS_GPIO_PORT                GPIOA

/* UWB control macros */
#define DW_WAKEUP_HIGH()                GPIO_SET_HIGH(DW_WAKEUP_GPIO_PORT, DW_WAKEUP_PIN)
#define DW_WAKEUP_LOW()                 GPIO_SET_LOW(DW_WAKEUP_GPIO_PORT, DW_WAKEUP_PIN)
#define DW_EXTON_ENABLE()               GPIO_SET_HIGH(DW_EXTON_GPIO_PORT, DW_EXTON_PIN)
#define DW_EXTON_DISABLE()              GPIO_SET_LOW(DW_EXTON_GPIO_PORT, DW_EXTON_PIN)

/* SPI chip select control */
#define SPI_CS_SELECT()                 GPIO_SET_LOW(DW_NSS_GPIO_PORT, DW_NSS_PIN)
#define SPI_CS_DESELECT()               GPIO_SET_HIGH(DW_NSS_GPIO_PORT, DW_NSS_PIN)

/*============================================================================
 * INTERRUPT CONFIGURATION
 *============================================================================*/

/* UWB reset interrupt configuration */
#define DW_RESET_EXTI_LINE              EXTI_Line2
#define DW_RESET_EXTI_PORT_SOURCE       GPIO_PortSourceGPIOA
#define DW_RESET_EXTI_PIN_SOURCE        GPIO_PinSource2
#define DW_RESET_EXTI_IRQn              EXTI2_IRQn

#if DECAIRQ_EXTI_USEIRQ
/* UWB interrupt configuration (when using interrupt mode) */
#define DW_IRQ_PIN                      GPIO_Pin_3
#define DW_IRQ_GPIO_PORT                GPIOA
#define DW_IRQ_EXTI_LINE                EXTI_Line3
#define DW_IRQ_EXTI_PORT_SOURCE         GPIO_PortSourceGPIOA
#define DW_IRQ_EXTI_PIN_SOURCE          GPIO_PinSource3
#define DW_IRQ_EXTI_IRQn                EXTI3_IRQn

/* Interrupt control macros */
#define port_GetEXT_IRQStatus()         EXTI_GetITEnStatus(DW_IRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()           NVIC_DisableIRQ(DW_IRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()            NVIC_EnableIRQ(DW_IRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()             GPIO_READ_PIN(DW_IRQ_GPIO_PORT, DW_IRQ_PIN)
#else
/* Polling mode - dummy macros */
#define port_GetEXT_IRQStatus()         (0)
#define port_DisableEXT_IRQ()           do {} while(0)
#define port_EnableEXT_IRQ()            do {} while(0)
#define port_CheckEXT_IRQ()             (0)
#endif

/*============================================================================
 * LEGACY COMPATIBILITY MACROS
 *============================================================================*/

/* Legacy GPIO macros for backward compatibility */
#define digitalHi(port, pin)            GPIO_SET_HIGH((port), (pin))
#define digitalLo(port, pin)            GPIO_SET_LOW((port), (pin))
#define digitalToggle(port, pin)        GPIO_TOGGLE((port), (pin))

/* Legacy pin definitions */
#define LED_GPIO                        LED_GPIO_PORT
#define KEY_GPIO                        KEY_GPIO_PORT
#define DW_RESET_GPIO                   DW_RESET_GPIO_PORT
#define DW_WAKEUP_GPIO                  DW_WAKEUP_GPIO_PORT
#define DW_EXTON_GPIO                   DW_EXTON_GPIO_PORT
#define DW_NSS_GPIO                     DW_NSS_GPIO_PORT

/* Legacy control macros */
#define EXTON_OPEN()                    DW_EXTON_ENABLE()
#define EXTON_CLOSE()                   DW_EXTON_DISABLE()
#define port_SPIx_set_chip_select()     SPI_CS_DESELECT()
#define port_SPIx_clear_chip_select()   SPI_CS_SELECT()

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief System initialization functions
 */
void RCC_Configuration(void);
void SysTick_Configuration(void);
void GPIO_Configuration(void);
void peripherals_init(void);

/**
 * @brief UWB interrupt management
 */
void port_set_dwic_isr(port_dwic_isr_t isr);
ITStatus EXTI_GetITEnStatus(IRQn_Type IRQn);

/**
 * @brief UWB device control functions
 */
void setup_DWICRSTnIRQ(int enable);
void reset_DWIC(void);
void wakeup_device_with_io(void);

/**
 * @brief Interrupt handlers
 */
void process_dwRSTn_irq(void);
void process_deca_irq(void);

/**
 * @brief Flash memory management functions
 */
void flash_init(void);
bool flash_read_config(void* config, uint16_t size);
bool flash_write_config(const void* config, uint16_t size);

/**
 * @brief OLED display functions
 */
void oled_init_display(void);
void oled_display_status(const char* status);
void oled_display_distance(float distance);
void oled_display_device_info(uint8_t mode, uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
