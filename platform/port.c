/**
 * @file    port.c
 * @brief   STM32F103 Hardware Abstraction Layer Implementation
 *
 * This file provides the implementation of hardware-specific functions for the
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

#include "port.h"
#include "spi.h"
#include "delay/delay.h"

/*============================================================================
 * PRIVATE VARIABLES
 *============================================================================*/

#if DECAIRQ_EXTI_USEIRQ
/* UWB IC interrupt handler */
static port_dwic_isr_t port_dwic_isr = NULL;
#endif

/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *============================================================================*/

static void configure_unused_pins(void);

/*============================================================================
 * SYSTEM CONFIGURATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Configure system clocks for STM32F103CB
 *
 * Sets up:
 * - HSE (8MHz external crystal)
 * - PLL to achieve 72MHz system clock
 * - Peripheral clocks (APB1: 36MHz, APB2: 72MHz)
 * - Flash latency settings
 */
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    RCC_ClocksTypeDef RCC_ClockFreq;

    /* Reset RCC to default state */
    RCC_DeInit();

    /* Enable HSE (High Speed External) oscillator */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait for HSE to be ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus != ERROR) {
        /* Initialize system (PLL configuration) */
        SystemInit();

        /* Enable Prefetch Buffer for better performance */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        /* Configure Flash latency for 72MHz operation */
        FLASH_SetLatency(FLASH_Latency_2);

        /* Configure AHB clock (HCLK = SYSCLK = 72MHz) */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* Configure APB2 clock (PCLK2 = HCLK = 72MHz) */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Configure APB1 clock (PCLK1 = HCLK/2 = 36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);

        /* Configure ADC clock (max 14MHz) */
        RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    }

    /* Get actual clock frequencies */
    RCC_GetClocksFreq(&RCC_ClockFreq);

    /* Enable peripheral clocks */
    /* SPI1 for UWB communication */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* Enable all GPIO ports and AFIO */
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
        ENABLE);
}

/**
 * @brief Configure SysTick timer for system timing
 *
 * Sets up SysTick to generate interrupts at 1MHz rate (1μs resolution)
 * for precise timing measurements required by UWB ranging.
 */
void SysTick_Configuration(void)
{
    /* Configure SysTick for 1MHz tick rate */
    SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC);
}

/**
 * @brief Configure GPIO pins for UWB interface and system I/O
 */
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*------------------------------------------------------------------------
     * Configure LED GPIO (PB1)
     *------------------------------------------------------------------------*/
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
    LED_OFF(); /* Initialize LED to OFF state */

    /*------------------------------------------------------------------------
     * Configure user input GPIOs (PB14, PB15)
     *------------------------------------------------------------------------*/
    GPIO_InitStructure.GPIO_Pin = KEY_MODE_PIN | KEY_SET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; /* Input with pull-up */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStructure);

    /*------------------------------------------------------------------------
     * Configure UWB control GPIOs
     *------------------------------------------------------------------------*/
    /* Reset pin (PA2) - Output */
    GPIO_InitStructure.GPIO_Pin = DW_RESET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(DW_RESET_GPIO_PORT, &GPIO_InitStructure);

    /* Wake-up pin (PA1) - Output */
    GPIO_InitStructure.GPIO_Pin = DW_WAKEUP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(DW_WAKEUP_GPIO_PORT, &GPIO_InitStructure);

    /* External enable pin (PA0) - Output */
    GPIO_InitStructure.GPIO_Pin = DW_EXTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(DW_EXTON_GPIO_PORT, &GPIO_InitStructure);

    /* Initialize UWB control pins to safe states */
    DW_WAKEUP_LOW();
    DW_EXTON_DISABLE();

    /*------------------------------------------------------------------------
     * Configure SPI pins for UWB communication
     *------------------------------------------------------------------------*/
    /* NSS pin (PA4) - Output (software controlled) */
    GPIO_InitStructure.GPIO_Pin = DW_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_NSS_GPIO_PORT, &GPIO_InitStructure);
    SPI_CS_DESELECT(); /* Initialize CS to deselected state */

    /* Configure SPI peripheral pins */
    SPI_Configuration();

#if DECAIRQ_EXTI_USEIRQ
    /*------------------------------------------------------------------------
     * Configure UWB interrupt pin (PA3) if using interrupt mode
     *------------------------------------------------------------------------*/
    GPIO_InitStructure.GPIO_Pin = DW_IRQ_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; /* Input with pull-down */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_IRQ_GPIO_PORT, &GPIO_InitStructure);
#endif

    /* Configure unused pins to analog input to reduce power consumption */
    configure_unused_pins();
}

/**
 * @brief Configure unused GPIO pins to reduce power consumption
 */
static void configure_unused_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure unused pins as analog input to minimize power consumption */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

    /* Configure unused GPIOA pins */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure unused GPIOB pins */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 |
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 |
                                  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 |
                                  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 |
                                  GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure all GPIOC pins as analog (not used in this design) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 * @brief Initialize all system peripherals
 */
void peripherals_init(void)
{
    /* System configuration already done in main initialization sequence */
    /* Additional peripheral initialization can be added here */

    /* Initialize UWB reset interrupt if needed */
    setup_DWICRSTnIRQ(1);
}

/*============================================================================
 * UWB DEVICE CONTROL FUNCTIONS
 *============================================================================*/

/**
 * @brief Reset the UWB transceiver chip
 */
void reset_DWIC(void)
{
    /* Assert reset (active low) */
    GPIO_SET_LOW(DW_RESET_GPIO_PORT, DW_RESET_PIN);

    /* Hold reset for minimum time */
    Delay_ms(2);

    /* Release reset */
    GPIO_SET_HIGH(DW_RESET_GPIO_PORT, DW_RESET_PIN);

    /* Wait for chip to come out of reset */
    Delay_ms(5);
}

/**
 * @brief Wake up UWB device using I/O toggle
 */
void wakeup_device_with_io(void)
{
    /* Toggle wake-up pin to wake device from sleep */
    DW_WAKEUP_HIGH();
    Delay_ms(1);
    DW_WAKEUP_LOW();

    /* Wait for device to wake up */
    Delay_ms(2);
}

/*============================================================================
 * INTERRUPT MANAGEMENT FUNCTIONS
 *============================================================================*/

/**
 * @brief Install UWB interrupt service routine
 *
 * @param isr Function pointer to interrupt handler
 */
void port_set_dwic_isr(port_dwic_isr_t isr)
{
#if DECAIRQ_EXTI_USEIRQ
    /* Disable interrupts while installing handler */
    port_DisableEXT_IRQ();

    /* Install the handler */
    port_dwic_isr = isr;

    /* Re-enable interrupts if handler is valid */
    if (isr != NULL) {
        port_EnableEXT_IRQ();
    }
#endif
}

/**
 * @brief Setup UWB reset interrupt
 *
 * @param enable 1 to enable interrupt, 0 to disable
 */
void setup_DWICRSTnIRQ(int enable)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    if (enable) {
        /* Configure EXTI line for reset pin */
        GPIO_EXTILineConfig(DW_RESET_EXTI_PORT_SOURCE, DW_RESET_EXTI_PIN_SOURCE);

        /* Configure EXTI line */
        EXTI_InitStructure.EXTI_Line = DW_RESET_EXTI_LINE;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        /* Configure NVIC */
        NVIC_InitStructure.NVIC_IRQChannel = DW_RESET_EXTI_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    } else {
        /* Disable EXTI line */
        EXTI_InitStructure.EXTI_Line = DW_RESET_EXTI_LINE;
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
        EXTI_Init(&EXTI_InitStructure);

        /* Disable NVIC */
        NVIC_InitStructure.NVIC_IRQChannel = DW_RESET_EXTI_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
}

/**
 * @brief Check if external interrupt is enabled
 *
 * @param IRQn Interrupt number to check
 * @return IT_SET if enabled, IT_RESET if disabled
 */
ITStatus EXTI_GetITEnStatus(IRQn_Type IRQn)
{
    return (NVIC->ISER[IRQn >> 0x05] & (uint32_t)(1 << (IRQn & 0x1F))) ? SET : RESET;
}

/*============================================================================
 * INTERRUPT HANDLERS
 *============================================================================*/

/**
 * @brief Process UWB reset interrupt
 */
void process_dwRSTn_irq(void)
{
    /* Clear the EXTI line pending bit */
    EXTI_ClearITPendingBit(DW_RESET_EXTI_LINE);

    /* Handle reset event - reinitialize UWB device */
    /* This can be implemented based on application requirements */
}

/**
 * @brief Process UWB data ready interrupt
 */
void process_deca_irq(void)
{
#if DECAIRQ_EXTI_USEIRQ
    /* Call the installed handler if available */
    if (port_dwic_isr != NULL) {
        port_dwic_isr();
    }
#endif
}

/*============================================================================
 * INTERRUPT SERVICE ROUTINES
 *============================================================================*/

/**
 * @brief EXTI2 interrupt handler (UWB reset pin)
 */
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(DW_RESET_EXTI_LINE) != RESET) {
        process_dwRSTn_irq();
    }
}

#if DECAIRQ_EXTI_USEIRQ
/**
 * @brief EXTI3 interrupt handler (UWB data ready pin)
 */
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(DW_IRQ_EXTI_LINE) != RESET) {
        /* Clear the EXTI line pending bit */
        EXTI_ClearITPendingBit(DW_IRQ_EXTI_LINE);

        /* Process the interrupt */
        process_deca_irq();
    }
}
#endif
