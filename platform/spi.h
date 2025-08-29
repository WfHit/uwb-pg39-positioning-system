#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f10x.h"

#define SPI_DW1000_PRESCALER		SPI_BaudRatePrescaler_8
#define SPI_DW1000						  SPI1
#define SPI_DW1000_GPIO					GPIOA
#define SPI_DW1000_CS						GPIO_Pin_4
#define SPI_DW1000_CS_GPIO		  GPIOA
#define SPI_DW1000_SCK					GPIO_Pin_5
#define SPI_DW1000_MISO					GPIO_Pin_6
#define SPI_DW1000_MOSI					GPIO_Pin_7

void spi_set_rate_high(void);
void spi_set_rate_low(void);
void SPI_Configuration(void);

#endif
