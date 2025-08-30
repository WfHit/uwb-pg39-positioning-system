/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <deca_spi.h>
#include <deca_device_api.h>
#include <port.h>
#include <stm32f10x.h>
#include "spi.h"

/****************************************************************************//**
 *
 *                              DW1000 SPI section
 *
 *******************************************************************************/
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
    return 0;
} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    return 0;
} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI when SPI CRC mode is used
 * Takes two separate byte buffers for write header and write data, and a CRC8 byte which is written last
 * returns 0 for success, or -1 for error
 */
int writetospiwithcrc(
                uint16_t      headerLength,
                const uint8_t *headerBuffer,
                uint16_t      bodyLength,
                const uint8_t *bodyBuffer,
                uint8_t       crc8)
{

	  int i = 0;
    decaIrqStatus_t stat;
    stat = decamutexon() ;
    SPI_DW1000_CS_GPIO->BRR = SPI_DW1000_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPI_DW1000->DR = headerBuffer[i];

    	while ((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

    	SPI_DW1000->DR ;
    }

    for(i=0; i<bodyLength; i++)
    {
     	SPI_DW1000->DR = bodyBuffer[i];

    	while((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

		  SPI_DW1000->DR ;
	  }

		for(i=0; i<1; i++)
    {
     	SPI_DW1000->DR = crc8;

    	while((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

		  SPI_DW1000->DR ;
	  }

    SPI_DW1000_CS_GPIO->BSRR = SPI_DW1000_CS;
    decamutexoff(stat) ;

    return 0;

} // end writetospiwithcrc()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success
 */
//#pragma GCC optimize ("O3")
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodylength, const uint8_t *bodyBuffer)
{
    int i = 0;
    decaIrqStatus_t stat;
    stat = decamutexon() ;
    SPI_DW1000_CS_GPIO->BRR = SPI_DW1000_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPI_DW1000->DR = headerBuffer[i];

    	while ((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

    	SPI_DW1000->DR ;
    }

    for(i=0; i<bodylength; i++)
    {
     	SPI_DW1000->DR = bodyBuffer[i];

    	while((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

		SPI_DW1000->DR ;
	}

    SPI_DW1000_CS_GPIO->BSRR = SPI_DW1000_CS;

    decamutexoff(stat) ;

    return 0;
} // end writetospi()

/*! ------------------------------------------------------------------------------------------------------------------
* @fn spi_cs_low_delay()
*
* @brief This function sets the CS to '0' for ms delay and than raises it up
*
* input parameters:
* @param ms_delay - The delay for CS to be in '0' state
*
* no return value
*/
//uint16_t spi_cs_low_delay(uint16_t delay_ms)
//{
//	/* Blocking: Check whether previous transfer has been finished */
//	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
//	/* Process Locked */
//	__HAL_LOCK(&hspi1);
//	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */
//	Sleep(delay_ms);
//	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */
//	/* Process Unlocked */
//	__HAL_UNLOCK(&hspi1);

//	return 0;
//}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
//#pragma GCC optimize ("O3")
int readfromspi(uint16_t  headerLength,
                uint8_t   *headerBuffer,
                uint16_t  readlength,
                uint8_t   *readBuffer)
{
   int i=0;
   decaIrqStatus_t  stat ;
   stat = decamutexon() ;
   /* Wait for SPIx Tx buffer empty */
   //while (port_SPIx_busy_sending());
   SPI_DW1000_CS_GPIO->BRR = SPI_DW1000_CS;
	 for(i=0; i<headerLength; i++)
	 {
		 SPI_DW1000->DR = headerBuffer[i];
	 	 while((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
		 readBuffer[0] = SPI_DW1000->DR ; // Dummy read as we write the header
	 }

	 for(i=0; i<readlength; i++)
	 {
		 SPI_DW1000->DR = 0;  // Dummy write as we read the message body
		 while((SPI_DW1000->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
		 readBuffer[i] = SPI_DW1000->DR ;//port_SPI_DW1000_receive_data(); //this clears RXNE bit
	 }

	 SPI_DW1000_CS_GPIO->BSRR = SPI_DW1000_CS;
   decamutexoff(stat) ;
	 return 0;

} // end readfromspi()

/****************************************************************************//**
 *
 *                              END OF DW1000 SPI section
 *
 *******************************************************************************/
