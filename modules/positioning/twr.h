/**
 * @file    twr.h
 * @brief   Two-Way Ranging (TWR) base definitions and structures
 * 
 * This header defines common structures, constants, and functions used by
 * both DS-TWR and HDS-TWR implementations. Contains timing calculations,
 * data structures, and utility functions for UWB ranging operations.
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#ifndef __TWR_H
#define __TWR_H

#include "../core/data_types.h"
#include "../core/system_config.h"
#include "../../drivers/decawave/deca_device_api.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * TYPE DEFINITIONS
 *============================================================================*/

/** @brief 64-bit signed integer type alias */
typedef int64_t int64;

/** @brief 64-bit unsigned integer type alias */
typedef uint64_t uint64;

/*============================================================================
 * UWB TIMING CONSTANTS
 *============================================================================*/

/**
 * @brief UWB microsecond (uus) to device time unit (dtu) conversion factor
 * 
 * UWB microsecond (UUS) to device time unit (DTU, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 μs and 1 μs = 499.2 * 128 dtu.
 */
#define UUS_TO_DWT_TIME         65536

/** @brief Speed of light constant for distance calculations */
#define SPEED_OF_LIGHT          299702547

/** @brief UWB time-of-flight calculation constant */
#define FLIGHT_OF_UWBTIME       (SPEED_OF_LIGHT * DWT_TIME_UNITS)

/** @brief Timestamp data length in bytes */
#define FINAL_MSG_TS_LEN        4

/** @brief Maximum frame length for standard packets */
#define FRAME_LEN_MAX           (127)

/** @brief Maximum frame length for extended packets */
#define FRAME_LEN_MAX_EX        (1023)

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

/**
 * @brief Calculation result data structure for positioning
 * 
 * This structure stores the positioning calculation results including
 * success flags, coordinates, and distance measurements for each tag.
 */
typedef struct 
{
	uint32_t Cal_Flag;                      ///< Calculation success flags (8-bit field, bit 1=success, bits 0-7 represent anchors A-H success)
	int16_t x;                              ///< Calculated X coordinate in centimeters
	int16_t y;                              ///< Calculated Y coordinate in centimeters  
	int16_t z;                              ///< Calculated Z coordinate in centimeters
	uint16_t Dist[ANCHOR_LIST_COUNT];       ///< Distance measurements from tag to anchors A-H
} Cal_data_t;

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief DWM1000 status register value */
extern uint32_t status_reg;

/** @brief DWM1000 received data packet length buffer */
extern uint32_t frame_len;

/** @brief Frame sequence number, incremented with each transmission */
extern uint32_t frame_seq_nb;

/** @brief Timestamp buffer for recording timing events */
extern uint32_t Time_ts[6];

/** @brief Distance calculation results for all anchors */
extern uint32_t Dist_Cal_All[ANCHOR_LIST_COUNT];

/** @brief Calculation data array for all trackable tags */
extern Cal_data_t Cal_data[TAG_USE_MAX_NUM];

/** @brief RX diagnostic information structure */
extern dwt_rxdiag_t rx_diag;

/*============================================================================
 * UART OUTPUT BUFFER DEFINITIONS
 *============================================================================*/

/** @brief UART buffer length for distance data output */
#define TAG_USART_BUF_DIST_LEN  350

/** @brief UART buffer length for RTLS data output */
#define TAG_USART_BUF_RTLS_LEN  60

/** @brief Maximum UART buffer length */
#define TAG_USART_BUF_MAXLEN    (TAG_USART_BUF_DIST_LEN + TAG_USART_BUF_RTLS_LEN)

/** @brief UART output string buffer for tag results */
extern char Tag_Usart_Str[TAG_USART_BUF_MAXLEN];

/*============================================================================
 * FUNCTION DECLARATIONS
 *============================================================================*/

/**
 * @brief Prepare tag positioning result for UART output
 * 
 * @param now_data Pointer to calculation data structure to output
 * @param format Output format selection
 * @param mode Output mode selection
 */
void Prepare_tag_result_output(Cal_data_t* now_data, uint8_t format, uint8_t mode);

/**
 * @brief Extract timestamp from final message timestamp field
 * 
 * @param ts_field Pointer to timestamp field in message
 * @param ts Pointer to extracted timestamp value
 */
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);

/**
 * @brief Extract distance from final message distance field
 * 
 * @param ts_field Pointer to distance field in message  
 * @param dist Pointer to extracted distance value
 */
void final_msg_get_dist(const uint8_t *ts_field, uint32_t *dist);

/**
 * @brief Get transmission timestamp as 64-bit value
 * 
 * @return 64-bit transmission timestamp
 */
uint64 get_tx_timestamp_u64(void);

/**
 * @brief Get reception timestamp as 64-bit value
 * 
 * @return 64-bit reception timestamp
 */
uint64 get_rx_timestamp_u64(void);

/**
 * @brief Set timestamp in final message timestamp field
 * 
 * @param ts_field Pointer to timestamp field in message
 * @param ts Timestamp value to set
 */
void final_msg_set_ts(uint8_t *ts_field, uint32_t ts);

/**
 * @brief Set distance in final message distance field
 * 
 * @param ts_field Pointer to distance field in message
 * @param dist Distance value to set
 */
void final_msg_set_dist(uint8_t *ts_field, uint32_t dist);

/**
 * @brief Calculate distance using TWR algorithm
 * 
 * @param tag_id Tag identifier for calculation
 * @param cal_dist Pointer to calculated distance result
 * @return Calculation status code
 */
int16_t Twr_CalDist(uint8_t tag_id, uint16_t *cal_dist);

/**
 * @brief Calculate distance using ranging algorithm
 * 
 * @param cal_dist Pointer to calculated distance result  
 * @return Calculation status code
 */
int16_t Range_CalDist(uint16_t *cal_dist);

#endif // __TWR_H

