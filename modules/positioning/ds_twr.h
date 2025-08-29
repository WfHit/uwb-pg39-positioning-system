/**
 * @file    ds_twr.h
 * @brief   Double-Sided Two-Way Ranging (DS-TWR) protocol header
 * 
 * This header defines the interface for DS-TWR protocol implementation.
 * DS-TWR provides high-precision distance measurement between UWB devices
 * by using multiple message exchanges to improve accuracy.
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#ifndef __DS_TWR_H
#define __DS_TWR_H

#include "twr.h"
#include "data_types.h"
#include <stdint.h>

/*============================================================================
 * BUFFER SIZE DEFINITIONS
 *============================================================================*/

#define DS_FIX_BUF_LEN          (32)    ///< Fixed portion of DS-TWR buffer length
#define DS_TX_BUF_LEN           (127)   ///< Maximum transmit buffer length
#define DS_RX_BUF_LEN           (127)   ///< Maximum receive buffer length

/*============================================================================
 * MESSAGE LENGTH DEFINITIONS
 *============================================================================*/

#define DS_POLL_LEN             (48)    ///< DS-TWR POLL message length
#define DS_RESP_LEN             (14)    ///< DS-TWR RESPONSE message length  
#define DS_FINAL_LEN            DS_TX_BUF_LEN  ///< DS-TWR FINAL message length
#define DS_ACK_LEN              DS_TX_BUF_LEN  ///< DS-TWR ACK message length
#define DS_ASK_LEN              (7)     ///< DS-TWR ASK message length
#define DS_REPLY_LEN            (9)     ///< DS-TWR REPLY message length

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief DWM1000 communication transmit data buffer */
extern uint8_t DS_send_msg[DS_TX_BUF_LEN];

/** @brief DWM1000 communication receive data buffer */
extern uint8_t DS_rx_buffer[DS_RX_BUF_LEN];

/** @brief System calculation state machine flag */
extern uint8_t SYS_Calculate_ACTIVE_FLAG;

/*============================================================================
 * FUNCTION DECLARATIONS
 *============================================================================*/

/**
 * @brief Perform DS-TWR distance measurement between two devices
 * 
 * Implements the complete DS-TWR protocol state machine for measuring
 * distance between a sender (A_ID) and receiver (B_ID).
 * 
 * @param A_ID Sender device ID
 * @param B_ID Receiver device ID  
 * @param MODE Operation mode for the measurement
 * @param ret Pointer to return status: 0=no change, 1=success, -1=failed, -2=timeout
 * @return Distance measurement result or error code
 */
int32_t DW1000send(uint8_t A_ID, uint8_t B_ID, uint8_t MODE, int8_t *ret);

/**
 * @brief DS-TWR mode handler for tag devices
 * 
 * Main loop function for tag devices operating in DS-TWR mode.
 * Handles the tag-side protocol state machine.
 */
void MODE_TAG_DS(void);

/**
 * @brief DS-TWR mode handler for sub-anchor devices
 * 
 * Main loop function for sub-anchor devices operating in DS-TWR mode.
 * Handles the sub-anchor-side protocol state machine.
 */
void MODE_SUB_ANCHOR_DS(void);

/**
 * @brief DS-TWR mode handler for main anchor devices
 * 
 * Main loop function for main anchor devices operating in DS-TWR mode.
 * Handles the main-anchor-side protocol state machine.
 */
void MODE_MAJOR_ANCHOR_DS(void);

#endif // __DS_TWR_H
