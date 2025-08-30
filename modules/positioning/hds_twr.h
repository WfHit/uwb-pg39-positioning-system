/**
 * @file    hds_twr.h
 * @brief   High-Density Symmetric Two-Way Ranging (HDS-TWR) protocol header
 *
 * This header defines the interface for HDS-TWR protocol implementation.
 * HDS-TWR is optimized for high-density networks with multiple tags and anchors
 * operating simultaneously with minimal interference.
 *
 * @author  UWB PG3.9 project
 * @date    2024
 */

#ifndef HDS_TWR_H
#define HDS_TWR_H

#include "data_types.h"
#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * TIMING CONSTANTS
 *============================================================================*/

/** @brief Anchor wait timeout for final message (0.1ms units) */
#define ANCHOR_WAITFINAL_MAX        120

/*============================================================================
 * BUFFER SIZE DEFINITIONS
 *============================================================================*/

/** @brief Maximum receive buffer length */
#define RX_MAX_LEN                  127

/** @brief Anchor inform message length */
#define TX_ANC_INFORM_LEN           50

/** @brief Tag poll message fixed length */
#define TX_TAG_POLL_FIX_LEN         8

/** @brief Tag poll message total length (including communication data) */
#define TX_TAG_POLL_LEN             (TX_TAG_POLL_FIX_LEN + UWB_COMMU_DATA_MAXLEN)

/** @brief Anchor response message fixed length */
#define TX_ANC_RESP_FIX_LEN         8

/** @brief Anchor response message total length (including communication data) */
#define TX_ANC_RESP_LEN             (TX_ANC_RESP_FIX_LEN + UWB_COMMU_DATA_MAXLEN)

/** @brief Tag final message length */
#define TX_TAG_FINAL_LEN            78

/** @brief Anchor request message length */
#define TX_ANC_REQ_LEN              6

/** @brief Anchor reply message length */
#define TX_ANC_REPLY_LEN            9

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief Anchor inform message buffer */
extern uint8_t TX_ANC_INFORM_BUFF[TX_ANC_INFORM_LEN];

/** @brief Tag poll message buffer */
extern uint8_t TX_TAG_POLL_BUFF[TX_TAG_POLL_LEN];

/** @brief Anchor response message buffer */
extern uint8_t TX_ANC_RESP_BUFF[TX_ANC_RESP_LEN];

/** @brief Tag final message buffer */
extern uint8_t TX_TAG_FINAL_BUFF[TX_TAG_FINAL_LEN];

/** @brief Anchor request message buffer (main anchor) */
extern uint8_t TX_ANC_REQ_BUFF[TX_ANC_REQ_LEN];

/** @brief Anchor reply message buffer (sub anchor) */
extern uint8_t TX_ANC_REPLY_BUFF[TX_ANC_REPLY_LEN];

/** @brief HDS-TWR receive buffer */
extern uint8_t HDS_rx_buffer[RX_MAX_LEN];

/** @brief Calculated distance result */
extern uint16_t Dis_cal;

/** @brief Current frame sequence number */
extern uint8_t frame_now;

/** @brief Anchor receive final message time flag */
extern uint16_t Anc_recvFinal_timeflag;

/** @brief System anchor response flag */
extern uint8_t SYS_ANC_RESP_FLAG;

/** @brief System anchor final message flag */
extern uint8_t SYS_ANC_FINAL_FLAG;

/** @brief System anchor delay send flag */
extern uint8_t SYS_ANC_DELAY_SEND_FLAG;

/*============================================================================
 * FUNCTION DECLARATIONS
 *============================================================================*/

/**
 * @brief Send HDS-TWR response message
 *
 * Handles the anchor-side response transmission in the HDS-TWR protocol.
 * This function manages the timing and data preparation for response messages.
 *
 * @param A_ID Anchor ID (sender)
 * @param B_ID Tag ID (receiver)
 * @param En Enable flag for communication data
 * @return 1 if successful, 0 if failed
 */
uint8_t HDS_TWR_Send_Resp(uint8_t A_ID, uint8_t B_ID, uint8_t En);

/**
 * @brief Receive HDS-TWR final message and calculate distance
 *
 * Handles the anchor-side final message reception and performs the
 * distance calculation using HDS-TWR algorithm.
 *
 * @param A_ID Anchor ID (receiver)
 * @param B_ID Tag ID (sender)
 * @param recv_timeout Reception timeout in UWB microseconds
 * @return 1 if successful, 0 if failed
 */
uint8_t HDS_TWR_Recv_FinalAndCal(uint8_t A_ID, uint8_t B_ID, uint16_t recv_timeout);

/**
 * @brief HDS-TWR mode handler for main anchor devices
 *
 * Main loop function for main anchor devices operating in HDS-TWR mode.
 * Coordinates the network timing and manages sub-anchor communications.
 */
void Mode_MainAnchor_HDS(void);

/**
 * @brief HDS-TWR mode handler for tag devices
 *
 * Main loop function for tag devices operating in HDS-TWR mode.
 * Handles the tag-side protocol state machine and ranging requests.
 */
void Mode_Tag_HDS(void);

/**
 * @brief HDS-TWR mode handler for sub-anchor devices
 *
 * Main loop function for sub-anchor devices operating in HDS-TWR mode.
 * Responds to ranging requests and coordinates with main anchor.
 */
void Mode_Sub_Anchor_HDS(void);

/*============================================================================
 * ADVANCED HDS-TWR FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize HDS-TWR protocol
 *
 * Sets up the HDS-TWR protocol parameters and initializes buffers.
 * Must be called before using HDS-TWR functions.
 *
 * @param device_type Device type: 0=tag, 1=main_anchor, 2=sub_anchor
 * @param device_id Unique device identifier
 * @return true if successful, false otherwise
 */
bool HDS_TWR_Init(uint8_t device_type, uint8_t device_id);

/**
 * @brief Perform HDS-TWR distance measurement
 *
 * High-level function to perform a complete HDS-TWR distance measurement
 * between a tag and anchor with automatic retry and error handling.
 *
 * @param target_id Target device ID
 * @param distance_cm Pointer to store measured distance in centimeters
 * @param max_retries Maximum number of retry attempts
 * @return true if measurement successful, false otherwise
 */
bool HDS_TWR_Measure_Distance(uint8_t target_id, uint16_t* distance_cm, uint8_t max_retries);

/**
 * @brief Set HDS-TWR network parameters
 *
 * Configures network timing parameters for optimal HDS-TWR operation
 * in high-density environments.
 *
 * @param slot_time Time slot duration in microseconds
 * @param guard_time Guard time between slots in microseconds
 * @param max_devices Maximum number of devices in network
 * @return true if parameters valid and set, false otherwise
 */
bool HDS_TWR_Set_Network_Params(uint16_t slot_time, uint16_t guard_time, uint8_t max_devices);

/**
 * @brief Get HDS-TWR network statistics
 *
 * Retrieves performance statistics for the HDS-TWR network including
 * collision rates, successful measurements, and timing accuracy.
 *
 * @param total_attempts Pointer to store total ranging attempts
 * @param successful_ranges Pointer to store successful range measurements
 * @param collision_count Pointer to store detected collision count
 * @param avg_accuracy Pointer to store average measurement accuracy
 */
void HDS_TWR_Get_Statistics(uint32_t* total_attempts, uint32_t* successful_ranges,
                           uint32_t* collision_count, float* avg_accuracy);

/**
 * @brief Reset HDS-TWR statistics
 *
 * Clears all accumulated statistics counters.
 */
void HDS_TWR_Reset_Statistics(void);

/*============================================================================
 * NETWORK COORDINATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Join HDS-TWR network
 *
 * Initiates the network joining process for a device. Handles discovery,
 * time synchronization, and slot assignment.
 *
 * @param network_id Target network identifier
 * @param device_capabilities Device capability flags
 * @return true if successfully joined network, false otherwise
 */
bool HDS_TWR_Join_Network(uint16_t network_id, uint8_t device_capabilities);

/**
 * @brief Leave HDS-TWR network
 *
 * Gracefully leaves the network and notifies other devices.
 *
 * @return true if successfully left network, false otherwise
 */
bool HDS_TWR_Leave_Network(void);

/**
 * @brief Synchronize network timing
 *
 * Performs timing synchronization with the network coordinator
 * to maintain accurate slot timing.
 *
 * @return true if synchronization successful, false otherwise
 */
bool HDS_TWR_Sync_Network(void);

/*============================================================================
 * ERROR CODES
 *============================================================================*/

typedef enum {
    HDS_TWR_SUCCESS = 0,            ///< Operation successful
    HDS_TWR_ERROR_TIMEOUT,          ///< Operation timed out
    HDS_TWR_ERROR_COLLISION,        ///< Network collision detected
    HDS_TWR_ERROR_INVALID_PARAMS,   ///< Invalid parameters provided
    HDS_TWR_ERROR_NETWORK_FULL,     ///< Network capacity exceeded
    HDS_TWR_ERROR_NOT_INITIALIZED,  ///< HDS-TWR not initialized
    HDS_TWR_ERROR_HARDWARE          ///< Hardware error
} hds_twr_error_t;

/**
 * @brief Get last HDS-TWR error
 *
 * @return Last error code from HDS-TWR operations
 */
hds_twr_error_t HDS_TWR_Get_Last_Error(void);

/**
 * @brief Get error description string
 *
 * @param error Error code
 * @return Human-readable error description
 */
const char* HDS_TWR_Get_Error_String(hds_twr_error_t error);

#endif // HDS_TWR_H
