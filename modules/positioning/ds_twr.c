/**
 * @file    ds_twr.c
 * @brief   Double-Sided Two-Way Ranging (DS-TWR) implementation
 *
 * This file implements the DS-TWR protocol for high-precision distance measurement
 * between UWB devices. DS-TWR provides improved accuracy over single-sided ranging
 * by using multiple message exchanges to cancel out clock drift effects.
 *
 * DS-TWR Protocol Sequence:
 * 1. POLL message (Tag -> Anchor)
 * 2. RESPONSE message (Anchor -> Tag)
 * 3. FINAL message (Tag -> Anchor)
 *
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "ds_twr.h"
#include "filter.h"
#include "twr.h"
#include <deca_device_api.h>
#include <deca_regs.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief System calculation state machine flag for DS-TWR process */
uint8_t SYS_Calculate_ACTIVE_FLAG = 0;

/** @brief DWM1000 communication transmit data buffer */
uint8_t DS_send_msg[DS_TX_BUF_LEN];

/** @brief DWM1000 communication receive data buffer */
uint8_t DS_rx_buffer[DS_RX_BUF_LEN];

/*============================================================================
 * PRIVATE CONSTANTS
 *============================================================================*/

/** @brief DS-TWR protocol identifiers */
#define DS_TWR_POLL_MSG_ID      0xAB    ///< POLL message identifier
#define DS_TWR_RESPONSE_MSG_ID  0xBC    ///< RESPONSE message identifier
#define DS_TWR_FINAL_MSG_ID     0xCD    ///< FINAL message identifier
#define DS_TWR_ACK_MSG_ID       0xDE    ///< ACK message identifier

/** @brief Timeout and state values */
#define DS_TWR_RX_TIMEOUT_UUS   9500    ///< Receive timeout in UWB microseconds
#define DS_TWR_STATE_IDLE       0       ///< State machine idle state
#define DS_TWR_STATE_TX_WAIT    1       ///< Waiting for transmission complete
#define DS_TWR_STATE_RX_WAIT    2       ///< Waiting for reception complete

/*============================================================================
 * FORWARD DECLARATIONS
 *============================================================================*/

static void ds_twr_prepare_poll_message(uint8_t sender_id, uint8_t receiver_id, uint8_t mode);
static void ds_twr_prepare_final_message(uint8_t sender_id, uint8_t receiver_id);
static bool ds_twr_validate_message(uint8_t expected_msg_id, uint8_t expected_sender, uint8_t expected_receiver);
static void ds_twr_handle_communication_data(uint8_t receiver_id);
static void ds_twr_extract_timestamps(void);

/*============================================================================
 * DS-TWR COMMUNICATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Perform DS-TWR distance measurement between two devices
 *
 * Implements the complete DS-TWR protocol state machine for measuring
 * distance between a sender (A_ID) and receiver (B_ID). The function
 * manages the entire three-way handshake and calculates the final distance.
 *
 * @param A_ID Sender device ID
 * @param B_ID Receiver device ID
 * @param MODE Operation mode for the measurement
 * @param ret Pointer to return status: 0=no change, 1=success, -1=failed, -2=timeout
 * @return Distance measurement result in centimeters, or 0 if failed
 */
int32_t DW1000send(uint8_t A_ID, uint8_t B_ID, uint8_t MODE, int8_t *ret)
{
    uint16_t result_dist = 0;
    uint32_t Time_ts_F[6];

    switch (SYS_Calculate_ACTIVE_FLAG) {
        case 0:  // Initialize and send POLL message
            ds_twr_prepare_poll_message(A_ID, B_ID, MODE);
            dwt_writetxdata(DS_POLL_LEN, DS_send_msg, 0);
            dwt_writetxfctrl(DS_POLL_LEN, 0, 1);
            dwt_setrxaftertxdelay(0);
            dwt_setrxtimeout(DS_TWR_RX_TIMEOUT_UUS);
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
            SYS_Calculate_ACTIVE_FLAG = 1;
            break;

        case 1:  // Wait for transmission complete and reception
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            if (status_reg & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
                SYS_Calculate_ACTIVE_FLAG = 2;
            } else {
                return 0;  // Still waiting
            }
            break;

        case 2:  // Process POLL transmission result
            frame_seq_nb = (frame_seq_nb < 0xFF) ? frame_seq_nb + 1 : 0;

            if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
                // Reception successful
                SYS_Calculate_ACTIVE_FLAG = 3;
            } else {
                // Reception failed or timeout
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                if (status_reg & SYS_STATUS_ALL_RX_TO) {
                    *ret = -2;  // Timeout
                }
                SYS_Calculate_ACTIVE_FLAG = 0;
                return 0;
            }
            break;

        case 3:  // Validate RESPONSE message
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX;
            dwt_readrxdata(DS_rx_buffer, frame_len, 0);

            if (ds_twr_validate_message(DS_TWR_RESPONSE_MSG_ID, B_ID, A_ID)) {
                SYS_Calculate_ACTIVE_FLAG = 4;
            } else {
                SYS_Calculate_ACTIVE_FLAG = 0;
                return 0;
            }
            break;

        case 4:  // Prepare and send FINAL message
            ds_twr_prepare_final_message(A_ID, B_ID);
            ds_twr_handle_communication_data(B_ID);

            uint8_t send_len = DS_FIX_BUF_LEN;
            // Add communication data length if present
            send_len += DS_send_msg[29];  // Communication data length field

            dwt_writetxdata(send_len, DS_send_msg, 0);
            dwt_writetxfctrl(send_len, 0, 1);
            dwt_setrxaftertxdelay(0);
            dwt_setrxtimeout(DS_TWR_RX_TIMEOUT_UUS);
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
            SYS_Calculate_ACTIVE_FLAG = 5;
            break;

        case 5:  // Wait for FINAL transmission and ACK reception
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            if (status_reg & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
                SYS_Calculate_ACTIVE_FLAG = 6;
            } else {
                return 0;  // Still waiting
            }
            break;

        case 6:  // Process FINAL transmission result
            frame_seq_nb = (frame_seq_nb < 0xFF) ? frame_seq_nb + 1 : 0;

            if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
                // Reception successful
                SYS_Calculate_ACTIVE_FLAG = 7;
            } else {
                // Reception failed or timeout
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                if (status_reg & SYS_STATUS_ALL_RX_TO) {
                    *ret = -2;  // Timeout
                }
                SYS_Calculate_ACTIVE_FLAG = 0;
                return 0;
            }
            break;

        case 7:  // Validate ACK message and extract timestamps
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX;
            dwt_readrxdata(DS_rx_buffer, frame_len, 0);

            if (ds_twr_validate_message(DS_TWR_ACK_MSG_ID, B_ID, A_ID)) {
                dwt_readdiagnostics(&rx_diag);  // Read signal strength information
                SYS_Calculate_ACTIVE_FLAG = 8;
            } else {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                SYS_Calculate_ACTIVE_FLAG = 0;
                return 0;
            }
            break;

        case 8:  // Extract timestamps and calculate distance
            ds_twr_extract_timestamps();

            // Extract received communication data if present
            if (DS_rx_buffer[28] == 1) {
                // Handle received communication data here
                // Implementation depends on communication helper structure
            }

            // Calculate distance using TWR algorithm
            if (Twr_CalDist(B_ID, &result_dist) == 0) {
                // Calculation failed
                SYS_Calculate_ACTIVE_FLAG = 0;
                *ret = -1;
                return result_dist;
            } else {
                // Success
                SYS_Calculate_ACTIVE_FLAG = 0;
                *ret = 1;
                return result_dist;
            }
            break;

        default:
            SYS_Calculate_ACTIVE_FLAG = 0;
            break;
    }

    return 0;  // Operation in progress
}

/*============================================================================
 * PRIVATE HELPER FUNCTIONS
 *============================================================================*/

/**
 * @brief Prepare POLL message for transmission
 *
 * @param sender_id ID of the sending device
 * @param receiver_id ID of the receiving device
 * @param mode Operation mode
 */
static void ds_twr_prepare_poll_message(uint8_t sender_id, uint8_t receiver_id, uint8_t mode)
{
    uint32_t i;

    memset(DS_send_msg, 0, sizeof(DS_send_msg));

    // Message header
    DS_send_msg[0] = sender_id;
    DS_send_msg[1] = receiver_id;
    DS_send_msg[2] = frame_seq_nb;
    DS_send_msg[3] = DS_TWR_POLL_MSG_ID;
    DS_send_msg[4] = mode;

    // Previous positioning success flag
    DS_send_msg[5] = Cal_data[receiver_id].Cal_Flag >> 16 & 0x01;

    // Previous positioning coordinates (big-endian format)
    DS_send_msg[6] = Cal_data[receiver_id].x >> 8;
    DS_send_msg[7] = Cal_data[receiver_id].x & 0x00FF;
    DS_send_msg[8] = Cal_data[receiver_id].y >> 8;
    DS_send_msg[9] = Cal_data[receiver_id].y & 0x00FF;
    DS_send_msg[10] = Cal_data[receiver_id].z >> 8;
    DS_send_msg[11] = Cal_data[receiver_id].z & 0x00FF;

    // Previous measurement success flags
    DS_send_msg[12] = Cal_data[receiver_id].Cal_Flag >> 8 & 0x00FF;
    DS_send_msg[13] = Cal_data[receiver_id].Cal_Flag & 0x00FF;

    // Previous distance measurements (big-endian format)
    for (i = 0; i < ANCHOR_LIST_COUNT; i++) {
        DS_send_msg[14 + i * 2] = Cal_data[receiver_id].Dist[i] >> 8;
        DS_send_msg[15 + i * 2] = Cal_data[receiver_id].Dist[i] & 0x00FF;
    }
}

/**
 * @brief Prepare FINAL message for transmission
 *
 * @param sender_id ID of the sending device
 * @param receiver_id ID of the receiving device
 */
static void ds_twr_prepare_final_message(uint8_t sender_id, uint8_t receiver_id)
{
    // Copy received message as template
    memcpy(DS_send_msg, DS_rx_buffer, DS_TX_BUF_LEN);

    // Record timestamps
    Time_ts[0] = get_tx_timestamp_u64();  // POLL transmission time T1
    Time_ts[3] = get_rx_timestamp_u64();  // RESPONSE reception time T4

    // Set timestamp fields in message
    final_msg_set_ts(&DS_send_msg[4], Time_ts[0]);   // T1 timestamp
    final_msg_set_ts(&DS_send_msg[16], Time_ts[3]);  // T4 timestamp

    // Update message header
    DS_send_msg[0] = sender_id;
    DS_send_msg[1] = receiver_id;
    DS_send_msg[2] = frame_seq_nb;
    DS_send_msg[3] = DS_TWR_FINAL_MSG_ID;
}

/**
 * @brief Validate received message format and addressing
 *
 * @param expected_msg_id Expected message ID
 * @param expected_sender Expected sender ID
 * @param expected_receiver Expected receiver ID
 * @return true if message is valid, false otherwise
 */
static bool ds_twr_validate_message(uint8_t expected_msg_id, uint8_t expected_sender, uint8_t expected_receiver)
{
    return (DS_rx_buffer[3] == expected_msg_id) &&
           (DS_rx_buffer[0] == expected_sender) &&
           (DS_rx_buffer[1] == expected_receiver);
}

/**
 * @brief Handle communication data preparation
 *
 * @param receiver_id ID of the receiving device
 */
static void ds_twr_handle_communication_data(uint8_t receiver_id)
{
    // Placeholder for communication helper implementation
    // This would integrate with the UWB communication helper system
    DS_send_msg[28] = 0;  // Communication data enable flag
    DS_send_msg[29] = 0;  // Communication data length
}

/**
 * @brief Extract all timestamps from received ACK message
 */
static void ds_twr_extract_timestamps(void)
{
    uint32_t Time_ts_F[6];

    // Extract timestamps from received message
    final_msg_get_ts(&DS_rx_buffer[4], &Time_ts_F[0]);   // T1
    final_msg_get_ts(&DS_rx_buffer[8], &Time_ts_F[1]);   // T2
    final_msg_get_ts(&DS_rx_buffer[12], &Time_ts_F[2]);  // T3
    final_msg_get_ts(&DS_rx_buffer[16], &Time_ts_F[3]);  // T4
    final_msg_get_ts(&DS_rx_buffer[24], &Time_ts_F[5]);  // T6

    // Convert to 32-bit values for calculation
    Time_ts[0] = (uint32_t)Time_ts_F[0];  // T1 - POLL transmission
    Time_ts[1] = (uint32_t)Time_ts_F[1];  // T2 - POLL reception
    Time_ts[2] = (uint32_t)Time_ts_F[2];  // T3 - RESPONSE transmission
    Time_ts[3] = (uint32_t)Time_ts_F[3];  // T4 - RESPONSE reception
    Time_ts[4] = (uint32_t)get_tx_timestamp_u64();  // T5 - FINAL transmission
    Time_ts[5] = (uint32_t)Time_ts_F[5];  // T6 - FINAL reception
}

/*============================================================================
 * DS-TWR MODE HANDLERS
 *============================================================================*/

/**
 * @brief DS-TWR mode handler for tag devices
 *
 * Main loop function for tag devices operating in DS-TWR mode.
 * Handles the tag-side protocol state machine.
 */
void MODE_TAG_DS(void)
{
    // Implementation would depend on specific system architecture
    // This is a placeholder for the tag mode implementation
}

/**
 * @brief DS-TWR mode handler for sub-anchor devices
 *
 * Main loop function for sub-anchor devices operating in DS-TWR mode.
 * Handles the sub-anchor-side protocol state machine.
 */
void MODE_SUB_ANCHOR_DS(void)
{
    // Implementation would depend on specific system architecture
    // This is a placeholder for the sub-anchor mode implementation
}

/**
 * @brief DS-TWR mode handler for main anchor devices
 *
 * Main loop function for main anchor devices operating in DS-TWR mode.
 * Handles the main-anchor-side protocol state machine.
 */
void MODE_MAJOR_ANCHOR_DS(void)
{
    // Implementation would depend on specific system architecture
    // This is a placeholder for the main anchor mode implementation
}

