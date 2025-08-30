/**
 * @file    twr.c
 * @brief   Two-Way Ranging (TWR) base implementation for UWB positioning
 *
 * This file implements the core TWR functionality including timestamp handling,
 * distance calculations, and data formatting. Provides the foundation for both
 * DS-TWR and HDS-TWR protocols.
 *
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "twr.h"
#include "filter.h"
#include <deca_device_api.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief Bitfield indicating which anchors have valid calculations */
uint32_t Calculate_FLAG = 0;

/** @brief DWM1000 status register snapshot for debugging */
uint32_t status_reg = 0;

/** @brief DWM1000 received data packet length buffer */
uint32_t frame_len = 0;

/** @brief Timestamp buffer for recording timing events during TWR */
uint32_t Time_ts[6];

/** @brief Distance calculation results for all anchors */
uint32_t Dist_Cal_All[ANCHOR_LIST_COUNT] = {0};

/** @brief Calculation data array for all trackable tags */
Cal_data_t Cal_data[TAG_USE_MAX_NUM];

/** @brief Frame sequence number, incremented with each transmission */
uint32_t frame_seq_nb = 0;

/** @brief RX diagnostic information structure */
dwt_rxdiag_t rx_diag;

/** @brief UART output string buffers */
char Tag_Usart_Str[TAG_USART_BUF_MAXLEN] = {0};
char Tag_ouput_dist_str[TAG_USART_BUF_DIST_LEN] = {0};
char Tag_ouput_rtls_str[TAG_USART_BUF_RTLS_LEN] = {0};

/*============================================================================
 * TIMESTAMP UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Extract timestamp from final message timestamp field
 *
 * Extracts a 32-bit timestamp from a 4-byte field in a TWR message.
 * Uses little-endian byte order.
 *
 * @param ts_field Pointer to timestamp field in message
 * @param ts Pointer to extracted timestamp value
 */
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/**
 * @brief Extract distance from final message distance field
 *
 * Extracts a 32-bit distance value from a 4-byte field in a TWR message.
 * Uses little-endian byte order.
 *
 * @param ts_field Pointer to distance field in message
 * @param dist Pointer to extracted distance value
 */
void final_msg_get_dist(const uint8_t *ts_field, uint32_t *dist)
{
    int i;
    *dist = 0;
    for (i = 0; i < 4; i++) {
        *dist += ts_field[i] << (i * 8);
    }
}

/**
 * @brief Get transmission timestamp as 64-bit value
 *
 * Reads the DWM1000 transmission timestamp and converts it to a 64-bit value.
 * This function assumes timestamp length is 40 bits.
 *
 * @return 64-bit transmission timestamp in device time units
 */
uint64 get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64 ts = 0;
    int i;

    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/**
 * @brief Get reception timestamp as 64-bit value
 *
 * Reads the DWM1000 reception timestamp and converts it to a 64-bit value.
 * This function assumes timestamp length is 40 bits.
 *
 * @return 64-bit reception timestamp in device time units
 */
uint64 get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64 ts = 0;
    int i;

    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/**
 * @brief Set timestamp in final message timestamp field
 *
 * Fills a timestamp field in a TWR message with the given 32-bit value.
 * Uses little-endian byte order (least significant byte at lower address).
 *
 * @param ts_field Pointer to timestamp field in message
 * @param ts Timestamp value to set
 */
void final_msg_set_ts(uint8_t *ts_field, uint32_t ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8_t) ts;
        ts >>= 8;
    }
}

/**
 * @brief Set distance in final message distance field
 *
 * Fills a distance field in a TWR message with the given 32-bit value.
 * Uses little-endian byte order.
 *
 * @param ts_field Pointer to distance field in message
 * @param dist Distance value to set
 */
void final_msg_set_dist(uint8_t *ts_field, uint32_t dist)
{
    int i;
    for (i = 0; i < 4; i++) {
        ts_field[i] = (uint8_t) dist;
        dist >>= 8;
    }
}

/*============================================================================
 * OUTPUT FORMATTING FUNCTIONS
 *============================================================================*/

/**
 * @brief Prepare ASCII-formatted distance output string
 *
 * Formats distance measurements from all anchors into a human-readable
 * ASCII string for UART output.
 *
 * @param now_data Pointer to calculation data structure to format
 */
void Prepare_ascii_dist_output(Cal_data_t* now_data)
{
    char dist_str[25];
    uint8_t i;

    memset(Tag_ouput_dist_str, 0, sizeof(Tag_ouput_dist_str));
    strcpy(Tag_ouput_dist_str, "Dist: ");

    for (i = 0; i < ANCHOR_LIST_COUNT; i++) {
        memset(dist_str, 0, sizeof(dist_str));

        if (i != ANCHOR_LIST_COUNT - 1) {
            // Not the last anchor - add comma separator
            if ((now_data->Cal_Flag >> i & 0x01) == 1) {
                sprintf(dist_str, "Anc%c: %d cm , ", 0x41 + i, now_data->Dist[i]);
            } else {
                sprintf(dist_str, "Anc%c: %d cm , ", 0x41 + i, -1);
            }
        } else {
            // Last anchor - add line ending
            if ((now_data->Cal_Flag >> i & 0x01) == 1) {
                sprintf(dist_str, "Anc%c: %d cm\r\n", 0x41 + i, now_data->Dist[i]);
            } else {
                sprintf(dist_str, "Anc%c: %d cm\r\n", 0x41 + i, -1);
            }
        }
        strcat(Tag_ouput_dist_str, dist_str);
    }
}

/**
 * @brief Prepare ASCII-formatted RTLS positioning output string
 *
 * Formats calculated position coordinates into a human-readable ASCII string
 * for UART output. Supports both 2D and 3D positioning modes.
 *
 * @param now_data Pointer to calculation data structure to format
 * @param mode Positioning mode: 1=2D, 2=3D
 */
void Prepare_ascii_rtls_output(Cal_data_t* now_data, uint8_t mode)
{
    memset(Tag_ouput_rtls_str, 0, sizeof(Tag_ouput_rtls_str));

    if (mode == 1) {
        // 2D positioning mode
        sprintf(Tag_ouput_rtls_str, "Rtls:X = %d cm , Y = %d cm %s\r\n",
                now_data->x, now_data->y,
                (now_data->Cal_Flag >> 16 & 0x01) == 1 ? "Yes" : "No");
    } else if (mode == 2) {
        // 3D positioning mode
        sprintf(Tag_ouput_rtls_str, "Rtls:X = %d cm , Y = %d cm, Z = %d cm %s\r\n",
                now_data->x, now_data->y, now_data->z,
                (now_data->Cal_Flag >> 16 & 0x01) == 1 ? "Yes" : "No");
    }
}

/**
 * @brief Prepare tag positioning result for UART output
 *
 * Prepares the complete output string based on requested format and mode.
 * Combines distance and RTLS data as specified.
 *
 * @param now_data Pointer to calculation data structure to output
 * @param format Output format flags (TAG_OUTPUT_DIST, TAG_OUTPUT_RTLS)
 * @param mode Output mode: 0=distance only, 1=2D positioning, 2=3D positioning
 */
void Prepare_tag_result_output(Cal_data_t* now_data, uint8_t format, uint8_t mode)
{
    memset(Tag_Usart_Str, 0, sizeof(Tag_Usart_Str));

    switch (mode) {
        case 0:  // Distance-only mode
            if (format & TAG_OUTPUT_DIST) {
                Prepare_ascii_dist_output(now_data);
                strcpy(Tag_Usart_Str, Tag_ouput_dist_str);
            }
            break;

        case 1:  // 2D positioning mode
        case 2:  // 3D positioning mode
            if (format & TAG_OUTPUT_RTLS) {
                Prepare_ascii_rtls_output(now_data, mode);
                strcpy(Tag_Usart_Str, Tag_ouput_rtls_str);
            }
            if (format & TAG_OUTPUT_DIST) {
                Prepare_ascii_dist_output(now_data);
                strcat(Tag_Usart_Str, Tag_ouput_dist_str);
            }
            break;

        default:
            break;
    }
}

/*============================================================================
 * DISTANCE CALCULATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Core TWR distance calculation algorithm
 *
 * Implements the standard TWR distance calculation using timestamp measurements.
 * Uses the symmetric double-sided TWR formula to calculate time-of-flight.
 *
 * @param cal_dist Pointer to store calculated distance result in centimeters
 * @return 1 if calculation successful, 0 if failed
 */
uint8_t Twr_Algorithm(uint16_t *cal_dist)
{
    double Ra, Rb, Da, Db;
    int64_t tof_dtu;
    double dist_temp;
    uint16_t temp = 0;
    uint8_t i;

    // Validate all timestamps are available
    for (i = 0; i < 6; i++) {
        if (Time_ts[i] == 0) {
            return 0;  // Missing timestamp
        }
    }

    // Calculate TWR timing parameters
    Ra = (double)(Time_ts[3] - Time_ts[0]);  // Tround1 = T4 - T1
    Rb = (double)(Time_ts[5] - Time_ts[2]);  // Tround2 = T6 - T3
    Da = (double)(Time_ts[4] - Time_ts[3]);  // Treply2 = T5 - T4
    Db = (double)(Time_ts[2] - Time_ts[1]);  // Treply1 = T3 - T2

    // Calculate time-of-flight using symmetric DS-TWR formula
    tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

    // Convert to distance using speed of light
    dist_temp = tof_dtu * FLIGHT_OF_UWBTIME;

    // Validate distance result
    if (dist_temp < 0) {
        if (dist_temp < -0.55) {
            // Large negative distance indicates measurement error
            return 0;
        } else {
            // Small negative values due to clock accuracy - set to zero
            dist_temp = 0;
            *cal_dist = temp;
            return 1;
        }
    }

    // Convert to centimeters and return
    temp = dist_temp * 100;
    *cal_dist = temp;
    return 1;
}

/**
 * @brief Calculate distance using TWR algorithm with filtering
 *
 * Performs TWR distance calculation and applies filtering to improve accuracy.
 * Uses the tag-specific distance filter to smooth measurements.
 *
 * @param tag_id Tag identifier for filtering
 * @param cal_dist Pointer to calculated distance result in centimeters
 * @return 1 if calculation successful, 0 if failed
 */
int16_t Twr_CalDist(uint8_t tag_id, uint16_t *cal_dist)
{
    float temp = 0.0f;

    // Perform basic TWR calculation
    if (Twr_Algorithm(cal_dist) == 0) {
        return 0;  // Calculation failed
    }

    // Apply distance filtering
    temp = *cal_dist;
    *cal_dist = Distance_Filter(tag_id, temp);

    return 1;  // Success
}

/**
 * @brief Calculate distance using ranging algorithm without filtering
 *
 * Performs basic distance calculation without filtering. Used for
 * applications that require raw measurements.
 *
 * @param cal_dist Pointer to calculated distance result in centimeters
 * @return 0 if calculation successful, -1 if failed
 */
int16_t Range_CalDist(uint16_t *cal_dist)
{
    if (Twr_Algorithm(cal_dist) == 0) {
        return -1;  // Calculation failed
    }
    return 0;  // Success
}

