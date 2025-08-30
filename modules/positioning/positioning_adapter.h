/**
 * @file    positioning_adapter.h
 * @brief   Adapter layer for integrating original TWR positioning algorithms
 *
 * This file provides a bridge between the original positioning algorithms
 * and the new UWB project architecture.
 *
 * @author  UWB PG3.9 project
 * @date    2024
 */

#ifndef POSITIONING_ADAPTER_H
#define POSITIONING_ADAPTER_H

#include "data_types.h"
#include "system_config.h"
#include "loc.h"
#include "ds_twr.h"
#include "hds_twr.h"
#include "filter.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * POSITIONING ALGORITHM TYPES
 *============================================================================*/

/**
 * @brief Positioning algorithm selection
 */
typedef enum {
    POSITIONING_ALGO_2D_CENTER_MASS = 0,    /* 2D center mass calculation */
    POSITIONING_ALGO_2D_LEAST_SQUARE,       /* 2D least squares method */
    POSITIONING_ALGO_2D_TAYLOR,             /* 2D Taylor series method */
    POSITIONING_ALGO_3D_LEAST_SQUARE,       /* 3D least squares method */
    POSITIONING_ALGO_3D_TAYLOR,             /* 3D Taylor series method */
    POSITIONING_ALGO_AUTO                    /* Automatic selection */
} positioning_algorithm_t;

/**
 * @brief Positioning result structure
 */
typedef struct {
    float x;                    /* X coordinate in meters */
    float y;                    /* Y coordinate in meters */
    float z;                    /* Z coordinate in meters */
    float accuracy_estimate;    /* Estimated accuracy in meters */
    positioning_algorithm_t algorithm_used;  /* Algorithm that was used */
    uint8_t anchor_count;       /* Number of anchors used */
    bool is_valid;              /* Whether result is valid */
} positioning_result_t;

/**
 * @brief Anchor data structure for positioning algorithms
 */
typedef struct {
    float x, y, z;              /* Anchor position in meters */
    float distance;             /* Measured distance in meters */
    uint8_t anchor_id;          /* Anchor identifier */
    bool is_valid;              /* Whether measurement is valid */
} positioning_anchor_data_t;

/*============================================================================
 * INITIALIZATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize positioning algorithms
 * @return true if successful, false otherwise
 */
bool positioning_adapter_init(void);

/**
 * @brief Configure positioning parameters
 * @param algorithm Algorithm to use (or AUTO for automatic selection)
 * @return true if successful, false otherwise
 */
bool positioning_adapter_configure(positioning_algorithm_t algorithm);

/*============================================================================
 * POSITIONING CALCULATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Calculate position using available anchor measurements
 * @param anchor_data Array of anchor measurements
 * @param anchor_count Number of valid anchor measurements
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate(const positioning_anchor_data_t* anchor_data,
                                 uint8_t anchor_count,
                                 positioning_result_t* result);

/**
 * @brief Calculate 2D position using center mass method
 * @param anchor_data Array of anchor measurements (minimum 3)
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate_2d_center_mass(const positioning_anchor_data_t* anchor_data,
                                                positioning_result_t* result);

/**
 * @brief Calculate 2D position using least squares method
 * @param anchor_data Array of anchor measurements (minimum 3)
 * @param anchor_count Number of anchor measurements
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate_2d_least_square(const positioning_anchor_data_t* anchor_data,
                                                  uint8_t anchor_count,
                                                  positioning_result_t* result);

/**
 * @brief Calculate 3D position using least squares method
 * @param anchor_data Array of anchor measurements (minimum 4)
 * @param anchor_count Number of anchor measurements
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate_3d_least_square(const positioning_anchor_data_t* anchor_data,
                                                  uint8_t anchor_count,
                                                  positioning_result_t* result);

/**
 * @brief Calculate 2D position using Taylor series method
 * @param anchor_data Array of anchor measurements
 * @param anchor_count Number of anchor measurements
 * @param initial_x Initial X estimate
 * @param initial_y Initial Y estimate
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate_2d_taylor(const positioning_anchor_data_t* anchor_data,
                                            uint8_t anchor_count,
                                            float initial_x, float initial_y,
                                            positioning_result_t* result);

/**
 * @brief Calculate 3D position using Taylor series method
 * @param anchor_data Array of anchor measurements
 * @param anchor_count Number of anchor measurements
 * @param initial_x Initial X estimate
 * @param initial_y Initial Y estimate
 * @param initial_z Initial Z estimate
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate_3d_taylor(const positioning_anchor_data_t* anchor_data,
                                            uint8_t anchor_count,
                                            float initial_x, float initial_y, float initial_z,
                                            positioning_result_t* result);

/*============================================================================
 * TWR ALGORITHM FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize DS-TWR (Double-Sided Two-Way Ranging)
 * @return true if successful, false otherwise
 */
bool positioning_adapter_ds_twr_init(void);

/**
 * @brief Initialize HDS-TWR (High-Density Symmetric Two-Way Ranging)
 * @return true if successful, false otherwise
 */
bool positioning_adapter_hds_twr_init(void);

/**
 * @brief Perform DS-TWR ranging measurement
 * @param anchor_id Target anchor ID
 * @param distance_mm Pointer to store measured distance in millimeters
 * @return true if measurement successful, false otherwise
 */
bool positioning_adapter_ds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm);

/**
 * @brief Perform HDS-TWR ranging measurement
 * @param anchor_id Target anchor ID
 * @param distance_mm Pointer to store measured distance in millimeters
 * @return true if measurement successful, false otherwise
 */
bool positioning_adapter_hds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm);

/*============================================================================
 * FILTERING AND VALIDATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Apply distance filtering to measurement
 * @param tag_id Tag identifier
 * @param distance_mm Raw distance measurement in millimeters
 * @return Filtered distance in millimeters
 */
uint32_t positioning_adapter_filter_distance(uint8_t tag_id, uint32_t distance_mm);

/**
 * @brief Validate positioning result
 * @param result Positioning result to validate
 * @return true if result is valid, false otherwise
 */
bool positioning_adapter_validate_result(const positioning_result_t* result);

/**
 * @brief Get positioning statistics
 * @param total_calculations Pointer to store total calculation count
 * @param successful_calculations Pointer to store successful calculation count
 * @param success_rate Pointer to store success rate (0.0 to 1.0)
 */
void positioning_adapter_get_statistics(uint32_t* total_calculations_out,
                                       uint32_t* successful_calculations_out,
                                       float* success_rate);

/**
 * @brief Reset positioning statistics
 */
void positioning_adapter_reset_statistics(void);

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Convert discovered anchor to positioning anchor data
 * @param discovered_anchor Source discovered anchor
 * @param distance_mm Measured distance in millimeters
 * @param positioning_anchor Destination positioning anchor data
 */
void positioning_adapter_convert_anchor_data(const discovered_anchor_t* discovered_anchor,
                                            uint32_t distance_mm,
                                            positioning_anchor_data_t* positioning_anchor);

/**
 * @brief Select best algorithm based on available anchors
 * @param anchor_count Number of available anchors
 * @param has_3d_anchors Whether 3D anchor positions are available
 * @return Recommended positioning algorithm
 */
positioning_algorithm_t positioning_adapter_select_algorithm(uint8_t anchor_count, bool has_3d_anchors);

/**
 * @brief Set device ID for ranging operations
 * @param device_id Device identifier for TWR operations
 */
void positioning_adapter_set_device_id(uint8_t device_id);

/*============================================================================
 * RANGING COMPATIBILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Send frame via UWB (compatibility wrapper)
 * @param data Frame data to send
 * @param length Frame length in bytes
 * @return true if successful, false otherwise
 */
bool ranging_send_frame(const uint8_t* data, uint16_t length);

/**
 * @brief Receive frame via UWB (compatibility wrapper)
 * @param buffer Buffer to store received data
 * @param buffer_size Buffer size in bytes
 * @param rssi Pointer to store RSSI value
 * @param timeout_ms Timeout in milliseconds
 * @return true if frame received, false on timeout
 */
bool ranging_receive_frame(uint8_t* buffer, uint16_t buffer_size, int8_t* rssi, uint32_t timeout_ms);

/**
 * @brief Measure distance to anchor (compatibility wrapper)
 * @param anchor_id Target anchor ID
 * @param distance_mm Pointer to store measured distance in millimeters
 * @param algorithm_used Pointer to store algorithm used
 * @return true if measurement successful, false otherwise
 */
bool ranging_measure_distance(uint8_t anchor_id, uint32_t* distance_mm, uint8_t* algorithm_used);

#ifdef __cplusplus
}
#endif

#endif /* POSITIONING_ADAPTER_H */
