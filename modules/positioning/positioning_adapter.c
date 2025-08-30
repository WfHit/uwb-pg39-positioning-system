/**
 * @file    positioning_adapter.c
 * @brief   Adapter layer for integrating positioning algorithms
 *
 * This file provides a bridge between the positioning algorithms and the
 * rest of the UWB project architecture. It provides a clean interface
 * for position calculations and algorithm selection.
 *
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "positioning_adapter.h"
#include "loc.h"
#include "ds_twr.h"
#include "hds_twr.h"
#include "filter.h"
#include "twr.h"
#include <string.h>
#include <math.h>

/*============================================================================
 * PRIVATE VARIABLES
 *============================================================================*/

/** @brief Current positioning algorithm configuration */
static positioning_algorithm_t current_algorithm = POSITIONING_ALGO_AUTO;

/** @brief Device ID for ranging operations */
static uint8_t device_id = 0;

/** @brief Statistics tracking */
static uint32_t total_calculations = 0;
static uint32_t successful_calculations = 0;

/** @brief Algorithm performance tracking */
typedef struct {
    uint32_t attempts;
    uint32_t successes;
    float avg_accuracy;
    uint32_t avg_time_ms;
} algorithm_stats_t;

static algorithm_stats_t algo_stats[6];  // One for each algorithm type

/*============================================================================
 * PRIVATE CONSTANTS
 *============================================================================*/

/** @brief Minimum accuracy threshold for valid results */
#define MIN_ACCURACY_THRESHOLD      0.1f    // 10 cm

/** @brief Maximum accuracy threshold for rejecting results */
#define MAX_ACCURACY_THRESHOLD      10.0f   // 10 m

/** @brief Minimum anchors for 2D positioning */
#define MIN_ANCHORS_2D              3

/** @brief Minimum anchors for 3D positioning */
#define MIN_ANCHORS_3D              4

/** @brief Maximum calculation time threshold (ms) */
#define MAX_CALCULATION_TIME_MS     100

/*============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 *============================================================================*/

static float calculate_position_accuracy(const positioning_anchor_data_t* anchor_data,
                                        uint8_t anchor_count,
                                        float x, float y, float z);
static bool validate_anchor_data(const positioning_anchor_data_t* anchor_data, uint8_t anchor_count);
static void update_algorithm_stats(positioning_algorithm_t algorithm, bool success, float accuracy, uint32_t time_ms);

/*============================================================================
 * INITIALIZATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize positioning algorithms
 *
 * Sets up the positioning system and initializes all sub-modules.
 * Must be called before using any positioning functions.
 *
 * @return true if successful, false otherwise
 */
bool positioning_adapter_init(void)
{
    // Initialize algorithm statistics
    memset(algo_stats, 0, sizeof(algo_stats));

    // Reset global statistics
    total_calculations = 0;
    successful_calculations = 0;

    // Initialize filter system for all possible tags
    for (int i = 0; i < TAG_USE_MAX_NUM; i++) {
        Position_Filter_Reset(i);
    }

    // Set default algorithm
    current_algorithm = POSITIONING_ALGO_AUTO;

    return true;
}

/**
 * @brief Configure positioning parameters
 *
 * @param algorithm Algorithm to use (or AUTO for automatic selection)
 * @return true if successful, false otherwise
 */
bool positioning_adapter_configure(positioning_algorithm_t algorithm)
{
    if (algorithm < POSITIONING_ALGO_2D_CENTER_MASS || algorithm > POSITIONING_ALGO_AUTO) {
        return false;
    }

    current_algorithm = algorithm;
    return true;
}

/*============================================================================
 * MAIN POSITIONING CALCULATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Calculate position using available anchor measurements
 *
 * Main entry point for position calculations. Automatically selects the best
 * algorithm based on available anchors and configuration.
 *
 * @param anchor_data Array of anchor measurements
 * @param anchor_count Number of valid anchor measurements
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate(const positioning_anchor_data_t* anchor_data,
                                 uint8_t anchor_count,
                                 positioning_result_t* result)
{
    uint32_t start_time = 0;  // Would be actual timer in real implementation
    positioning_algorithm_t selected_algorithm;
    bool success = false;

    // Initialize result structure
    memset(result, 0, sizeof(positioning_result_t));
    result->anchor_count = anchor_count;

    // Validate input data
    if (!validate_anchor_data(anchor_data, anchor_count)) {
        return false;
    }

    total_calculations++;

    // Select algorithm
    if (current_algorithm == POSITIONING_ALGO_AUTO) {
        bool has_3d_anchors = false;
        for (int i = 0; i < anchor_count; i++) {
            if (anchor_data[i].z != 0) {
                has_3d_anchors = true;
                break;
            }
        }
        selected_algorithm = positioning_adapter_select_algorithm(anchor_count, has_3d_anchors);
    } else {
        selected_algorithm = current_algorithm;
    }

    result->algorithm_used = selected_algorithm;

    // Perform calculation based on selected algorithm
    switch (selected_algorithm) {
        case POSITIONING_ALGO_2D_CENTER_MASS:
            success = positioning_adapter_calculate_2d_center_mass(anchor_data, result);
            break;

        case POSITIONING_ALGO_2D_LEAST_SQUARE:
            success = positioning_adapter_calculate_2d_least_square(anchor_data, anchor_count, result);
            break;

        case POSITIONING_ALGO_2D_TAYLOR:
            // Use centroid as initial estimate for Taylor method
            if (positioning_adapter_calculate_2d_center_mass(anchor_data, result)) {
                success = positioning_adapter_calculate_2d_taylor(anchor_data, anchor_count,
                                                                result->x, result->y, result);
            }
            break;

        case POSITIONING_ALGO_3D_LEAST_SQUARE:
            success = positioning_adapter_calculate_3d_least_square(anchor_data, anchor_count, result);
            break;

        case POSITIONING_ALGO_3D_TAYLOR:
            // Use 3D least squares as initial estimate for Taylor method
            if (positioning_adapter_calculate_3d_least_square(anchor_data, anchor_count, result)) {
                success = positioning_adapter_calculate_3d_taylor(anchor_data, anchor_count,
                                                                result->x, result->y, result->z, result);
            }
            break;

        default:
            success = false;
            break;
    }

    if (success) {
        // Calculate accuracy estimate
        result->accuracy_estimate = calculate_position_accuracy(anchor_data, anchor_count,
                                                              result->x, result->y, result->z);

        // Validate result
        result->is_valid = positioning_adapter_validate_result(result);

        if (result->is_valid) {
            successful_calculations++;
        }
    }

    // Update algorithm statistics
    uint32_t calc_time = 0;  // Would be actual elapsed time
    update_algorithm_stats(selected_algorithm, success && result->is_valid,
                          result->accuracy_estimate, calc_time);

    return success && result->is_valid;
}

/**
 * @brief Calculate 2D position using center mass method
 *
 * Simple algorithm that calculates position as the weighted centroid of
 * anchor positions, with weights inversely proportional to distance.
 *
 * @param anchor_data Array of anchor measurements (minimum 3)
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate_2d_center_mass(const positioning_anchor_data_t* anchor_data,
                                                positioning_result_t* result)
{
    float sum_x = 0, sum_y = 0;
    float sum_weights = 0;
    int valid_anchors = 0;

    // Calculate weighted centroid
    for (int i = 0; i < result->anchor_count; i++) {
        if (anchor_data[i].is_valid && anchor_data[i].distance > 0) {
            float weight = 1.0f / (anchor_data[i].distance + 0.1f);  // Avoid division by zero

            sum_x += anchor_data[i].x * weight;
            sum_y += anchor_data[i].y * weight;
            sum_weights += weight;
            valid_anchors++;
        }
    }

    if (valid_anchors < MIN_ANCHORS_2D || sum_weights == 0) {
        return false;
    }

    result->x = sum_x / sum_weights;
    result->y = sum_y / sum_weights;
    result->z = 0;

    return true;
}

/**
 * @brief Calculate 2D position using least squares method
 *
 * Uses the mathematical algorithms from loc.c to perform 2D positioning
 * with proper error handling and data conversion.
 *
 * @param anchor_data Array of anchor measurements (minimum 3)
 * @param anchor_count Number of anchor measurements
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate_2d_least_square(const positioning_anchor_data_t* anchor_data,
                                                  uint8_t anchor_count,
                                                  positioning_result_t* result)
{
    Anchor_t anc_list[ANCHOR_LIST_COUNT];
    float point_out[2];

    if (anchor_count < MIN_ANCHORS_2D) {
        return false;
    }

    // Convert anchor data format
    memset(anc_list, 0, sizeof(anc_list));
    for (int i = 0; i < anchor_count && i < ANCHOR_LIST_COUNT; i++) {
        if (anchor_data[i].is_valid) {
            anc_list[i].x = anchor_data[i].x;
            anc_list[i].y = anchor_data[i].y;
            anc_list[i].z = anchor_data[i].z;
            anc_list[i].dist = anchor_data[i].distance;
        }
    }

    // Perform calculation
    if (Rtls_Cal_2D(anc_list, point_out)) {
        result->x = point_out[0];
        result->y = point_out[1];
        result->z = 0;
        return true;
    }

    return false;
}

/**
 * @brief Calculate 3D position using least squares method
 *
 * Uses the mathematical algorithms from loc.c to perform 3D positioning
 * with proper error handling and data conversion.
 *
 * @param anchor_data Array of anchor measurements (minimum 4)
 * @param anchor_count Number of anchor measurements
 * @param result Pointer to store positioning result
 * @return true if calculation successful, false otherwise
 */
bool positioning_adapter_calculate_3d_least_square(const positioning_anchor_data_t* anchor_data,
                                                  uint8_t anchor_count,
                                                  positioning_result_t* result)
{
    Anchor_t anc_list[ANCHOR_LIST_COUNT];
    float point_out[3];

    if (anchor_count < MIN_ANCHORS_3D) {
        return false;
    }

    // Convert anchor data format
    memset(anc_list, 0, sizeof(anc_list));
    for (int i = 0; i < anchor_count && i < ANCHOR_LIST_COUNT; i++) {
        if (anchor_data[i].is_valid) {
            anc_list[i].x = anchor_data[i].x;
            anc_list[i].y = anchor_data[i].y;
            anc_list[i].z = anchor_data[i].z;
            anc_list[i].dist = anchor_data[i].distance;
        }
    }

    // Perform calculation
    if (Rtls_Cal_3D(anc_list, point_out)) {
        result->x = point_out[0];
        result->y = point_out[1];
        result->z = point_out[2];
        return true;
    }

    return false;
}

/**
 * @brief Calculate 2D position using Taylor series method
 *
 * Iterative algorithm that refines an initial position estimate using
 * Taylor series expansion. More accurate but requires good initial estimate.
 *
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
                                            positioning_result_t* result)
{
    // This is a placeholder for Taylor series implementation
    // The actual implementation would require iterative refinement
    // For now, fall back to least squares method
    return positioning_adapter_calculate_2d_least_square(anchor_data, anchor_count, result);
}

/**
 * @brief Calculate 3D position using Taylor series method
 *
 * Iterative algorithm that refines an initial 3D position estimate using
 * Taylor series expansion.
 *
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
                                            positioning_result_t* result)
{
    // This is a placeholder for Taylor series implementation
    // The actual implementation would require iterative refinement
    // For now, fall back to least squares method
    return positioning_adapter_calculate_3d_least_square(anchor_data, anchor_count, result);
}

/*============================================================================
 * TWR ALGORITHM FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize DS-TWR (Double-Sided Two-Way Ranging)
 * @return true if successful, false otherwise
 */
bool positioning_adapter_ds_twr_init(void)
{
    // Initialize DS-TWR specific variables
    SYS_Calculate_ACTIVE_FLAG = 0;
    return true;
}

/**
 * @brief Initialize HDS-TWR (High-Density Symmetric Two-Way Ranging)
 * @return true if successful, false otherwise
 */
bool positioning_adapter_hds_twr_init(void)
{
    // Initialize HDS-TWR specific variables
    // Implementation depends on HDS-TWR system design
    return true;
}

/**
 * @brief Perform DS-TWR ranging measurement
 *
 * @param anchor_id Target anchor ID
 * @param distance_mm Pointer to store measured distance in millimeters
 * @return true if measurement successful, false otherwise
 */
bool positioning_adapter_ds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm)
{
    int8_t ret = 0;
    int32_t result;

    result = DW1000send(device_id, anchor_id, 0, &ret);

    if (ret == 1) {  // Success
        *distance_mm = (uint32_t)result * 10;  // Convert cm to mm
        return true;
    }

    return false;
}

/**
 * @brief Perform HDS-TWR ranging measurement
 *
 * @param anchor_id Target anchor ID
 * @param distance_mm Pointer to store measured distance in millimeters
 * @return true if measurement successful, false otherwise
 */
bool positioning_adapter_hds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm)
{
    // Placeholder for HDS-TWR implementation
    // Would call appropriate HDS-TWR functions
    return false;
}

/*============================================================================
 * FILTERING AND VALIDATION FUNCTIONS
 *============================================================================*/

/**
 * @brief Apply distance filtering to measurement
 *
 * @param tag_id Tag identifier
 * @param distance_mm Raw distance measurement in millimeters
 * @return Filtered distance in millimeters
 */
uint32_t positioning_adapter_filter_distance(uint8_t tag_id, uint32_t distance_mm)
{
    float distance_m = distance_mm / 1000.0f;
    float filtered_m = Distance_Filter(tag_id, distance_m);
    return (uint32_t)(filtered_m * 1000);
}

/**
 * @brief Validate positioning result
 *
 * @param result Positioning result to validate
 * @return true if result is valid, false otherwise
 */
bool positioning_adapter_validate_result(const positioning_result_t* result)
{
    if (!result) {
        return false;
    }

    // Check for NaN or infinite values
    if (isnan(result->x) || isnan(result->y) || isnan(result->z) ||
        isinf(result->x) || isinf(result->y) || isinf(result->z)) {
        return false;
    }

    // Check accuracy estimate
    if (result->accuracy_estimate < MIN_ACCURACY_THRESHOLD ||
        result->accuracy_estimate > MAX_ACCURACY_THRESHOLD) {
        return false;
    }

    // Check reasonable coordinate ranges (adjust based on your system)
    const float MAX_COORD = 1000.0f;  // 1000 meters
    if (fabsf(result->x) > MAX_COORD || fabsf(result->y) > MAX_COORD || fabsf(result->z) > MAX_COORD) {
        return false;
    }

    return true;
}

/*============================================================================
 * STATISTICS AND UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Get positioning statistics
 *
 * @param total_calculations_out Pointer to store total calculation count
 * @param successful_calculations_out Pointer to store successful calculation count
 * @param success_rate Pointer to store success rate (0.0 to 1.0)
 */
void positioning_adapter_get_statistics(uint32_t* total_calculations_out,
                                       uint32_t* successful_calculations_out,
                                       float* success_rate)
{
    *total_calculations_out = total_calculations;
    *successful_calculations_out = successful_calculations;
    *success_rate = (total_calculations > 0) ?
                   ((float)successful_calculations / total_calculations) : 0.0f;
}

/**
 * @brief Reset positioning statistics
 */
void positioning_adapter_reset_statistics(void)
{
    total_calculations = 0;
    successful_calculations = 0;
    memset(algo_stats, 0, sizeof(algo_stats));
}

/**
 * @brief Select best algorithm based on available anchors
 *
 * @param anchor_count Number of available anchors
 * @param has_3d_anchors Whether 3D anchor positions are available
 * @return Recommended positioning algorithm
 */
positioning_algorithm_t positioning_adapter_select_algorithm(uint8_t anchor_count, bool has_3d_anchors)
{
    if (has_3d_anchors && anchor_count >= MIN_ANCHORS_3D) {
        return POSITIONING_ALGO_3D_LEAST_SQUARE;
    } else if (anchor_count >= MIN_ANCHORS_2D) {
        return POSITIONING_ALGO_2D_LEAST_SQUARE;
    } else {
        return POSITIONING_ALGO_2D_CENTER_MASS;  // Fallback for limited anchors
    }
}

/**
 * @brief Set device ID for ranging operations
 *
 * @param new_device_id Device identifier for TWR operations
 */
void positioning_adapter_set_device_id(uint8_t new_device_id)
{
    device_id = new_device_id;
}

/*============================================================================
 * UTILITY CONVERSION FUNCTIONS
 *============================================================================*/

/**
 * @brief Convert discovered anchor to positioning anchor data
 *
 * @param discovered_anchor Source discovered anchor
 * @param distance_mm Measured distance in millimeters
 * @param positioning_anchor Destination positioning anchor data
 */
void positioning_adapter_convert_anchor_data(const discovered_anchor_t* discovered_anchor,
                                            uint32_t distance_mm,
                                            positioning_anchor_data_t* positioning_anchor)
{
    positioning_anchor->anchor_id = discovered_anchor->anchor_id;
    positioning_anchor->x = discovered_anchor->position.x_cm / 100.0f;  // Convert cm to m
    positioning_anchor->y = discovered_anchor->position.y_cm / 100.0f;
    positioning_anchor->z = discovered_anchor->position.z_cm / 100.0f;
    positioning_anchor->distance = distance_mm / 1000.0f;  // Convert mm to m
    positioning_anchor->is_valid = discovered_anchor->is_active && (distance_mm > 0);
}

/*============================================================================
 * PRIVATE HELPER FUNCTIONS
 *============================================================================*/

/**
 * @brief Calculate estimated accuracy of position result
 */
static float calculate_position_accuracy(const positioning_anchor_data_t* anchor_data,
                                        uint8_t anchor_count,
                                        float x, float y, float z)
{
    float total_error = 0;
    int valid_measurements = 0;

    for (int i = 0; i < anchor_count; i++) {
        if (anchor_data[i].is_valid) {
            float dx = x - anchor_data[i].x;
            float dy = y - anchor_data[i].y;
            float dz = z - anchor_data[i].z;
            float calculated_distance = sqrtf(dx*dx + dy*dy + dz*dz);

            float error = fabsf(calculated_distance - anchor_data[i].distance);
            total_error += error;
            valid_measurements++;
        }
    }

    return (valid_measurements > 0) ? (total_error / valid_measurements) : INFINITY;
}

/**
 * @brief Validate anchor data array
 */
static bool validate_anchor_data(const positioning_anchor_data_t* anchor_data, uint8_t anchor_count)
{
    if (!anchor_data || anchor_count == 0) {
        return false;
    }

    int valid_anchors = 0;
    for (int i = 0; i < anchor_count; i++) {
        if (anchor_data[i].is_valid && anchor_data[i].distance > 0) {
            valid_anchors++;
        }
    }

    return valid_anchors >= MIN_ANCHORS_2D;
}

/**
 * @brief Update algorithm performance statistics
 */
static void update_algorithm_stats(positioning_algorithm_t algorithm, bool success,
                                  float accuracy, uint32_t time_ms)
{
    if (algorithm >= 0 && algorithm < 6) {
        algo_stats[algorithm].attempts++;

        if (success) {
            algo_stats[algorithm].successes++;

            // Update running average of accuracy
            float alpha = 0.1f;  // Smoothing factor
            algo_stats[algorithm].avg_accuracy =
                alpha * accuracy + (1.0f - alpha) * algo_stats[algorithm].avg_accuracy;

            // Update running average of calculation time
            algo_stats[algorithm].avg_time_ms =
                (uint32_t)(alpha * time_ms + (1.0f - alpha) * algo_stats[algorithm].avg_time_ms);
        }
    }
}

/*============================================================================
 * RANGING COMPATIBILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Send frame via UWB (compatibility wrapper)
 *
 * @param data Frame data to send
 * @param length Frame length in bytes
 * @return true if successful, false otherwise
 */
bool ranging_send_frame(const uint8_t* data, uint16_t length)
{
    // This would interface with the actual UWB driver
    // Placeholder implementation
    return false;
}

/**
 * @brief Receive frame via UWB (compatibility wrapper)
 *
 * @param buffer Buffer to store received data
 * @param buffer_size Buffer size in bytes
 * @param rssi Pointer to store RSSI value
 * @param timeout_ms Timeout in milliseconds
 * @return true if frame received, false on timeout
 */
bool ranging_receive_frame(uint8_t* buffer, uint16_t buffer_size, int8_t* rssi, uint32_t timeout_ms)
{
    // This would interface with the actual UWB driver
    // Placeholder implementation
    return false;
}

/**
 * @brief Measure distance to anchor (compatibility wrapper)
 *
 * @param anchor_id Target anchor ID
 * @param distance_mm Pointer to store measured distance in millimeters
 * @param algorithm_used Pointer to store algorithm used
 * @return true if measurement successful, false otherwise
 */
bool ranging_measure_distance(uint8_t anchor_id, uint32_t* distance_mm, uint8_t* algorithm_used)
{
    *algorithm_used = 0;  // DS-TWR
    return positioning_adapter_ds_twr_measure(anchor_id, distance_mm);
}
