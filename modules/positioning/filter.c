/**
 * @file    filter.c
 * @brief   Distance and position filtering algorithms for UWB positioning
 *
 * This file implements various filtering algorithms used to smooth and improve
 * the accuracy of distance measurements and position calculations in the UWB
 * positioning system. Includes outlier rejection and Kalman filtering.
 *
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "filter.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief Array of tag distance filtering buffers for all tracked tags */
Tag_t tag_DistList[TAG_USE_MAX_NUM];

/*============================================================================
 * PRIVATE CONSTANTS
 *============================================================================*/

/** @brief Default Kalman filter process noise */
#define DEFAULT_PROCESS_NOISE_Q     0.01f

/** @brief Default Kalman filter measurement noise */
#define DEFAULT_MEASUREMENT_NOISE_R 0.1f

/** @brief Maximum reasonable distance for outlier detection (meters) */
#define MAX_REASONABLE_DISTANCE     100.0f

/** @brief Minimum distance change threshold for filtering */
#define MIN_DISTANCE_CHANGE         0.001f

/*============================================================================
 * DISTANCE FILTERING FUNCTIONS
 *============================================================================*/

/**
 * @brief Distance filter using average with outlier rejection
 *
 * Implements a 3-sample average filter that removes the highest and
 * lowest values to reduce the impact of measurement outliers. Maintains a
 * circular buffer of the last 3 measurements per tag.
 *
 * Algorithm:
 * 1. Store measurement in circular buffer
 * 2. Find min, max, and sum of all 3 samples
 * 3. Return average of remaining sample (total - max - min)
 *
 * @param index Tag ID (index into tag_DistList array)
 * @param dist_now Current distance measurement in meters
 * @return Filtered distance value (average after outlier removal)
 */
float Distance_Filter(int index, float dist_now)
{
    float result = 0;
    float max, min;
    char i;
    Tag_t *tag;

    // Validate input parameters
    if (index < 0 || index >= TAG_USE_MAX_NUM) {
        return dist_now;  // Invalid index, return raw measurement
    }

    // Basic range check for outlier detection
    if (dist_now < 0 || dist_now > MAX_REASONABLE_DISTANCE) {
        return tag_DistList[index].Dist[tag_DistList[index].Dist_index - 1];  // Return previous valid measurement
    }

    tag = &tag_DistList[index];

    // Update circular buffer index (0, 1, 2, 0, 1, 2...)
    if (tag->Dist_index >= 3) {
        tag->Dist_index = 0;
    }

    // Store current measurement in circular buffer
    tag->Dist[tag->Dist_index] = dist_now;
    tag->Dist_index = tag->Dist_index + 1;

    // Find max, min, and sum of all 3 samples
    max = tag->Dist[0];
    min = max;
    result = max;

    for (i = 1; i < 3; i++) {
        if (max < tag->Dist[i]) {
            max = tag->Dist[i];
        }
        if (min > tag->Dist[i]) {
            min = tag->Dist[i];
        }
        result += tag->Dist[i];
    }

    // Return average of remaining sample (total - max - min)
    // This effectively removes the outlier values
    return (result - max - min);
}

/**
 * @brief Enhanced distance filter with adaptive thresholding
 *
 * Advanced distance filter that adapts to measurement characteristics.
 * Uses statistical analysis to detect and reject outliers.
 *
 * @param index Tag ID (index into tag_DistList array)
 * @param dist_now Current distance measurement in meters
 * @param outlier_threshold Threshold for outlier detection (standard deviations)
 * @return Filtered distance value
 */
float Distance_Filter_Adaptive(int index, float dist_now, float outlier_threshold)
{
    Tag_t *tag;
    float mean, variance, std_dev;
    float samples[3];
    int i;

    // Validate input parameters
    if (index < 0 || index >= TAG_USE_MAX_NUM) {
        return dist_now;
    }

    tag = &tag_DistList[index];

    // Update circular buffer
    if (tag->Dist_index >= 3) {
        tag->Dist_index = 0;
    }
    tag->Dist[tag->Dist_index] = dist_now;
    tag->Dist_index++;

    // Copy samples for analysis
    for (i = 0; i < 3; i++) {
        samples[i] = tag->Dist[i];
    }

    // Calculate mean
    mean = (samples[0] + samples[1] + samples[2]) / 3.0f;

    // Calculate variance
    variance = 0;
    for (i = 0; i < 3; i++) {
        float diff = samples[i] - mean;
        variance += diff * diff;
    }
    variance /= 3.0f;
    std_dev = sqrtf(variance);

    // Check if current measurement is an outlier
    if (fabsf(dist_now - mean) > outlier_threshold * std_dev) {
        // Return mean of other two samples
        float sum = samples[0] + samples[1] + samples[2] - dist_now;
        return sum / 2.0f;
    }

    return mean;  // Return filtered value
}

/*============================================================================
 * POSITION FILTERING FUNCTIONS
 *============================================================================*/

/**
 * @brief 1D Kalman filter for position coordinates
 *
 * Implements a simple 1D Kalman filter for smoothing position coordinates.
 * Each tag maintains separate filter states for X, Y, and Z coordinates.
 * The filter helps reduce noise in position calculations.
 *
 * Kalman Filter Equations:
 * 1. Prediction: x_pred = x_prev, P_pred = P_prev + Q
 * 2. Update: K = P_pred / (P_pred + R)
 * 3. Estimate: x_est = x_pred + K * (measurement - x_pred)
 * 4. Covariance: P_est = (1 - K) * P_pred
 *
 * @param measurement Current coordinate measurement
 * @param process_noise_q Process noise variance (system uncertainty)
 * @param measurement_noise_r Measurement noise variance (sensor uncertainty)
 * @param tag_idx Tag ID (index into tag_DistList array)
 * @param coordinate_mode Filter mode: 0=X, 1=Y, 2=Z coordinate
 * @return Filtered coordinate value
 */
float Position_KalmanFilter(const float measurement, float process_noise_q,
                           float measurement_noise_r, char tag_idx, char coordinate_mode)
{
    const float R = measurement_noise_r;  // Measurement noise variance
    const float Q = process_noise_q;      // Process noise variance

    Tag_t *tag;
    float last_estimate;
    float filtered_value;
    float prediction_variance;
    float current_variance;
    float kalman_gain;

    // Validate input parameters
    if (tag_idx < 0 || tag_idx >= TAG_USE_MAX_NUM) {
        return measurement;  // Invalid tag index
    }

    if (coordinate_mode < 0 || coordinate_mode > 2) {
        return measurement;  // Invalid coordinate mode
    }

    tag = &tag_DistList[tag_idx];

    // Get previous state based on coordinate mode
    switch (coordinate_mode) {
        case 0: // X coordinate
            last_estimate = tag->last_x;
            prediction_variance = tag->p_last_x + Q;
            break;
        case 1: // Y coordinate
            last_estimate = tag->last_y;
            prediction_variance = tag->p_last_y + Q;
            break;
        case 2: // Z coordinate
            last_estimate = tag->last_z;
            prediction_variance = tag->p_last_z + Q;
            break;
        default:
            return measurement; // Should not reach here
    }

    // Kalman filter update equations
    kalman_gain = prediction_variance / (prediction_variance + R);
    filtered_value = last_estimate + kalman_gain * (measurement - last_estimate);
    current_variance = (1.0f - kalman_gain) * prediction_variance;

    // Save filter state based on coordinate mode
    switch (coordinate_mode) {
        case 0: // X coordinate
            tag->last_x = filtered_value;
            tag->p_last_x = current_variance;
            break;
        case 1: // Y coordinate
            tag->last_y = filtered_value;
            tag->p_last_y = current_variance;
            break;
        case 2: // Z coordinate
            tag->last_z = filtered_value;
            tag->p_last_z = current_variance;
            break;
    }

    return filtered_value;
}

/**
 * @brief Initialize Kalman filter for a specific tag
 *
 * Initializes the Kalman filter state for all coordinates of a tag.
 * Should be called when a tag is first detected or after a long period
 * of inactivity.
 *
 * @param tag_idx Tag ID to initialize
 * @param initial_x Initial X coordinate estimate
 * @param initial_y Initial Y coordinate estimate
 * @param initial_z Initial Z coordinate estimate
 * @param initial_variance Initial error covariance
 */
void Position_KalmanFilter_Init(char tag_idx, float initial_x, float initial_y,
                               float initial_z, float initial_variance)
{
    Tag_t *tag;

    if (tag_idx < 0 || tag_idx >= TAG_USE_MAX_NUM) {
        return;  // Invalid tag index
    }

    tag = &tag_DistList[tag_idx];

    // Initialize position estimates
    tag->last_x = initial_x;
    tag->last_y = initial_y;
    tag->last_z = initial_z;

    // Initialize error covariances
    tag->p_last_x = initial_variance;
    tag->p_last_y = initial_variance;
    tag->p_last_z = initial_variance;

    // Initialize distance filter
    tag->Dist_index = 0;
    for (int i = 0; i < 3; i++) {
        tag->Dist[i] = 0.0f;
    }
}

/**
 * @brief Reset filter state for a specific tag
 *
 * Resets all filter states for a tag. Useful when a tag has been
 * out of range for an extended period.
 *
 * @param tag_idx Tag ID to reset
 */
void Position_Filter_Reset(char tag_idx)
{
    Position_KalmanFilter_Init(tag_idx, 0.0f, 0.0f, 0.0f, 1.0f);
}

/**
 * @brief Apply comprehensive position filtering
 *
 * Applies multiple filtering stages to position coordinates including
 * outlier detection, range validation, and Kalman filtering.
 *
 * @param tag_idx Tag ID
 * @param raw_x Raw X coordinate
 * @param raw_y Raw Y coordinate
 * @param raw_z Raw Z coordinate
 * @param filtered_x Pointer to store filtered X coordinate
 * @param filtered_y Pointer to store filtered Y coordinate
 * @param filtered_z Pointer to store filtered Z coordinate
 * @return true if filtering successful, false if coordinates rejected
 */
bool Position_Filter_Comprehensive(char tag_idx, float raw_x, float raw_y, float raw_z,
                                  float *filtered_x, float *filtered_y, float *filtered_z)
{
    // Basic range validation
    const float MAX_COORDINATE = 1000.0f;  // Maximum reasonable coordinate value

    if (fabsf(raw_x) > MAX_COORDINATE || fabsf(raw_y) > MAX_COORDINATE || fabsf(raw_z) > MAX_COORDINATE) {
        return false;  // Coordinates out of reasonable range
    }

    // Apply Kalman filtering with default noise parameters
    *filtered_x = Position_KalmanFilter(raw_x, DEFAULT_PROCESS_NOISE_Q,
                                       DEFAULT_MEASUREMENT_NOISE_R, tag_idx, 0);
    *filtered_y = Position_KalmanFilter(raw_y, DEFAULT_PROCESS_NOISE_Q,
                                       DEFAULT_MEASUREMENT_NOISE_R, tag_idx, 1);
    *filtered_z = Position_KalmanFilter(raw_z, DEFAULT_PROCESS_NOISE_Q,
                                       DEFAULT_MEASUREMENT_NOISE_R, tag_idx, 2);

    return true;
}

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Get filter statistics for a specific tag
 *
 * @param tag_idx Tag ID
 * @param variance_x Pointer to store X coordinate variance
 * @param variance_y Pointer to store Y coordinate variance
 * @param variance_z Pointer to store Z coordinate variance
 */
void Position_Filter_GetStatistics(char tag_idx, float *variance_x,
                                  float *variance_y, float *variance_z)
{
    Tag_t *tag;

    if (tag_idx < 0 || tag_idx >= TAG_USE_MAX_NUM) {
        *variance_x = *variance_y = *variance_z = 0.0f;
        return;
    }

    tag = &tag_DistList[tag_idx];

    *variance_x = tag->p_last_x;
    *variance_y = tag->p_last_y;
    *variance_z = tag->p_last_z;
}
