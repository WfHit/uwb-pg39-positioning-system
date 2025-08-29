/**
 * @file    filter.h  
 * @brief   Distance and position filtering algorithms header
 * 
 * This header defines structures and functions for filtering distance measurements
 * and position coordinates in the UWB positioning system. Provides outlier rejection
 * and Kalman filtering capabilities to improve measurement accuracy.
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include "data_types.h"

/*============================================================================
 * TYPE DEFINITIONS
 *============================================================================*/

/**
 * @brief Filter state structure for each tracked tag
 * 
 * This structure maintains the filtering state for a single tag, including
 * distance measurement buffers and Kalman filter states for position coordinates.
 */
typedef struct 
{
	float Dist[3];          ///< Circular buffer for last 3 distance measurements
	char Dist_index;        ///< Current index in distance circular buffer (0-2)
	
	// Kalman filter state for X coordinate
	float p_last_x;         ///< Previous error covariance for X position
	float last_x;           ///< Previous filtered X position estimate
	
	// Kalman filter state for Y coordinate  
	float p_last_y;         ///< Previous error covariance for Y position
	float last_y;           ///< Previous filtered Y position estimate
	
	// Kalman filter state for Z coordinate
	float p_last_z;         ///< Previous error covariance for Z position 
	float last_z;           ///< Previous filtered Z position estimate
	
} __attribute((packed)) Tag_t;

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief Array of filter states for all trackable tags */
extern Tag_t tag_DistList[TAG_USE_MAX_NUM];

/*============================================================================
 * FUNCTION DECLARATIONS
 *============================================================================*/

/**
 * @brief Distance filter using average with outlier rejection
 * 
 * @param index Tag ID (0 to TAG_USE_MAX_NUM-1)
 * @param dist_now Current distance measurement in meters
 * @return Filtered distance value in meters
 */
float Distance_Filter(int index, float dist_now);

/**
 * @brief 1D Kalman filter for position coordinates
 * 
 * @param measurement Current coordinate measurement
 * @param process_noise_q Process noise variance (system uncertainty)
 * @param measurement_noise_r Measurement noise variance (sensor uncertainty)
 * @param tag_idx Tag ID (0 to TAG_USE_MAX_NUM-1)
 * @param coordinate_mode Coordinate to filter: 0=X, 1=Y, 2=Z
 * @return Filtered coordinate value
 */
float Position_KalmanFilter(const float measurement, float process_noise_q,
                           float measurement_noise_r, char tag_idx, char coordinate_mode);

/*============================================================================
 * LEGACY FUNCTION ALIASES (for backward compatibility)
 *============================================================================*/

/** @brief Legacy alias for Distance_Filter */
#define Average_ex(index, dist_now) Distance_Filter(index, dist_now)

/** @brief Legacy alias for Position_KalmanFilter */
#define KalmanFilter(measurement, process_q, measure_r, idx, mode) \
    Position_KalmanFilter(measurement, process_q, measure_r, idx, mode)

#endif // __FILTER_H
