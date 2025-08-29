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

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief Array of tag distance filtering buffers for all tracked tags */
Tag_t tag_DistList[TAG_USE_MAX_NUM];

/*============================================================================
 * DISTANCE FILTERING FUNCTIONS
 *============================================================================*/

/**
 * @brief Average filter with outlier rejection
 * 
 * Implements a simple 3-sample average filter that removes the highest and
 * lowest values to reduce the impact of measurement outliers. Maintains a
 * circular buffer of the last 3 measurements per tag.
 * 
 * @param index Tag ID (index into tag_DistList array)
 * @param dist_now Current distance measurement in meters
 * @return Filtered distance value (average of remaining sample after outlier removal)
 */
float Distance_Filter(int index, float dist_now)
{
	float result = 0;
	float max, min;
	char i;
	Tag_t *tag = &tag_DistList[index];
	
	// Update circular buffer index (0, 1, 2, 0, 1, 2...)
	if(tag->Dist_index >= 3)
		tag->Dist_index = 0;

	// Store current measurement in circular buffer
	tag->Dist[tag->Dist_index] = dist_now;
	tag->Dist_index = tag->Dist_index + 1;
	
	// Find max, min, and sum of all 3 samples
	max = tag->Dist[0];
	min = max;
	result = max;
	
	for(i = 1; i < 3; i++) {
		if(max < tag->Dist[i])
			max = tag->Dist[i];
		if(min > tag->Dist[i])
			min = tag->Dist[i];
		result += tag->Dist[i];
	}
	
	// Return average of remaining sample (total - max - min)
	return (result - max - min);
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
 * @param measurement Current measurement value (coordinate)
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
     
	Tag_t *tag = &tag_DistList[tag_idx];
	
    float last_estimate;
    float filtered_value;
    float prediction_variance;
    float current_variance;
    float kalman_gain;

	// Get previous state based on coordinate mode
	switch(coordinate_mode) {
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
			return measurement; // Invalid mode, return raw measurement
	}

	// Kalman filter update equations
	kalman_gain = prediction_variance / (prediction_variance + R);
	filtered_value = last_estimate + kalman_gain * (measurement - last_estimate);
	current_variance = (1 - kalman_gain) * prediction_variance;
		
	// Save filter state based on coordinate mode
	switch(coordinate_mode) {
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


