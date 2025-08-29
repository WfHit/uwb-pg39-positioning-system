/**
 * @file    positioning_adapter.c
 * @brief   Implementation of positioning algorithm adapter layer
 * 
 * This file bridges the original TWR positioning algorithms with the new
 * UWB project architecture, providing a clean interface for position calculations.
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "positioning_adapter.h"
#include "ds_twr.h"
#include "hds_twr.h"
#include <arm_math.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include "../utils/timing_utils.h"
#include <string.h>
#include <math.h>

// Include the original TWR implementations
extern int32_t DW1000send(uint8_t A_ID, uint8_t B_ID, uint8_t MODE, int8_t *ret);
extern uint8_t HDS_TWR_Send_Resp(uint8_t A_ID, uint8_t B_ID, uint8_t En);
extern uint8_t HDS_TWR_Recv_FinalAndCal(uint8_t A_ID, uint8_t B_ID, uint16_t recv_timeout);
extern uint16_t Dis_cal; // Distance result from HDS-TWR

// Define modes for TWR algorithms
#define DS_TWR_MODE     1
#define HDS_TWR_MODE    2

// Additional DW3000 status constants for compatibility
#define SYS_STATUS_ALL_RX_TO    (SYS_STATUS_RXRFTO_BIT_MASK | SYS_STATUS_RXPTO_BIT_MASK)
#define SYS_STATUS_ALL_RX_ERR   (SYS_STATUS_RXPHE_BIT_MASK | SYS_STATUS_RXRFSL_BIT_MASK)

// Current device ID for ranging operations
static uint8_t current_device_id = 0;

/*============================================================================
 * PRIVATE VARIABLES
 *============================================================================*/

static positioning_algorithm_t current_algorithm = POSITIONING_ALGO_AUTO;
static uint32_t total_calculations = 0;
static uint32_t successful_calculations = 0;
static bool is_initialized = false;

/*============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 *============================================================================*/

static bool validate_anchor_data(const positioning_anchor_data_t* anchor_data, uint8_t count);
static float estimate_accuracy(const positioning_anchor_data_t* anchor_data, uint8_t count, 
                              const positioning_result_t* result);

/*============================================================================
 * INITIALIZATION FUNCTIONS
 *============================================================================*/

bool positioning_adapter_init(void)
{
    // Initialize ARM math library (already done by CMSIS)
    // Reset statistics
    positioning_adapter_reset_statistics();
    
    // Initialize DS-TWR and HDS-TWR if needed
    positioning_adapter_ds_twr_init();
    positioning_adapter_hds_twr_init();
    
    is_initialized = true;
    return true;
}

bool positioning_adapter_configure(positioning_algorithm_t algorithm)
{
    if (!is_initialized) {
        return false;
    }
    
    current_algorithm = algorithm;
    return true;
}

/*============================================================================
 * POSITIONING CALCULATION FUNCTIONS
 *============================================================================*/

bool positioning_adapter_calculate(const positioning_anchor_data_t* anchor_data,
                                 uint8_t anchor_count,
                                 positioning_result_t* result)
{
    if (!is_initialized || !anchor_data || !result || anchor_count < 3) {
        return false;
    }
    
    // Validate input data
    if (!validate_anchor_data(anchor_data, anchor_count)) {
        return false;
    }
    
    total_calculations++;
    
    // Clear result structure
    memset(result, 0, sizeof(positioning_result_t));
    result->anchor_count = anchor_count;
    
    positioning_algorithm_t algo_to_use = current_algorithm;
    
    // Auto-select algorithm if needed
    if (algo_to_use == POSITIONING_ALGO_AUTO) {
        bool has_3d = false;
        for (uint8_t i = 0; i < anchor_count; i++) {
            if (fabsf(anchor_data[i].z) > 0.1f) {  // Non-zero Z coordinate
                has_3d = true;
                break;
            }
        }
        algo_to_use = positioning_adapter_select_algorithm(anchor_count, has_3d);
    }
    
    bool success = false;
    
    // Execute selected algorithm
    switch (algo_to_use) {
        case POSITIONING_ALGO_2D_CENTER_MASS:
            if (anchor_count >= 3) {
                success = positioning_adapter_calculate_2d_center_mass(anchor_data, result);
            }
            break;
            
        case POSITIONING_ALGO_2D_LEAST_SQUARE:
            if (anchor_count >= 3) {
                success = positioning_adapter_calculate_2d_least_square(anchor_data, anchor_count, result);
            }
            break;
            
        case POSITIONING_ALGO_3D_LEAST_SQUARE:
            if (anchor_count >= 4) {
                success = positioning_adapter_calculate_3d_least_square(anchor_data, anchor_count, result);
            }
            break;
            
        case POSITIONING_ALGO_2D_TAYLOR:
            if (anchor_count >= 3) {
                // Use center mass as initial estimate for Taylor method
                positioning_result_t initial_result;
                if (positioning_adapter_calculate_2d_center_mass(anchor_data, &initial_result)) {
                    success = positioning_adapter_calculate_2d_taylor(anchor_data, anchor_count,
                                                                    initial_result.x, initial_result.y, result);
                }
            }
            break;
            
        case POSITIONING_ALGO_3D_TAYLOR:
            if (anchor_count >= 4) {
                // Use least squares as initial estimate for Taylor method
                positioning_result_t initial_result;
                if (positioning_adapter_calculate_3d_least_square(anchor_data, anchor_count, &initial_result)) {
                    success = positioning_adapter_calculate_3d_taylor(anchor_data, anchor_count,
                                                                    initial_result.x, initial_result.y, initial_result.z, result);
                }
            }
            break;
            
        default:
            success = false;
            break;
    }
    
    if (success) {
        result->algorithm_used = algo_to_use;
        result->is_valid = positioning_adapter_validate_result(result);
        result->accuracy_estimate = estimate_accuracy(anchor_data, anchor_count, result);
        successful_calculations++;
    }
    
    return success;
}

bool positioning_adapter_calculate_2d_center_mass(const positioning_anchor_data_t* anchor_data,
                                                positioning_result_t* result)
{
    if (!anchor_data || !result) {
        return false;
    }
    
    // Convert to format expected by original algorithm
    float anc_a[3] = {anchor_data[0].x, anchor_data[0].y, anchor_data[0].distance};
    float anc_b[3] = {anchor_data[1].x, anchor_data[1].y, anchor_data[1].distance};
    float anc_c[3] = {anchor_data[2].x, anchor_data[2].y, anchor_data[2].distance};
    float cal_result[2] = {0};
    
    // Call original algorithm
    uint8_t success = Cal_2D_AllCenterMass(anc_a, anc_b, anc_c, cal_result);
    
    if (success) {
        result->x = cal_result[0];
        result->y = cal_result[1];
        result->z = 0.0f;  // 2D calculation
        return true;
    }
    
    return false;
}

bool positioning_adapter_calculate_2d_least_square(const positioning_anchor_data_t* anchor_data,
                                                  uint8_t anchor_count,
                                                  positioning_result_t* result)
{
    // This would require implementing a 2D least squares version
    // For now, fall back to center mass for 3 anchors
    if (anchor_count == 3) {
        return positioning_adapter_calculate_2d_center_mass(anchor_data, result);
    }
    
    // For more than 3 anchors, we'd need to implement the 2D least squares algorithm
    // This is a placeholder for future implementation
    return false;
}

bool positioning_adapter_calculate_3d_least_square(const positioning_anchor_data_t* anchor_data,
                                                  uint8_t anchor_count,
                                                  positioning_result_t* result)
{
    if (!anchor_data || !result || anchor_count < 4) {
        return false;
    }
    
    // Convert to format expected by original algorithm
    float ancs[anchor_count][4];
    for (uint8_t i = 0; i < anchor_count; i++) {
        ancs[i][0] = anchor_data[i].x;
        ancs[i][1] = anchor_data[i].y;
        ancs[i][2] = anchor_data[i].z;
        ancs[i][3] = anchor_data[i].distance;
    }
    
    float cal_result[3] = {0};
    
    // Call original algorithm
    uint8_t success = Cal_3D_LeastSquare((const float(*)[4])ancs, anchor_count, cal_result);
    
    if (success) {
        result->x = cal_result[0];
        result->y = cal_result[1];
        result->z = cal_result[2];
        return true;
    }
    
    return false;
}

bool positioning_adapter_calculate_2d_taylor(const positioning_anchor_data_t* anchor_data,
                                            uint8_t anchor_count,
                                            float initial_x, float initial_y,
                                            positioning_result_t* result)
{
    if (!anchor_data || !result || anchor_count < 3) {
        return false;
    }
    
    // Convert to format expected by original algorithm
    float ancs[anchor_count][3];
    for (uint8_t i = 0; i < anchor_count; i++) {
        ancs[i][0] = anchor_data[i].x;
        ancs[i][1] = anchor_data[i].y;
        ancs[i][2] = anchor_data[i].distance;
    }
    
    float cal_result[2] = {0};
    
    // Call original algorithm
    int8_t success = Cal_Taylor_2D(ancs, anchor_count, initial_x, initial_y, cal_result);
    
    if (success > 0) {
        result->x = cal_result[0];
        result->y = cal_result[1];
        result->z = 0.0f;  // 2D calculation
        return true;
    }
    
    return false;
}

bool positioning_adapter_calculate_3d_taylor(const positioning_anchor_data_t* anchor_data,
                                            uint8_t anchor_count,
                                            float initial_x, float initial_y, float initial_z,
                                            positioning_result_t* result)
{
    if (!anchor_data || !result || anchor_count < 4) {
        return false;
    }
    
    // Convert to format expected by original algorithm
    float ancs[anchor_count][4];
    for (uint8_t i = 0; i < anchor_count; i++) {
        ancs[i][0] = anchor_data[i].x;
        ancs[i][1] = anchor_data[i].y;
        ancs[i][2] = anchor_data[i].z;
        ancs[i][3] = anchor_data[i].distance;
    }
    
    float cal_result[3] = {0};
    
    // Call original algorithm
    int8_t success = Cal_Taylor_3D((const float(*)[4])ancs, anchor_count, 
                                  initial_x, initial_y, initial_z, cal_result);
    
    if (success > 0) {
        result->x = cal_result[0];
        result->y = cal_result[1];
        result->z = cal_result[2];
        return true;
    }
    
    return false;
}

/*============================================================================
 * TWR ALGORITHM FUNCTIONS
 *============================================================================*/

bool positioning_adapter_ds_twr_init(void)
{
    // Initialize DW3000 hardware using DecaWave API
    if (dwt_initialise(0) != DWT_SUCCESS) {
        return false;
    }
    
    // Configure DW3000 for DS-TWR operation
    // Note: Configuration parameters would need to be set properly for production
    
    return true;
}

bool positioning_adapter_hds_twr_init(void)
{
    // Initialize DW3000 hardware using DecaWave API
    if (dwt_initialise(0) != DWT_SUCCESS) {
        return false;
    }
    
    // Configure DW3000 for HDS-TWR operation
    // Note: Configuration parameters would need to be set properly for production
    
    return true;
}

bool positioning_adapter_ds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm)
{
    // Call the original DS-TWR measurement function
    // This interfaces with ds_twr.c implementation
    int8_t result_status;
    int32_t result_distance = DW1000send(current_device_id, anchor_id, DS_TWR_MODE, &result_status);
    
    if (result_status == 1 && result_distance > 0) {
        *distance_mm = (uint32_t)result_distance;
        return true;
    }
    
    return false;
}

bool positioning_adapter_hds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm)
{
    // HDS-TWR implementation is not complete in current codebase
    // Use DS-TWR as fallback for now
    return positioning_adapter_ds_twr_measure(anchor_id, distance_mm);
}

/*============================================================================
 * FILTERING AND VALIDATION FUNCTIONS
 *============================================================================*/

uint32_t positioning_adapter_filter_distance(uint8_t tag_id, uint32_t distance_mm)
{
    // Interface with filtering functions
    float distance_filtered = Distance_Filter(tag_id, (float)distance_mm / 1000.0f);
    return (uint32_t)(distance_filtered * 1000.0f);
}

bool positioning_adapter_validate_result(const positioning_result_t* result)
{
    if (!result) {
        return false;
    }
    
    // Basic validation checks
    // Check for reasonable coordinate values (within ±1000 meters)
    if (fabsf(result->x) > 1000.0f || fabsf(result->y) > 1000.0f || fabsf(result->z) > 1000.0f) {
        return false;
    }
    
    // Check for NaN or infinite values
    if (!isfinite(result->x) || !isfinite(result->y) || !isfinite(result->z)) {
        return false;
    }
    
    return true;
}

void positioning_adapter_get_statistics(uint32_t* total_calculations_out,
                                       uint32_t* successful_calculations_out,
                                       float* success_rate)
{
    if (total_calculations_out) {
        *total_calculations_out = total_calculations;
    }
    if (successful_calculations_out) {
        *successful_calculations_out = successful_calculations;
    }
    if (success_rate) {
        *success_rate = (total_calculations > 0) ? 
                       ((float)successful_calculations / (float)total_calculations) : 0.0f;
    }
}

void positioning_adapter_reset_statistics(void)
{
    total_calculations = 0;
    successful_calculations = 0;
}

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

void positioning_adapter_convert_anchor_data(const discovered_anchor_t* discovered_anchor,
                                            uint32_t distance_mm,
                                            positioning_anchor_data_t* positioning_anchor)
{
    if (!discovered_anchor || !positioning_anchor) {
        return;
    }
    
    positioning_anchor->x = discovered_anchor->position.x_cm / 100.0f;
    positioning_anchor->y = discovered_anchor->position.y_cm / 100.0f;
    positioning_anchor->z = discovered_anchor->position.z_cm / 100.0f;
    positioning_anchor->distance = (float)distance_mm / 1000.0f;  // Convert mm to meters
    positioning_anchor->anchor_id = discovered_anchor->anchor_id;
    positioning_anchor->is_valid = true;
}

positioning_algorithm_t positioning_adapter_select_algorithm(uint8_t anchor_count, bool has_3d_anchors)
{
    if (anchor_count < 3) {
        return POSITIONING_ALGO_2D_CENTER_MASS;  // Fallback, though insufficient
    }
    
    if (has_3d_anchors && anchor_count >= 4) {
        // Prefer 3D algorithms when we have 3D anchor positions
        return (anchor_count >= 6) ? POSITIONING_ALGO_3D_TAYLOR : POSITIONING_ALGO_3D_LEAST_SQUARE;
    } else {
        // Use 2D algorithms
        if (anchor_count == 3) {
            return POSITIONING_ALGO_2D_CENTER_MASS;
        } else {
            return (anchor_count >= 6) ? POSITIONING_ALGO_2D_TAYLOR : POSITIONING_ALGO_2D_CENTER_MASS;
        }
    }
}

/*============================================================================
 * PRIVATE HELPER FUNCTIONS
 *============================================================================*/

static bool validate_anchor_data(const positioning_anchor_data_t* anchor_data, uint8_t count)
{
    if (!anchor_data || count == 0) {
        return false;
    }
    
    for (uint8_t i = 0; i < count; i++) {
        if (!anchor_data[i].is_valid) {
            return false;
        }
        
        // Check for reasonable distance values (0.1m to 1000m)
        if (anchor_data[i].distance < 0.1f || anchor_data[i].distance > 1000.0f) {
            return false;
        }
        
        // Check for finite values
        if (!isfinite(anchor_data[i].x) || !isfinite(anchor_data[i].y) || 
            !isfinite(anchor_data[i].z) || !isfinite(anchor_data[i].distance)) {
            return false;
        }
    }
    
    return true;
}

static float estimate_accuracy(const positioning_anchor_data_t* anchor_data, uint8_t count, 
                              const positioning_result_t* result)
{
    if (!anchor_data || !result || count == 0) {
        return 999.0f;  // High uncertainty
    }
    
    // Calculate residuals (difference between measured and calculated distances)
    float residual_sum = 0.0f;
    
    for (uint8_t i = 0; i < count; i++) {
        float dx = result->x - anchor_data[i].x;
        float dy = result->y - anchor_data[i].y;
        float dz = result->z - anchor_data[i].z;
        float calculated_distance = sqrtf(dx*dx + dy*dy + dz*dz);
        
        float residual = fabsf(calculated_distance - anchor_data[i].distance);
        residual_sum += residual * residual;
    }
    
    // RMS residual as accuracy estimate
    float rms_residual = sqrtf(residual_sum / count);
    
    // Scale based on anchor count (more anchors = better accuracy)
    float geometry_factor = 1.0f / sqrtf((float)count);
    
    return rms_residual * geometry_factor;
}

void positioning_adapter_set_device_id(uint8_t device_id)
{
    current_device_id = device_id;
}

/*============================================================================
 * RANGING COMPATIBILITY FUNCTIONS
 *============================================================================*/

bool ranging_send_frame(const uint8_t* data, uint16_t length)
{
    // Send frame using DW3000 API
    dwt_writetxdata(length, data, 0);
    dwt_writetxfctrl(length, 0, 1);
    
    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
    return (ret == DWT_SUCCESS);
}

bool ranging_receive_frame(uint8_t* buffer, uint16_t buffer_size, int8_t* rssi, uint32_t timeout_ms)
{
    // Enable receiver
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    // Wait for reception or timeout
    uint32_t start_time = get_system_time_ms();
    
    while (get_system_time_ms() - start_time < timeout_ms) {
        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            // Successfully received frame
            uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
            
            if (frame_len <= buffer_size) {
                dwt_readrxdata(buffer, frame_len, 0);
                
                // Get RSSI if requested
                if (rssi) {
                    *rssi = -80; // Placeholder RSSI value
                }
                
                // Clear status register
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                return true;
            }
        }
        
        if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            // Error or timeout occurred
            break;
        }
        
        // Small delay to avoid busy waiting
        delay_ms(1);
    }
    
    // Clear error flags and turn off transceiver
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    dwt_forcetrxoff();
    return false;
}

bool ranging_measure_distance(uint8_t anchor_id, uint32_t* distance_mm, uint8_t* algorithm_used)
{
    // Use DS-TWR by default
    bool success = positioning_adapter_ds_twr_measure(anchor_id, distance_mm);
    
    if (success && algorithm_used) {
        *algorithm_used = DS_TWR_MODE; // DS-TWR algorithm
    }
    
    return success;
}
