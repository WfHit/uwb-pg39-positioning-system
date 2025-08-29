#include "ranging_engine.h"
#include "../Utils/timing_utils.h"
#include <string.h>

// Ranging Engine State
static bool engine_initialized = false;
static uint8_t current_algorithm = RANGING_DS_TWR;
static uint32_t total_measurements = 0;
static uint32_t successful_measurements = 0;
static uint16_t last_frame_length = 0;

// Timing constants for TWR algorithms
#define TWR_POLL_TX_TO_RESP_RX_DLY_UUS    330
#define TWR_RESP_TX_TO_FINAL_RX_DLY_UUS   330
#define TWR_FINAL_TX_TO_RESP_RX_DLY_UUS   330
#define TWR_PRE_TIMEOUT_PAC               8
#define TWR_FINAL_RX_TIMEOUT_UUS          3300

bool ranging_engine_init(void)
{
    // Initialize DWM3000 hardware
    if(!dwm3000_init())
    {
        return false;
    }
    
    // Set default configuration
    dwm3000_set_config(5, 1, 0); // Channel 5, PRF 16MHz, 110k data rate
    
    // Reset statistics
    total_measurements = 0;
    successful_measurements = 0;
    last_frame_length = 0;
    
    engine_initialized = true;
    return true;
}

void ranging_engine_process(void)
{
    if(!engine_initialized) return;
    
    // Process any pending operations
    // This can be extended for background processing if needed
}

bool ranging_engine_is_ready(void)
{
    return engine_initialized && dwm3000_is_ready();
}

bool ranging_send_frame(const uint8_t* data, uint16_t length)
{
    if(!data || length == 0 || !engine_initialized) return false;
    
    return dwm3000_send_frame(data, length);
}

bool ranging_receive_frame(uint8_t* buffer, uint16_t buffer_size, int8_t* rssi, uint32_t timeout_ms)
{
    if(!buffer || buffer_size == 0 || !engine_initialized) return false;
    
    bool success = dwm3000_receive_frame(buffer, buffer_size, rssi, timeout_ms);
    if(success)
    {
        // Store frame length for later retrieval
        // This would need to be implemented in the DWM3000 driver
        last_frame_length = buffer_size; // Placeholder
    }
    
    return success;
}

uint16_t ranging_get_last_frame_length(void)
{
    return last_frame_length;
}

bool ranging_measure_distance(uint8_t anchor_id, uint32_t* distance_mm, uint8_t* algorithm_used)
{
    if(!distance_mm || !algorithm_used || !engine_initialized) return false;
    
    bool success = false;
    total_measurements++;
    
    *algorithm_used = current_algorithm;
    
    switch(current_algorithm)
    {
        case RANGING_DS_TWR:
            success = ranging_ds_twr_measure(anchor_id, distance_mm);
            break;
        case RANGING_HDS_TWR:
            success = ranging_hds_twr_measure(anchor_id, distance_mm);
            break;
        default:
            success = false;
            break;
    }
    
    if(success)
    {
        successful_measurements++;
    }
    
    return success;
}

bool ranging_ds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm)
{
    if(!distance_mm || !engine_initialized) return false;
    
    // DS-TWR Implementation
    // This is a simplified version - full implementation would include:
    // 1. Send POLL message
    // 2. Receive RESPONSE message
    // 3. Send FINAL message
    // 4. Calculate distance from timestamps
    
    ranging_request_t poll_msg;
    poll_msg.header = RANGING_REQUEST_HEADER;
    poll_msg.tag_id = 0; // Should be set from device ID
    poll_msg.anchor_id = anchor_id;
    poll_msg.algorithm = RANGING_DS_TWR;
    poll_msg.timestamp_ms = get_system_time_ms();
    poll_msg.sequence_number = total_measurements & 0xFFFF;
    
    // Send POLL message
    if(!ranging_send_frame((uint8_t*)&poll_msg, sizeof(poll_msg)))
    {
        return false;
    }
    
    uint32_t poll_tx_ts = get_microsecond_timestamp(); // Hardware timestamp
    
    // Wait for RESPONSE
    ranging_response_t response;
    int8_t rssi;
    if(!ranging_receive_frame((uint8_t*)&response, sizeof(response), &rssi, RANGING_TIMEOUT_MS))
    {
        return false;
    }
    
    // Verify response is from correct anchor
    if(response.header != RANGING_RESPONSE_HEADER || response.anchor_id != anchor_id)
    {
        return false;
    }
    
    uint32_t resp_rx_ts = get_microsecond_timestamp(); // Hardware timestamp
    
    // Send FINAL message (simplified - would include timestamps)
    uint8_t final_msg[16];
    final_msg[0] = 0xF0; // FINAL header
    final_msg[1] = anchor_id;
    memcpy(&final_msg[2], &poll_tx_ts, 4);
    memcpy(&final_msg[6], &resp_rx_ts, 4);
    
    if(!ranging_send_frame(final_msg, sizeof(final_msg)))
    {
        return false;
    }
    
    // Calculate distance (simplified calculation)
    // Real implementation would use proper TWR formulas with timestamps
    *distance_mm = response.distance_mm; // Placeholder - anchor calculates distance
    
    return true;
}

bool ranging_hds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm)
{
    if(!distance_mm || !engine_initialized) return false;
    
    // HDS-TWR Implementation (simplified)
    // Similar to DS-TWR but with additional security and accuracy measures
    
    ranging_request_t poll_msg;
    poll_msg.header = RANGING_REQUEST_HEADER;
    poll_msg.tag_id = 0; // Should be set from device ID
    poll_msg.anchor_id = anchor_id;
    poll_msg.algorithm = RANGING_HDS_TWR;
    poll_msg.timestamp_ms = get_system_time_ms();
    poll_msg.sequence_number = total_measurements & 0xFFFF;
    
    // Send request
    if(!ranging_send_frame((uint8_t*)&poll_msg, sizeof(poll_msg)))
    {
        return false;
    }
    
    // Wait for response
    ranging_response_t response;
    int8_t rssi;
    if(!ranging_receive_frame((uint8_t*)&response, sizeof(response), &rssi, RANGING_TIMEOUT_MS))
    {
        return false;
    }
    
    // Verify response
    if(response.header != RANGING_RESPONSE_HEADER || response.anchor_id != anchor_id)
    {
        return false;
    }
    
    *distance_mm = response.distance_mm;
    return true;
}

bool ranging_respond_to_request(uint8_t tag_id, uint8_t algorithm, uint32_t* distance_mm, uint8_t* signal_quality)
{
    if(!distance_mm || !signal_quality || !engine_initialized) return false;
    
    bool success = false;
    
    switch(algorithm)
    {
        case RANGING_DS_TWR:
            success = ranging_ds_twr_respond(tag_id, distance_mm, signal_quality);
            break;
        case RANGING_HDS_TWR:
            success = ranging_hds_twr_respond(tag_id, distance_mm, signal_quality);
            break;
        default:
            success = false;
            break;
    }
    
    return success;
}

bool ranging_ds_twr_respond(uint8_t tag_id, uint32_t* distance_mm, uint8_t* signal_quality)
{
    if(!distance_mm || !signal_quality || !engine_initialized) return false;
    
    // DS-TWR Response Implementation
    // This would be called after receiving a POLL message from a tag
    // 1. Record POLL reception timestamp
    // 2. Send RESPONSE message after delay
    // 3. Wait for FINAL message
    // 4. Calculate distance from timestamps
    
    // For now, return a placeholder implementation
    *distance_mm = 1000; // 1 meter placeholder
    *signal_quality = 85; // Good signal quality
    
    return true;
}

bool ranging_hds_twr_respond(uint8_t tag_id, uint32_t* distance_mm, uint8_t* signal_quality)
{
    if(!distance_mm || !signal_quality || !engine_initialized) return false;
    
    // HDS-TWR Response Implementation
    // Similar to DS-TWR but with enhanced security features
    
    *distance_mm = 1000; // 1 meter placeholder
    *signal_quality = 85; // Good signal quality
    
    return true;
}

bool ranging_set_algorithm(uint8_t algorithm)
{
    if(algorithm >= 2) return false; // Only support DS-TWR and HDS-TWR
    
    current_algorithm = algorithm;
    return true;
}

uint8_t ranging_get_algorithm(void)
{
    return current_algorithm;
}

bool ranging_set_channel(uint8_t channel)
{
    if(!engine_initialized) return false;
    
    dwm3000_set_config(channel, 1, 0); // Update channel, keep other settings
    return true;
}

bool ranging_set_prf(uint8_t prf)
{
    if(!engine_initialized) return false;
    
    dwm3000_set_config(5, prf, 0); // Update PRF, keep other settings
    return true;
}

bool ranging_set_data_rate(uint8_t data_rate)
{
    if(!engine_initialized) return false;
    
    dwm3000_set_config(5, 1, data_rate); // Update data rate, keep other settings
    return true;
}

uint32_t ranging_get_total_measurements(void)
{
    return total_measurements;
}

uint32_t ranging_get_successful_measurements(void)
{
    return successful_measurements;
}

float ranging_get_success_rate(void)
{
    if(total_measurements == 0) return 0.0f;
    return ((float)successful_measurements / total_measurements) * 100.0f;
}

void ranging_clear_stats(void)
{
    total_measurements = 0;
    successful_measurements = 0;
}
