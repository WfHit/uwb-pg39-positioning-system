#include "tag_coordinator.h"
#include "../communication/uart_protocol.h"
#include "../positioning/positioning_adapter.h"
#include "../utils/geometry_utils.h"
#include "../utils/timing_utils.h"
#include <string.h>
#include <math.h>

static tag_coordinator_t tag_coord;
static uint8_t tag_device_id;
static bool coordinator_initialized = false;

bool tag_coordinator_init(uint8_t tag_id)
{
    memset(&tag_coord, 0, sizeof(tag_coordinator_t));
    
    tag_device_id = tag_id;
    tag_coord.current_zone = 0xFF; // Unknown zone initially
    tag_coord.last_discovery_ms = 0;
    tag_coord.last_measurement_ms = 0;
    tag_coord.measurement_sequence = 0;
    tag_coord.current_measurement_index = 0;
    tag_coord.discovery_active = false;
    tag_coord.measurement_active = false;
    tag_coord.total_measurements = 0;
    tag_coord.successful_measurements = 0;
    
    // Initialize selected anchors array
    for(uint8_t i = 0; i < MAX_SIMULTANEOUS_MEASUREMENTS; i++)
    {
        tag_coord.selected_anchors[i] = 0xFF; // Invalid index
    }
    
    coordinator_initialized = true;
    return true;
}

void tag_coordinator_process(void)
{
    if(!coordinator_initialized) return;
    
    uint32_t current_time = get_system_time_ms();
    
    // Discovery phase - every 2 seconds
    if(current_time - tag_coord.last_discovery_ms >= DISCOVERY_INTERVAL_MS)
    {
        anchor_discovery_start();
        tag_coord.last_discovery_ms = current_time;
    }
    
    // Process ongoing discovery
    if(tag_coord.discovery_active)
    {
        anchor_discovery_process();
    }
    
    // Measurement phase - 5Hz (200ms interval)
    if(current_time - tag_coord.last_measurement_ms >= MEASUREMENT_INTERVAL_MS)
    {
        if(tag_coord.anchor_count >= MIN_ANCHORS_FOR_POSITIONING)
        {
            uint8_t selected_count = select_optimal_anchors();
            if(selected_count >= MIN_ANCHORS_FOR_POSITIONING)
            {
                perform_measurements();
            }
        }
        tag_coord.last_measurement_ms = current_time;
    }
    
    // Cleanup stale anchors every 30 seconds
    static uint32_t last_cleanup = 0;
    if(current_time - last_cleanup >= 30000)
    {
        cleanup_stale_anchors();
        last_cleanup = current_time;
    }
}

void anchor_discovery_start(void)
{
    discovery_request_t discovery_msg;
    discovery_msg.header = DISCOVERY_REQUEST_HEADER;
    discovery_msg.tag_id = tag_device_id;
    discovery_msg.sequence_number = tag_coord.measurement_sequence++;
    discovery_msg.timestamp_ms = get_system_time_ms();
    discovery_msg.zone_filter = 0xFF; // All zones
    discovery_msg.reserved[0] = 0;
    discovery_msg.reserved[1] = 0;
    
    // Send discovery request via UWB
    if(ranging_send_frame((uint8_t*)&discovery_msg, sizeof(discovery_msg)))
    {
        tag_coord.discovery_active = true;
    }
}

void anchor_discovery_process(void)
{
    discovery_response_t response;
    int8_t rssi;
    
    // Check for discovery responses with timeout
    if(ranging_receive_frame((uint8_t*)&response, sizeof(response), &rssi, DISCOVERY_TIMEOUT_MS))
    {
        if(response.header == DISCOVERY_RESPONSE_HEADER)
        {
            process_discovery_response(&response, rssi);
        }
    }
    else
    {
        // Timeout reached, end discovery phase
        tag_coord.discovery_active = false;
    }
}

bool process_discovery_response(const discovery_response_t* response, int8_t rssi)
{
    if(!response) return false;
    
    // Check if anchor already exists
    int8_t existing_index = -1;
    for(uint8_t i = 0; i < tag_coord.anchor_count; i++)
    {
        if(tag_coord.anchor_list[i].anchor_id == response->anchor_id)
        {
            existing_index = i;
            break;
        }
    }
    
    discovered_anchor_t anchor;
    anchor.anchor_id = response->anchor_id;
    anchor.zone_id = response->zone_id;
    anchor.position = response->position;
    anchor.rssi = rssi;
    anchor.last_seen_ms = get_system_time_ms();
    anchor.signal_quality = (rssi > RSSI_THRESHOLD) ? 
                           (100 + rssi + 80) : 0; // Convert RSSI to 0-100 scale
    anchor.is_active = (anchor.signal_quality > 0);
    
    if(existing_index >= 0)
    {
        // Update existing anchor
        tag_coord.anchor_list[existing_index] = anchor;
        tag_coord.anchor_list[existing_index].measurement_count++;
    }
    else if(tag_coord.anchor_count < MAX_TOTAL_ANCHORS)
    {
        // Add new anchor
        tag_coord.anchor_list[tag_coord.anchor_count] = anchor;
        tag_coord.anchor_list[tag_coord.anchor_count].measurement_count = 0;
        tag_coord.anchor_count++;
    }
    
    return true;
}

uint8_t select_optimal_anchors(void)
{
    // Step 1: Filter anchors by signal quality and activity
    uint8_t candidates[MAX_TOTAL_ANCHORS];
    uint8_t candidate_count = 0;
    
    for(uint8_t i = 0; i < tag_coord.anchor_count; i++)
    {
        if(tag_coord.anchor_list[i].is_active && 
           tag_coord.anchor_list[i].signal_quality > 0 && 
           tag_coord.anchor_list[i].rssi > RSSI_THRESHOLD)
        {
            candidates[candidate_count++] = i;
        }
    }
    
    if(candidate_count < MIN_ANCHORS_FOR_POSITIONING)
        return 0;
    
    // Step 2: Clear previous selection
    for(uint8_t i = 0; i < MAX_SIMULTANEOUS_MEASUREMENTS; i++)
    {
        tag_coord.selected_anchors[i] = 0xFF;
    }
    
    uint8_t selected_count = 0;
    
    // Step 3: Select strongest anchor first
    uint8_t strongest_idx = find_strongest_anchor(candidates, candidate_count);
    if(strongest_idx != 0xFF)
    {
        tag_coord.selected_anchors[selected_count++] = candidates[strongest_idx];
        
        // Remove from candidates
        for(uint8_t i = strongest_idx; i < candidate_count - 1; i++)
        {
            candidates[i] = candidates[i + 1];
        }
        candidate_count--;
    }
    
    // Step 4: Select remaining anchors for geometric diversity
    while(selected_count < MAX_SIMULTANEOUS_MEASUREMENTS && 
          candidate_count > 0 && 
          selected_count < candidate_count + 1)
    {
        uint8_t best_idx = select_best_geometric_anchor(candidates, candidate_count, 
                                                       tag_coord.selected_anchors, selected_count);
        if(best_idx != 0xFF && best_idx < candidate_count)
        {
            tag_coord.selected_anchors[selected_count++] = candidates[best_idx];
            
            // Remove from candidates
            for(uint8_t i = best_idx; i < candidate_count - 1; i++)
            {
                candidates[i] = candidates[i + 1];
            }
            candidate_count--;
        }
        else
            break;
    }
    
    return selected_count;
}

uint8_t find_strongest_anchor(const uint8_t* candidates, uint8_t count)
{
    if(count == 0) return 0xFF;
    
    uint8_t strongest_idx = 0;
    int8_t strongest_rssi = tag_coord.anchor_list[candidates[0]].rssi;
    
    for(uint8_t i = 1; i < count; i++)
    {
        if(tag_coord.anchor_list[candidates[i]].rssi > strongest_rssi)
        {
            strongest_rssi = tag_coord.anchor_list[candidates[i]].rssi;
            strongest_idx = i;
        }
    }
    
    return strongest_idx;
}

uint8_t select_best_geometric_anchor(const uint8_t* candidates, uint8_t candidate_count,
                                    const uint8_t* selected, uint8_t selected_count)
{
    if(candidate_count == 0 || selected_count == 0) return 0xFF;
    
    float best_diversity = -1.0f;
    uint8_t best_idx = 0xFF;
    
    for(uint8_t i = 0; i < candidate_count; i++)
    {
        // Create temporary selection including this candidate
        uint8_t temp_selection[MAX_SIMULTANEOUS_MEASUREMENTS];
        for(uint8_t j = 0; j < selected_count; j++)
        {
            temp_selection[j] = selected[j];
        }
        temp_selection[selected_count] = candidates[i];
        
        // Calculate geometric diversity
        float diversity = calculate_geometric_diversity_score(temp_selection, selected_count + 1, 
                                                             tag_coord.anchor_list);
        
        // Weight by signal quality
        float signal_weight = tag_coord.anchor_list[candidates[i]].signal_quality / 100.0f;
        float weighted_score = diversity * 0.7f + signal_weight * 0.3f;
        
        if(weighted_score > best_diversity)
        {
            best_diversity = weighted_score;
            best_idx = i;
        }
    }
    
    return best_idx;
}

void perform_measurements(void)
{
    tag_coord.measurement_active = true;
    
    // Perform ranging with each selected anchor individually
    for(uint8_t i = 0; i < MAX_SIMULTANEOUS_MEASUREMENTS; i++)
    {
        if(tag_coord.selected_anchors[i] != 0xFF)
        {
            bool success = perform_single_measurement(tag_coord.selected_anchors[i]);
            if(success)
            {
                tag_coord.successful_measurements++;
            }
            tag_coord.total_measurements++;
            
            // Small delay between measurements to avoid interference
            delay_ms(10);
        }
    }
    
    tag_coord.measurement_active = false;
}

bool perform_single_measurement(uint8_t anchor_index)
{
    if(anchor_index >= tag_coord.anchor_count) return false;
    
    discovered_anchor_t* anchor = &tag_coord.anchor_list[anchor_index];
    
    uint32_t distance_mm = 0;
    uint8_t algorithm = 0; // Will be set by ranging engine
    
    // Perform ranging measurement
    bool success = ranging_measure_distance(anchor->anchor_id, &distance_mm, &algorithm);
    
    if(success)
    {
        // Update anchor statistics
        anchor->last_measurement_ms = get_system_time_ms();
        anchor->measurement_count++;
        
        // Send measurement immediately with accurate timestamp
        send_measurement_immediately(anchor, distance_mm, algorithm);
        
        return true;
    }
    
    return false;
}

void send_measurement_immediately(const discovered_anchor_t* anchor, uint32_t distance_mm, uint8_t algorithm)
{
    uwb_measurement_frame_t frame;
    
    frame.frame_header = MEASUREMENT_FRAME_HEADER;
    frame.tag_id = tag_device_id;
    frame.timestamp_ms = get_system_time_ms();
    frame.anchor_id = anchor->anchor_id;
    frame.distance_mm = distance_mm;
    frame.anchor_x_cm = anchor->position.x_cm;
    frame.anchor_y_cm = anchor->position.y_cm;
    frame.anchor_z_cm = anchor->position.z_cm;
    frame.signal_quality = anchor->signal_quality;
    frame.zone_id = anchor->zone_id;
    frame.algorithm_used = algorithm;
    frame.measurement_sequence = tag_coord.measurement_sequence++;
    
    // Send via UART
    uart_send_measurement_frame(&frame);
}

void cleanup_stale_anchors(void)
{
    uint32_t current_time = get_system_time_ms();
    uint8_t active_count = 0;
    
    for(uint8_t i = 0; i < tag_coord.anchor_count; i++)
    {
        if(current_time - tag_coord.anchor_list[i].last_seen_ms > ANCHOR_TIMEOUT_MS)
        {
            tag_coord.anchor_list[i].is_active = false;
        }
        else
        {
            if(active_count != i)
            {
                tag_coord.anchor_list[active_count] = tag_coord.anchor_list[i];
            }
            active_count++;
        }
    }
    
    tag_coord.anchor_count = active_count;
}

void get_coordinator_stats(system_stats_t* stats)
{
    if(!stats) return;
    
    stats->uptime_ms = get_system_time_ms();
    stats->total_measurements = tag_coord.total_measurements;
    stats->successful_measurements = tag_coord.successful_measurements;
    stats->failed_measurements = tag_coord.total_measurements - tag_coord.successful_measurements;
    stats->success_rate = (tag_coord.total_measurements > 0) ? 
                         ((float)tag_coord.successful_measurements / tag_coord.total_measurements * 100.0f) : 0.0f;
    stats->active_anchors = get_active_anchor_count();
}

uint8_t get_active_anchor_count(void)
{
    uint8_t count = 0;
    for(uint8_t i = 0; i < tag_coord.anchor_count; i++)
    {
        if(tag_coord.anchor_list[i].is_active)
            count++;
    }
    return count;
}

uint8_t get_current_zone(void)
{
    return tag_coord.current_zone;
}

float get_measurement_success_rate(void)
{
    if(tag_coord.total_measurements == 0) return 0.0f;
    return ((float)tag_coord.successful_measurements / tag_coord.total_measurements) * 100.0f;
}

void tag_coordinator_reset(void)
{
    coordinator_initialized = false;
    memset(&tag_coord, 0, sizeof(tag_coordinator_t));
}

bool tag_coordinator_is_active(void)
{
    return coordinator_initialized;
}
