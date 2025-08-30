#include "anchor_manager.h"
#include "../positioning/positioning_adapter.h"
#include "../utils/timing_utils.h"
#include <string.h>

static anchor_manager_t anchor_mgr;
static bool manager_initialized = false;
static bool response_throttling = false;

bool anchor_manager_init(uint8_t anchor_id, uint8_t zone_id, const position_t* position)
{
    if(!position) return false;

    memset(&anchor_mgr, 0, sizeof(anchor_manager_t));

    anchor_mgr.anchor_id = anchor_id;
    anchor_mgr.zone_id = zone_id;
    anchor_mgr.position = *position;
    anchor_mgr.is_initialized = true;
    anchor_mgr.last_discovery_broadcast = 0;
    anchor_mgr.total_requests_received = 0;
    anchor_mgr.total_responses_sent = 0;
    anchor_mgr.current_load = 0;

    manager_initialized = true;
    return true;
}

void anchor_manager_process(void)
{
    if(!manager_initialized) return;

    uint8_t rx_buffer[64];
    uint8_t frame_length;
    int8_t rssi;

    // Check for incoming frames
    if(ranging_receive_frame(rx_buffer, sizeof(rx_buffer), &rssi, 1)) // 1ms timeout for non-blocking
    {
        frame_length = ranging_get_last_frame_length();

        // Process different frame types
        if(frame_length >= sizeof(discovery_request_t) &&
           rx_buffer[0] == DISCOVERY_REQUEST_HEADER)
        {
            process_discovery_request((discovery_request_t*)rx_buffer);
        }
        else if(frame_length >= sizeof(ranging_request_t) &&
                rx_buffer[0] == RANGING_REQUEST_HEADER)
        {
            process_ranging_request((ranging_request_t*)rx_buffer);
        }
    }

    // Update load statistics periodically
    static uint32_t last_load_update = 0;
    uint32_t current_time = get_system_time_ms();
    if(current_time - last_load_update >= 1000) // Every second
    {
        update_load_statistics();
        last_load_update = current_time;
    }
}

bool process_discovery_request(const discovery_request_t* request)
{
    if(!request || !manager_initialized) return false;

    anchor_mgr.total_requests_received++;

    // Check if this anchor should respond (zone filtering)
    if(request->zone_filter != 0xFF && request->zone_filter != anchor_mgr.zone_id)
    {
        return false; // Wrong zone
    }

    // Check load and throttling
    if(response_throttling && is_overloaded())
    {
        return false; // Skip response due to high load
    }

    // Send discovery response
    send_discovery_response(request->tag_id, request->sequence_number);

    return true;
}

void send_discovery_response(uint8_t tag_id, uint16_t sequence)
{
    discovery_response_t response;

    response.header = DISCOVERY_RESPONSE_HEADER;
    response.anchor_id = anchor_mgr.anchor_id;
    response.zone_id = anchor_mgr.zone_id;
    response.position = anchor_mgr.position;
    response.capabilities = 0x01; // Basic ranging capability
    response.load_factor = anchor_mgr.current_load;
    response.sequence_response = sequence;

    if(ranging_send_frame((uint8_t*)&response, sizeof(response)))
    {
        anchor_mgr.total_responses_sent++;
        anchor_mgr.last_discovery_broadcast = get_system_time_ms();
    }
}

bool process_ranging_request(const ranging_request_t* request)
{
    if(!request || !manager_initialized) return false;

    // Verify this request is for this anchor
    if(request->anchor_id != anchor_mgr.anchor_id)
    {
        return false;
    }

    anchor_mgr.total_requests_received++;

    // Check load and throttling
    if(response_throttling && is_overloaded())
    {
        return false; // Skip response due to high load
    }

    // Perform ranging measurement
    uint32_t distance_mm = 0;
    uint8_t signal_quality = 0;

    bool success = ranging_respond_to_request(request->tag_id, request->algorithm,
                                            &distance_mm, &signal_quality);

    if(success)
    {
        send_ranging_response(request, distance_mm, signal_quality);
        return true;
    }

    return false;
}

void send_ranging_response(const ranging_request_t* request, uint32_t distance_mm, uint8_t quality)
{
    ranging_response_t response;

    response.header = RANGING_RESPONSE_HEADER;
    response.anchor_id = anchor_mgr.anchor_id;
    response.tag_id = request->tag_id;
    response.distance_mm = distance_mm;
    response.signal_quality = quality;
    response.timestamp_ms = get_system_time_ms();
    response.sequence_response = request->sequence_number;

    if(ranging_send_frame((uint8_t*)&response, sizeof(response)))
    {
        anchor_mgr.total_responses_sent++;
    }
}

bool set_anchor_position(const position_t* position)
{
    if(!position || !manager_initialized) return false;

    anchor_mgr.position = *position;
    return true;
}

bool set_anchor_zone(uint8_t zone_id)
{
    if(!manager_initialized) return false;

    anchor_mgr.zone_id = zone_id;
    return true;
}

bool get_anchor_position(position_t* position)
{
    if(!position || !manager_initialized) return false;

    *position = anchor_mgr.position;
    return true;
}

uint8_t get_anchor_zone(void)
{
    return manager_initialized ? anchor_mgr.zone_id : 0;
}

uint8_t get_anchor_id(void)
{
    return manager_initialized ? anchor_mgr.anchor_id : 0;
}

uint8_t get_anchor_load(void)
{
    return manager_initialized ? anchor_mgr.current_load : 0;
}

uint32_t get_total_requests(void)
{
    return manager_initialized ? anchor_mgr.total_requests_received : 0;
}

uint32_t get_total_responses(void)
{
    return manager_initialized ? anchor_mgr.total_responses_sent : 0;
}

void get_anchor_stats(system_stats_t* stats)
{
    if(!stats || !manager_initialized) return;

    stats->uptime_ms = get_system_time_ms();
    stats->total_discoveries = anchor_mgr.total_requests_received;
    stats->total_measurements = anchor_mgr.total_responses_sent;
    stats->successful_measurements = anchor_mgr.total_responses_sent;
    stats->failed_measurements = anchor_mgr.total_requests_received - anchor_mgr.total_responses_sent;
    stats->success_rate = (anchor_mgr.total_requests_received > 0) ?
                         ((float)anchor_mgr.total_responses_sent / anchor_mgr.total_requests_received * 100.0f) : 100.0f;
    stats->active_anchors = 1; // This anchor is active
}

void update_load_statistics(void)
{
    if(!manager_initialized) return;

    static uint32_t last_request_count = 0;
    static uint32_t last_update_time = 0;

    uint32_t current_time = get_system_time_ms();
    uint32_t time_diff = current_time - last_update_time;

    if(time_diff >= 1000) // Calculate load over 1 second window
    {
        uint32_t request_diff = anchor_mgr.total_requests_received - last_request_count;

        // Calculate load as percentage of maximum expected requests per second
        // Assuming max 50 requests per second as 100% load
        anchor_mgr.current_load = (request_diff > 50) ? 100 : (request_diff * 2);

        last_request_count = anchor_mgr.total_requests_received;
        last_update_time = current_time;
    }
}

bool is_overloaded(void)
{
    return anchor_mgr.current_load > 80; // 80% load threshold
}

void throttle_responses(bool enable)
{
    response_throttling = enable;
}

void anchor_manager_reset(void)
{
    manager_initialized = false;
    memset(&anchor_mgr, 0, sizeof(anchor_manager_t));
}

bool anchor_manager_is_active(void)
{
    return manager_initialized;
}
