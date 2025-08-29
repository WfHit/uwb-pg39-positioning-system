#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include "system_config.h"

// Position Structure
typedef struct {
    int16_t x_cm;
    int16_t y_cm;
    int16_t z_cm;
} position_t;

// Discovered Anchor Information
typedef struct {
    uint8_t anchor_id;
    uint8_t zone_id;
    position_t position;
    int8_t rssi;
    uint32_t last_seen_ms;
    uint8_t signal_quality;
    bool is_active;
    uint16_t measurement_count;
    uint32_t last_measurement_ms;
} discovered_anchor_t;

// Measurement Data Structure for UART Output
typedef struct {
    uint16_t frame_header;      // 0xAA55
    uint8_t  tag_id;           
    uint32_t timestamp_ms;     
    uint8_t  anchor_id;        // Single measurement per frame
    uint32_t distance_mm;  
    int16_t  anchor_x_cm;     
    int16_t  anchor_y_cm;     
    int16_t  anchor_z_cm;     
    uint8_t  signal_quality; 
    uint8_t  zone_id;      
    uint8_t  algorithm_used;   // 0=DS-TWR, 1=HDS-TWR
    uint8_t  measurement_sequence;
    uint16_t checksum;         
} __attribute__((packed)) uwb_measurement_frame_t;

// Discovery Request Frame
typedef struct {
    uint8_t header;            // 0xD0
    uint8_t tag_id;
    uint8_t sequence_number;
    uint32_t timestamp_ms;
    uint8_t zone_filter;       // 0xFF for all zones
    uint8_t reserved[2];
} __attribute__((packed)) discovery_request_t;

// Discovery Response Frame
typedef struct {
    uint8_t header;            // 0xD1
    uint8_t anchor_id;
    uint8_t zone_id;
    position_t position;
    uint8_t capabilities;      // Bit flags for anchor capabilities
    uint8_t load_factor;       // Current load (0-100%)
    uint16_t sequence_response;
} __attribute__((packed)) discovery_response_t;

// Ranging Request Frame
typedef struct {
    uint8_t header;            // 0xR0
    uint8_t tag_id;
    uint8_t anchor_id;
    uint8_t algorithm;         // DS-TWR or HDS-TWR
    uint32_t timestamp_ms;
    uint16_t sequence_number;
} __attribute__((packed)) ranging_request_t;

// Ranging Response Frame
typedef struct {
    uint8_t header;            // 0xR1
    uint8_t anchor_id;
    uint8_t tag_id;
    uint32_t distance_mm;
    uint8_t signal_quality;
    uint32_t timestamp_ms;
    uint16_t sequence_response;
} __attribute__((packed)) ranging_response_t;

// Tag Coordinator State
typedef struct {
    discovered_anchor_t anchor_list[MAX_TOTAL_ANCHORS];
    uint8_t anchor_count;
    uint8_t selected_anchors[MAX_SIMULTANEOUS_MEASUREMENTS];
    uint8_t current_zone;
    uint32_t last_discovery_ms;
    uint32_t last_measurement_ms;
    uint16_t measurement_sequence;
    uint8_t current_measurement_index;
    bool discovery_active;
    bool measurement_active;
    uint32_t total_measurements;
    uint32_t successful_measurements;
} tag_coordinator_t;

// Anchor Manager State
typedef struct {
    uint8_t anchor_id;
    uint8_t zone_id;
    position_t position;
    bool is_initialized;
    uint32_t last_discovery_broadcast;
    uint32_t total_requests_received;
    uint32_t total_responses_sent;
    uint8_t current_load;
} anchor_manager_t;

// System Statistics
typedef struct {
    uint32_t uptime_ms;
    uint32_t total_discoveries;
    uint32_t total_measurements;
    uint32_t successful_measurements;
    uint32_t failed_measurements;
    uint32_t uart_frames_sent;
    uint32_t uart_errors;
    float success_rate;
    uint8_t active_anchors;
} system_stats_t;

#endif // DATA_TYPES_H
