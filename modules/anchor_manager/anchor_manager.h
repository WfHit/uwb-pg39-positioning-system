#ifndef ANCHOR_MANAGER_H
#define ANCHOR_MANAGER_H

#include "system_config.h"
#include "data_types.h"

// Function Declarations
bool anchor_manager_init(uint8_t anchor_id, uint8_t zone_id, const position_t* position);
void anchor_manager_process(void);
void anchor_manager_reset(void);
bool anchor_manager_is_active(void);

// Discovery Response Functions
bool process_discovery_request(const discovery_request_t* request);
void send_discovery_response(uint8_t tag_id, uint16_t sequence);

// Ranging Response Functions
bool process_ranging_request(const ranging_request_t* request);
void send_ranging_response(const ranging_request_t* request, uint32_t distance_mm, uint8_t quality);

// Configuration Functions
bool set_anchor_position(const position_t* position);
bool set_anchor_zone(uint8_t zone_id);
bool get_anchor_position(position_t* position);
uint8_t get_anchor_zone(void);
uint8_t get_anchor_id(void);

// Status and Statistics
uint8_t get_anchor_load(void);
uint32_t get_total_requests(void);
uint32_t get_total_responses(void);
void get_anchor_stats(system_stats_t* stats);

// Load Management
void update_load_statistics(void);
bool is_overloaded(void);
void throttle_responses(bool enable);

#endif // ANCHOR_MANAGER_H
