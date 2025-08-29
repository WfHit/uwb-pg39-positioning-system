#ifndef TAG_COORDINATOR_H
#define TAG_COORDINATOR_H

#include "system_config.h"
#include "data_types.h"

// Function Declarations
bool tag_coordinator_init(uint8_t tag_id);
void tag_coordinator_process(void);
void tag_coordinator_reset(void);
bool tag_coordinator_is_active(void);

// Discovery Functions
void anchor_discovery_start(void);
void anchor_discovery_process(void);
bool process_discovery_response(const discovery_response_t* response, int8_t rssi);
void cleanup_stale_anchors(void);

// Anchor Selection Functions
uint8_t select_optimal_anchors(void);
uint8_t find_strongest_anchor(const uint8_t* candidates, uint8_t count);
uint8_t select_best_geometric_anchor(const uint8_t* candidates, uint8_t candidate_count,
                                    const uint8_t* selected, uint8_t selected_count);
float calculate_geometric_diversity(const uint8_t* anchor_indices, uint8_t count);
void update_anchor_list(const discovered_anchor_t* new_anchor);

// Measurement Functions
void perform_measurements(void);
bool perform_single_measurement(uint8_t anchor_index);
void send_measurement_immediately(const discovered_anchor_t* anchor, uint32_t distance_mm, uint8_t algorithm);

// Statistics and Monitoring
void get_coordinator_stats(system_stats_t* stats);
uint8_t get_active_anchor_count(void);
uint8_t get_current_zone(void);
float get_measurement_success_rate(void);

// Debug Functions
#ifdef DEBUG_MODE
void print_anchor_list(void);
void print_coordinator_state(void);
void print_measurement_stats(void);
#endif

#endif // TAG_COORDINATOR_H
