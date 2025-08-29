#ifndef RANGING_ENGINE_H
#define RANGING_ENGINE_H

#include "../Core/system_config.h"
#include "../Core/data_types.h"

// Function Declarations
bool ranging_engine_init(void);
void ranging_engine_process(void);
bool ranging_engine_is_ready(void);

// Frame Transmission/Reception
bool ranging_send_frame(const uint8_t* data, uint16_t length);
bool ranging_receive_frame(uint8_t* buffer, uint16_t buffer_size, int8_t* rssi, uint32_t timeout_ms);
uint16_t ranging_get_last_frame_length(void);

// Distance Measurement (Tag Side)
bool ranging_measure_distance(uint8_t anchor_id, uint32_t* distance_mm, uint8_t* algorithm_used);
bool ranging_ds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm);
bool ranging_hds_twr_measure(uint8_t anchor_id, uint32_t* distance_mm);

// Distance Response (Anchor Side)
bool ranging_respond_to_request(uint8_t tag_id, uint8_t algorithm, uint32_t* distance_mm, uint8_t* signal_quality);
bool ranging_ds_twr_respond(uint8_t tag_id, uint32_t* distance_mm, uint8_t* signal_quality);
bool ranging_hds_twr_respond(uint8_t tag_id, uint32_t* distance_mm, uint8_t* signal_quality);

// Configuration
bool ranging_set_algorithm(uint8_t algorithm);
uint8_t ranging_get_algorithm(void);
bool ranging_set_channel(uint8_t channel);
bool ranging_set_prf(uint8_t prf);
bool ranging_set_data_rate(uint8_t data_rate);

// Statistics and Diagnostics
uint32_t ranging_get_total_measurements(void);
uint32_t ranging_get_successful_measurements(void);
float ranging_get_success_rate(void);
void ranging_clear_stats(void);

// Hardware Interface (to be implemented with DWM3000 driver)
extern bool dwm3000_init(void);
extern bool dwm3000_send_frame(const uint8_t* data, uint16_t length);
extern bool dwm3000_receive_frame(uint8_t* buffer, uint16_t buffer_size, int8_t* rssi, uint32_t timeout_ms);
extern bool dwm3000_is_ready(void);
extern void dwm3000_set_config(uint8_t channel, uint8_t prf, uint8_t data_rate);

#endif // RANGING_ENGINE_H
