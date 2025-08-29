#ifndef TIMING_UTILS_H
#define TIMING_UTILS_H

#include "../Core/system_config.h"
#include <stdint.h>

// System timing functions
uint32_t get_system_time_ms(void);
uint64_t get_microsecond_timestamp(void);
void delay_ms(uint32_t milliseconds);
void delay_us(uint32_t microseconds);

// Timer management
bool start_timer(uint8_t timer_id, uint32_t duration_ms);
bool stop_timer(uint8_t timer_id);
bool is_timer_expired(uint8_t timer_id);
uint32_t get_timer_remaining(uint8_t timer_id);

// Performance measurement
void performance_start_measurement(uint8_t measurement_id);
uint32_t performance_end_measurement(uint8_t measurement_id);
void performance_reset_measurements(void);

// Timeout utilities
bool has_timeout_occurred(uint32_t start_time_ms, uint32_t timeout_ms);
uint32_t get_elapsed_time_ms(uint32_t start_time_ms);

// External hardware timing functions (to be implemented)
extern uint32_t hw_get_system_tick(void);
extern uint64_t hw_get_high_precision_timestamp(void);
extern void hw_delay_ms(uint32_t ms);
extern void hw_delay_us(uint32_t us);

#endif // TIMING_UTILS_H
