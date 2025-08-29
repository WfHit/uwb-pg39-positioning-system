#include "timing_utils.h"

// Timer management structures
#define MAX_TIMERS 8
typedef struct {
    bool active;
    uint32_t start_time_ms;
    uint32_t duration_ms;
} software_timer_t;

static software_timer_t timers[MAX_TIMERS];

// Performance measurement
#define MAX_MEASUREMENTS 4
typedef struct {
    bool active;
    uint32_t start_time_ms;
    uint32_t total_time_ms;
    uint32_t measurement_count;
} performance_measurement_t;

static performance_measurement_t measurements[MAX_MEASUREMENTS];

uint32_t get_system_time_ms(void)
{
    return hw_get_system_tick();
}

uint64_t get_microsecond_timestamp(void)
{
    return hw_get_high_precision_timestamp();
}

void delay_ms(uint32_t milliseconds)
{
    hw_delay_ms(milliseconds);
}

void delay_us(uint32_t microseconds)
{
    hw_delay_us(microseconds);
}

bool start_timer(uint8_t timer_id, uint32_t duration_ms)
{
    if(timer_id >= MAX_TIMERS) return false;
    
    timers[timer_id].active = true;
    timers[timer_id].start_time_ms = get_system_time_ms();
    timers[timer_id].duration_ms = duration_ms;
    
    return true;
}

bool stop_timer(uint8_t timer_id)
{
    if(timer_id >= MAX_TIMERS) return false;
    
    timers[timer_id].active = false;
    return true;
}

bool is_timer_expired(uint8_t timer_id)
{
    if(timer_id >= MAX_TIMERS || !timers[timer_id].active) return false;
    
    uint32_t current_time = get_system_time_ms();
    uint32_t elapsed = current_time - timers[timer_id].start_time_ms;
    
    if(elapsed >= timers[timer_id].duration_ms)
    {
        timers[timer_id].active = false; // Auto-stop expired timer
        return true;
    }
    
    return false;
}

uint32_t get_timer_remaining(uint8_t timer_id)
{
    if(timer_id >= MAX_TIMERS || !timers[timer_id].active) return 0;
    
    uint32_t current_time = get_system_time_ms();
    uint32_t elapsed = current_time - timers[timer_id].start_time_ms;
    
    if(elapsed >= timers[timer_id].duration_ms)
    {
        return 0;
    }
    
    return timers[timer_id].duration_ms - elapsed;
}

void performance_start_measurement(uint8_t measurement_id)
{
    if(measurement_id >= MAX_MEASUREMENTS) return;
    
    measurements[measurement_id].active = true;
    measurements[measurement_id].start_time_ms = get_system_time_ms();
}

uint32_t performance_end_measurement(uint8_t measurement_id)
{
    if(measurement_id >= MAX_MEASUREMENTS || !measurements[measurement_id].active) return 0;
    
    uint32_t end_time = get_system_time_ms();
    uint32_t duration = end_time - measurements[measurement_id].start_time_ms;
    
    measurements[measurement_id].total_time_ms += duration;
    measurements[measurement_id].measurement_count++;
    measurements[measurement_id].active = false;
    
    return duration;
}

void performance_reset_measurements(void)
{
    for(uint8_t i = 0; i < MAX_MEASUREMENTS; i++)
    {
        measurements[i].active = false;
        measurements[i].start_time_ms = 0;
        measurements[i].total_time_ms = 0;
        measurements[i].measurement_count = 0;
    }
}

bool has_timeout_occurred(uint32_t start_time_ms, uint32_t timeout_ms)
{
    uint32_t current_time = get_system_time_ms();
    
    // Handle timer overflow
    if(current_time >= start_time_ms)
    {
        return (current_time - start_time_ms) >= timeout_ms;
    }
    else
    {
        // Timer has overflowed
        uint32_t elapsed = (0xFFFFFFFF - start_time_ms) + current_time + 1;
        return elapsed >= timeout_ms;
    }
}

uint32_t get_elapsed_time_ms(uint32_t start_time_ms)
{
    uint32_t current_time = get_system_time_ms();
    
    // Handle timer overflow
    if(current_time >= start_time_ms)
    {
        return current_time - start_time_ms;
    }
    else
    {
        // Timer has overflowed
        return (0xFFFFFFFF - start_time_ms) + current_time + 1;
    }
}
