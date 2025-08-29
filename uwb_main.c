#include "uwb_main.h"
#include "Utils/timing_utils.h"
#include <string.h>

// System state
static uint8_t current_device_mode = DEVICE_MODE_TAG;
static uint8_t current_device_id = 0;
static bool system_initialized = false;
static uint8_t last_error = ERROR_NONE;

// Configuration structure
typedef struct {
    uint8_t device_mode;
    uint8_t device_id;
    uint8_t zone_id;
    position_t anchor_position;
    uint8_t ranging_algorithm;
    uint8_t channel;
    uint8_t prf;
    uint8_t data_rate;
    uint32_t checksum;
} system_config_t;

static system_config_t system_config;

// External hardware interface functions
extern bool hw_flash_read(uint32_t address, uint8_t* data, uint16_t length);
extern bool hw_flash_write(uint32_t address, const uint8_t* data, uint16_t length);
extern bool hw_peripherals_init(void);
extern void hw_system_reset(void);

bool uwb_system_init(uint8_t device_mode, uint8_t device_id)
{
    // Initialize hardware peripherals
    if(!hw_peripherals_init())
    {
        last_error = ERROR_COMMUNICATION;
        return false;
    }
    
    // Load configuration from flash
    if(!load_configuration_from_flash())
    {
        // Use default configuration if flash read fails
        reset_to_factory_defaults();
    }
    
    // Override with provided parameters
    current_device_mode = device_mode;
    current_device_id = device_id;
    
    // Initialize ranging engine
    if(!ranging_engine_init())
    {
        last_error = ERROR_COMMUNICATION;
        return false;
    }
    
    // Initialize UART protocol
    if(!uart_protocol_init(UART_BAUDRATE))
    {
        last_error = ERROR_COMMUNICATION;
        return false;
    }
    
    // Configure device based on mode
    bool config_success = false;
    switch(device_mode)
    {
        case DEVICE_MODE_TAG:
            config_success = configure_as_tag(device_id);
            break;
        case DEVICE_MODE_SUB_ANCHOR:
            config_success = configure_as_anchor(device_id, system_config.zone_id, 
                                               &system_config.anchor_position);
            break;
        case DEVICE_MODE_MAIN_ANCHOR:
            config_success = configure_as_main_anchor(device_id, system_config.zone_id, 
                                                    &system_config.anchor_position);
            break;
        default:
            last_error = ERROR_INVALID_DATA;
            return false;
    }
    
    if(!config_success)
    {
        last_error = ERROR_INVALID_DATA;
        return false;
    }
    
    system_initialized = true;
    last_error = ERROR_NONE;
    return true;
}

void uwb_system_process(void)
{
    if(!system_initialized) return;
    
    // Process ranging engine
    ranging_engine_process();
    
    // Process UART communication
    uart_protocol_process();
    
    // Process based on device mode
    switch(current_device_mode)
    {
        case DEVICE_MODE_TAG:
            tag_coordinator_process();
            break;
        case DEVICE_MODE_SUB_ANCHOR:
        case DEVICE_MODE_MAIN_ANCHOR:
            anchor_manager_process();
            break;
    }
    
    // Handle any errors
    uint8_t error = get_last_error();
    if(error != ERROR_NONE)
    {
        system_error_handler(error);
    }
}

bool configure_as_tag(uint8_t tag_id)
{
    current_device_mode = DEVICE_MODE_TAG;
    current_device_id = tag_id;
    
    // Initialize tag coordinator
    return tag_coordinator_init(tag_id);
}

bool configure_as_anchor(uint8_t anchor_id, uint8_t zone_id, const position_t* position)
{
    if(!position) return false;
    
    current_device_mode = DEVICE_MODE_SUB_ANCHOR;
    current_device_id = anchor_id;
    
    // Initialize anchor manager
    return anchor_manager_init(anchor_id, zone_id, position);
}

bool configure_as_main_anchor(uint8_t anchor_id, uint8_t zone_id, const position_t* position)
{
    if(!position) return false;
    
    current_device_mode = DEVICE_MODE_MAIN_ANCHOR;
    current_device_id = anchor_id;
    
    // Initialize anchor manager (same as sub-anchor for now)
    return anchor_manager_init(anchor_id, zone_id, position);
}

void get_system_status(system_stats_t* stats)
{
    if(!stats) return;
    
    memset(stats, 0, sizeof(system_stats_t));
    
    switch(current_device_mode)
    {
        case DEVICE_MODE_TAG:
            if(tag_coordinator_is_active())
            {
                get_coordinator_stats(stats);
            }
            break;
        case DEVICE_MODE_SUB_ANCHOR:
        case DEVICE_MODE_MAIN_ANCHOR:
            if(anchor_manager_is_active())
            {
                get_anchor_stats(stats);
            }
            break;
    }
    
    // Add UART statistics
    stats->uart_frames_sent = get_uart_frames_sent();
    stats->uart_errors = get_uart_errors();
}

uint8_t get_device_mode(void)
{
    return current_device_mode;
}

uint8_t get_device_id(void)
{
    return current_device_id;
}

bool is_system_ready(void)
{
    if(!system_initialized) return false;
    
    bool ranging_ready = ranging_engine_is_ready();
    bool uart_ready = is_uart_ready();
    
    bool mode_ready = false;
    switch(current_device_mode)
    {
        case DEVICE_MODE_TAG:
            mode_ready = tag_coordinator_is_active();
            break;
        case DEVICE_MODE_SUB_ANCHOR:
        case DEVICE_MODE_MAIN_ANCHOR:
            mode_ready = anchor_manager_is_active();
            break;
    }
    
    return ranging_ready && uart_ready && mode_ready;
}

bool load_configuration_from_flash(void)
{
    // Read configuration from flash memory
    if(!hw_flash_read(0x08000000, (uint8_t*)&system_config, sizeof(system_config_t)))
    {
        return false;
    }
    
    // Verify checksum
    uint32_t calculated_checksum = 0;
    uint8_t* config_bytes = (uint8_t*)&system_config;
    for(uint16_t i = 0; i < sizeof(system_config_t) - 4; i++)
    {
        calculated_checksum += config_bytes[i];
    }
    
    if(calculated_checksum != system_config.checksum)
    {
        return false; // Invalid configuration
    }
    
    return true;
}

bool save_configuration_to_flash(void)
{
    // Update current configuration
    system_config.device_mode = current_device_mode;
    system_config.device_id = current_device_id;
    
    // Calculate checksum
    system_config.checksum = 0;
    uint8_t* config_bytes = (uint8_t*)&system_config;
    for(uint16_t i = 0; i < sizeof(system_config_t) - 4; i++)
    {
        system_config.checksum += config_bytes[i];
    }
    
    // Write to flash
    return hw_flash_write(0x08000000, (uint8_t*)&system_config, sizeof(system_config_t));
}

void reset_to_factory_defaults(void)
{
    memset(&system_config, 0, sizeof(system_config_t));
    
    // Set default values
    system_config.device_mode = DEVICE_MODE_TAG;
    system_config.device_id = 1;
    system_config.zone_id = 1;
    system_config.anchor_position.x_cm = 0;
    system_config.anchor_position.y_cm = 0;
    system_config.anchor_position.z_cm = 250; // 2.5m height
    system_config.ranging_algorithm = RANGING_DS_TWR;
    system_config.channel = 5;
    system_config.prf = 1;
    system_config.data_rate = 0;
}

void system_error_handler(uint8_t error_code)
{
    switch(error_code)
    {
        case ERROR_TIMEOUT:
            // Handle timeout - maybe reset communication
            break;
        case ERROR_INVALID_DATA:
            // Handle invalid data - maybe reset to defaults
            break;
        case ERROR_NO_ANCHORS:
            // Handle no anchors found - continue discovery
            break;
        case ERROR_INSUFFICIENT_ANCHORS:
            // Handle insufficient anchors - lower requirements
            break;
        case ERROR_COMMUNICATION:
            // Handle communication error - reset system
            uwb_system_shutdown();
            hw_system_reset();
            break;
        default:
            break;
    }
    
    // Clear the error after handling
    clear_error_flags();
}

uint8_t get_last_error(void)
{
    return last_error;
}

void clear_error_flags(void)
{
    last_error = ERROR_NONE;
}

void uwb_system_shutdown(void)
{
    if(system_initialized)
    {
        // Shutdown modules
        switch(current_device_mode)
        {
            case DEVICE_MODE_TAG:
                tag_coordinator_reset();
                break;
            case DEVICE_MODE_SUB_ANCHOR:
            case DEVICE_MODE_MAIN_ANCHOR:
                anchor_manager_reset();
                break;
        }
        
        // Save configuration
        save_configuration_to_flash();
        
        system_initialized = false;
    }
}
