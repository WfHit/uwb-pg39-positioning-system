#ifndef UWB_MAIN_H
#define UWB_MAIN_H

#include "../modules/core/system_config.h"
#include "../modules/core/data_types.h"
#include "../modules/tag_coordinator/tag_coordinator.h"
#include "../modules/anchor_manager/anchor_manager.h"
#include "../modules/communication/uart_protocol.h"
#include "../modules/positioning/positioning_adapter.h"

// Application initialization
bool uwb_system_init(uint8_t device_mode, uint8_t device_id);
void uwb_system_process(void);
void uwb_system_shutdown(void);

// Device mode configuration
bool configure_as_tag(uint8_t tag_id);
bool configure_as_anchor(uint8_t anchor_id, uint8_t zone_id, const position_t* position);

// System status and statistics
void get_system_status(system_stats_t* stats);
uint8_t get_device_mode(void);
uint8_t get_device_id(void);
bool is_system_ready(void);

// Configuration management
bool load_configuration_from_flash(void);
bool save_configuration_to_flash(void);
void reset_to_factory_defaults(void);

// Error handling
void system_error_handler(uint8_t error_code);
uint8_t get_last_error(void);
void clear_error_flags(void);

#endif // UWB_MAIN_H
