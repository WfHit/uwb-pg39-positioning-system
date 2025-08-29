#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "system_config.h"
#include "data_types.h"

// Function Declarations
bool uart_protocol_init(uint32_t baudrate);
void uart_protocol_process(void);
bool uart_send_measurement_frame(const uwb_measurement_frame_t* frame);
bool uart_send_raw_data(const uint8_t* data, uint16_t length);

// Frame Processing
uint16_t calculate_checksum(const uint8_t* data, uint16_t length);
bool verify_checksum(const uint8_t* data, uint16_t length, uint16_t expected_checksum);
void uart_frame_error_handler(uint8_t error_code);

// Statistics and Monitoring
uint32_t get_uart_frames_sent(void);
uint32_t get_uart_errors(void);
bool is_uart_ready(void);
void clear_uart_stats(void);

// Buffer Management
bool uart_buffer_available(void);
uint16_t uart_get_buffer_usage(void);
void uart_flush_buffers(void);

#endif // UART_PROTOCOL_H
