#include "uart_protocol.h"
#include "../Utils/timing_utils.h"
#include <string.h>

// UART Statistics
static uint32_t frames_sent = 0;
static uint32_t uart_errors = 0;
static bool uart_initialized = false;

// Buffer for UART transmission
static uint8_t tx_buffer[UART_BUFFER_SIZE];
static uint16_t tx_buffer_head = 0;
static uint16_t tx_buffer_tail = 0;
static bool tx_buffer_full = false;

// External UART functions (to be implemented in hardware layer)
extern bool hw_uart_init(uint32_t baudrate);
extern bool hw_uart_transmit(const uint8_t* data, uint16_t length);
extern bool hw_uart_is_ready(void);
extern void hw_uart_process(void);

bool uart_protocol_init(uint32_t baudrate)
{
    // Initialize hardware UART
    if(!hw_uart_init(baudrate))
    {
        return false;
    }

    // Initialize buffers
    tx_buffer_head = 0;
    tx_buffer_tail = 0;
    tx_buffer_full = false;

    // Reset statistics
    frames_sent = 0;
    uart_errors = 0;

    uart_initialized = true;
    return true;
}

void uart_protocol_process(void)
{
    if(!uart_initialized) return;

    // Process hardware UART
    hw_uart_process();

    // Process transmit buffer
    if(tx_buffer_head != tx_buffer_tail || tx_buffer_full)
    {
        if(hw_uart_is_ready())
        {
            // Calculate available data
            uint16_t data_length;
            if(tx_buffer_full)
            {
                data_length = UART_BUFFER_SIZE;
            }
            else if(tx_buffer_head > tx_buffer_tail)
            {
                data_length = tx_buffer_head - tx_buffer_tail;
            }
            else
            {
                data_length = UART_BUFFER_SIZE - tx_buffer_tail;
            }

            // Transmit data
            if(hw_uart_transmit(&tx_buffer[tx_buffer_tail], data_length))
            {
                tx_buffer_tail = (tx_buffer_tail + data_length) % UART_BUFFER_SIZE;
                tx_buffer_full = false;
            }
            else
            {
                uart_errors++;
            }
        }
    }
}

bool uart_send_measurement_frame(const uwb_measurement_frame_t* frame)
{
    if(!frame || !uart_initialized) return false;

    // Create a copy with calculated checksum
    uwb_measurement_frame_t frame_copy = *frame;
    frame_copy.checksum = calculate_checksum((uint8_t*)&frame_copy,
                                           sizeof(uwb_measurement_frame_t) - sizeof(uint16_t));

    // Send the frame
    bool success = uart_send_raw_data((uint8_t*)&frame_copy, sizeof(uwb_measurement_frame_t));

    if(success)
    {
        frames_sent++;
    }
    else
    {
        uart_errors++;
    }

    return success;
}

bool uart_send_raw_data(const uint8_t* data, uint16_t length)
{
    if(!data || length == 0 || !uart_initialized) return false;

    // Check if buffer has enough space
    uint16_t available_space;
    if(tx_buffer_full)
    {
        available_space = 0;
    }
    else if(tx_buffer_head >= tx_buffer_tail)
    {
        available_space = UART_BUFFER_SIZE - (tx_buffer_head - tx_buffer_tail) - 1;
    }
    else
    {
        available_space = tx_buffer_tail - tx_buffer_head - 1;
    }

    if(length > available_space)
    {
        uart_errors++;
        return false; // Buffer overflow
    }

    // Copy data to buffer
    for(uint16_t i = 0; i < length; i++)
    {
        tx_buffer[tx_buffer_head] = data[i];
        tx_buffer_head = (tx_buffer_head + 1) % UART_BUFFER_SIZE;

        if(tx_buffer_head == tx_buffer_tail)
        {
            tx_buffer_full = true;
            break;
        }
    }

    return true;
}

uint16_t calculate_checksum(const uint8_t* data, uint16_t length)
{
    if(!data || length == 0) return 0;

    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }

    return crc;
}

bool verify_checksum(const uint8_t* data, uint16_t length, uint16_t expected_checksum)
{
    if(!data || length == 0) return false;

    uint16_t calculated_checksum = calculate_checksum(data, length);
    return (calculated_checksum == expected_checksum);
}

void uart_frame_error_handler(uint8_t error_code)
{
    uart_errors++;

    // Log error or take corrective action based on error_code
    switch(error_code)
    {
        case ERROR_TIMEOUT:
            // Handle timeout
            break;
        case ERROR_INVALID_DATA:
            // Handle invalid data
            break;
        case ERROR_COMMUNICATION:
            // Handle communication error
            break;
        default:
            break;
    }
}

uint32_t get_uart_frames_sent(void)
{
    return frames_sent;
}

uint32_t get_uart_errors(void)
{
    return uart_errors;
}

bool is_uart_ready(void)
{
    return uart_initialized && hw_uart_is_ready();
}

void clear_uart_stats(void)
{
    frames_sent = 0;
    uart_errors = 0;
}

bool uart_buffer_available(void)
{
    return !tx_buffer_full;
}

uint16_t uart_get_buffer_usage(void)
{
    if(tx_buffer_full)
    {
        return UART_BUFFER_SIZE;
    }
    else if(tx_buffer_head >= tx_buffer_tail)
    {
        return tx_buffer_head - tx_buffer_tail;
    }
    else
    {
        return UART_BUFFER_SIZE - (tx_buffer_tail - tx_buffer_head);
    }
}

void uart_flush_buffers(void)
{
    tx_buffer_head = 0;
    tx_buffer_tail = 0;
    tx_buffer_full = false;
}
