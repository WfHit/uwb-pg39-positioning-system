#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// System Configuration
#define MAX_ANCHORS_PER_ZONE        15
#define MAX_TOTAL_ANCHORS          100
#define MAX_SIMULTANEOUS_MEASUREMENTS 6
#define DISCOVERY_INTERVAL_MS      2000
#define MEASUREMENT_RATE_HZ        5
#define RSSI_THRESHOLD            -80
#define MIN_ANCHORS_FOR_POSITIONING 3
#define ANCHOR_TIMEOUT_MS         10000
#define MAX_ZONES                 20

// Frame Headers
#define DISCOVERY_REQUEST_HEADER   0xD0
#define DISCOVERY_RESPONSE_HEADER  0xD1
#define RANGING_REQUEST_HEADER     0xR0
#define RANGING_RESPONSE_HEADER    0xR1
#define MEASUREMENT_FRAME_HEADER   0xAA55

// Device Modes
#define DEVICE_MODE_TAG            0
#define DEVICE_MODE_ANCHOR         1

// Ranging Algorithms
#define RANGING_DS_TWR             0
#define RANGING_HDS_TWR            1

// Communication Settings
#define UART_BAUDRATE             115200
#define UART_BUFFER_SIZE          512

// Timing Constants
#define MEASUREMENT_INTERVAL_MS    (1000 / MEASUREMENT_RATE_HZ)
#define DISCOVERY_TIMEOUT_MS       100
#define RANGING_TIMEOUT_MS         50

// Signal Quality Thresholds
#define SIGNAL_QUALITY_EXCELLENT   90
#define SIGNAL_QUALITY_GOOD        70
#define SIGNAL_QUALITY_FAIR        50
#define SIGNAL_QUALITY_POOR        30

// Error Codes
#define ERROR_NONE                 0
#define ERROR_TIMEOUT              1
#define ERROR_INVALID_DATA         2
#define ERROR_NO_ANCHORS           3
#define ERROR_INSUFFICIENT_ANCHORS 4
#define ERROR_COMMUNICATION        5

#endif // SYSTEM_CONFIG_H
