/**
 * @file uart_protocol.c
 * @brief UART protocol implementation for sensor node
 */
     
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "uart_protocol.h"
#include "protocol.h"

// Static variables
static uart_inst_t *uart_instance;

bool uart_protocol_init(uart_inst_t *uart_id, uint baud_rate, uint tx_pin, uint rx_pin) {
    printf("Initializing UART protocol...\n");
    
    uart_instance = uart_id;
    
    // Initialize UART only if not already initialized
    if (!uart_is_enabled(uart_instance)) {
        uart_init(uart_instance, baud_rate);
    }
    
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_pull_up(rx_pin);  // Enable pull-up on RX pin
    
    // Enable FIFO
    uart_set_fifo_enabled(uart_instance, true);
    
    printf("UART protocol initialized on pins TX:%d RX:%d at %d baud\n", 
           tx_pin, rx_pin, baud_rate);
    
    // Verify UART is working
    printf("Testing UART communication...\n");
    uart_puts(uart_instance, "UART_TEST\n");
    
    return true;
}

uint8_t uart_protocol_calculate_checksum(protocol_packet_t *packet) {
    uint8_t checksum = 0;
    checksum ^= packet->start;
    checksum ^= packet->command;
    checksum ^= packet->length;
    
    for (int i = 0; i < packet->length; i++) {
        checksum ^= packet->data[i];
    }
    
    return checksum;
}

bool uart_protocol_validate_packet(protocol_packet_t *packet) {
    if (packet->start != PROTOCOL_START_BYTE) {
        return false;
    }
    
    if (packet->length > PROTOCOL_MAX_DATA_LENGTH) {
        return false;
    }
    
    uint8_t calculated_checksum = uart_protocol_calculate_checksum(packet);
    if (calculated_checksum != packet->checksum) {
        return false;
    }
    
    return true;
}

static bool uart_protocol_send_packet(protocol_packet_t *packet) {
    if (!uart_is_writable(uart_instance)) {
        printf("UART not ready for writing\n");
        return false;
    }
    
    packet->checksum = uart_protocol_calculate_checksum(packet);
    
    // Send packet with timeout
    absolute_time_t timeout = make_timeout_time_ms(100);
    
    if (!uart_is_writable(uart_instance)) {
        if (time_reached(timeout)) {
            printf("UART write timeout\n");
            return false;
        }
    }
    uart_putc(uart_instance, packet->start);
    
    uart_putc(uart_instance, packet->command);
    uart_putc(uart_instance, packet->length);
    
    for (int i = 0; i < packet->length; i++) {
        uart_putc(uart_instance, packet->data[i]);
    }
    
    uart_putc(uart_instance, packet->checksum);
    
    return true;
}

bool uart_protocol_send_light_data(light_data_t *data) {
    protocol_packet_t packet = {
        .start = PROTOCOL_START_BYTE,
        .command = CMD_SENSOR_DATA,
        .length = 11  // 1 byte type + 6 bytes readings + 4 bytes lux
    };
    
    packet.data[0] = data->sensor_type;
    packet.data[1] = (data->full >> 8) & 0xFF;
    packet.data[2] = data->full & 0xFF;
    packet.data[3] = (data->ir >> 8) & 0xFF;
    packet.data[4] = data->ir & 0xFF;
    packet.data[5] = (data->visible >> 8) & 0xFF;
    packet.data[6] = data->visible & 0xFF;
    
    float *lux_ptr = (float*)&packet.data[7];
    *lux_ptr = data->lux;
    
    return uart_protocol_send_packet(&packet);
}

bool uart_protocol_send_alert(uint8_t alert_type, float value, float threshold) {
    protocol_packet_t packet = {
        .start = PROTOCOL_START_BYTE,
        .command = CMD_ALERT,
        .length = 9  // 1 byte alert type + 4 bytes value + 4 bytes threshold
    };
    
    packet.data[0] = alert_type;
    float *value_ptr = (float*)&packet.data[1];
    *value_ptr = value;
    float *threshold_ptr = (float*)&packet.data[5];
    *threshold_ptr = threshold;
    
    return uart_protocol_send_packet(&packet);
}

bool uart_protocol_data_available(void) {
    return uart_is_readable(uart_instance);
}

bool uart_protocol_receive_packet(protocol_packet_t *packet) {
    if (!uart_is_readable(uart_instance)) {
        return false;
    }
    
    // Wait for start byte with timeout
    absolute_time_t timeout = make_timeout_time_ms(100);
    while (uart_is_readable(uart_instance)) {
        uint8_t byte = uart_getc(uart_instance);
        if (byte == PROTOCOL_START_BYTE) {
            packet->start = byte;
            break;
        }
        if (time_reached(timeout)) {
            return false;
        }
    }
    
    // Read remaining packet with timeout checks
    timeout = make_timeout_time_ms(100);
    
    if (!uart_is_readable_within_us(uart_instance, 100000)) return false;
    packet->command = uart_getc(uart_instance);
    
    if (!uart_is_readable_within_us(uart_instance, 100000)) return false;
    packet->length = uart_getc(uart_instance);
    
    if (packet->length > PROTOCOL_MAX_DATA_LENGTH) return false;
    
    for (int i = 0; i < packet->length; i++) {
        if (!uart_is_readable_within_us(uart_instance, 100000)) return false;
        packet->data[i] = uart_getc(uart_instance);
    }
    
    if (!uart_is_readable_within_us(uart_instance, 100000)) return false;
    packet->checksum = uart_getc(uart_instance);
    
    return uart_protocol_validate_packet(packet);
}

void uart_protocol_send_bme680_data(const bme680_data_t *data) {
    protocol_packet_t packet = {
        .start = PROTOCOL_START_BYTE,
        .command = CMD_SENSOR_DATA,
        .length = 5  // 1 byte type + 4 bytes float
    };
    
    packet.data[0] = data->sensor_type;
    memcpy(&packet.data[1], &data->humidity, sizeof(float));
    packet.checksum = uart_protocol_calculate_checksum(&packet);
    
    if (!uart_protocol_send_packet(&packet)) {
        printf("Failed to send BME680 data\n");
    }
}
