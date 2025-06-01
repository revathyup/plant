/**
 * @file main.c
 * @brief Main program for the sensor node
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "tsl2591.h"
#include "mcp9700.h"
#include "stemma_soil.h"
#include "uart_protocol.h"
#include "../protocol.h"
#include "bme680.h"

// Pin definitions
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define TEMP_SENSOR_PIN 26  // ADC0
#define LED_PIN 25          // Onboard LED
#define UART_TX_PIN 8
#define UART_RX_PIN 9
#define BME680_SDA_PIN 2    // Groove 2 SDA
#define BME680_SCL_PIN 3    // Groove 2 SCL

// Sensor thresholds
#define LIGHT_HIGH_THRESHOLD 1000.0f    // lux
#define LIGHT_LOW_THRESHOLD 50.0f       // lux
#define TEMP_HIGH_THRESHOLD 30.0f       // °C
#define TEMP_LOW_THRESHOLD 10.0f        // °C
#define SOIL_DRY_THRESHOLD 300          // Scaled value 0-1000
#define SOIL_WET_THRESHOLD 700          // Scaled value 0-1000

// Global variables
static const uint I2C_FREQ = 100 * 1000;  // 100 KHz
static const uint UART_BAUD = 115200;     // 115200 baud
static i2c_inst_t *i2c = i2c0;
static i2c_inst_t *bme680_i2c = i2c1;     // Second I2C for BME680
static uart_inst_t *uart = uart1;
static uint8_t led_state = 0;

int main() {
    // Initialize stdio first
    stdio_init_all();
    printf("\n\n=== Sensor Node Starting ===\n");
    
    // Initialize hardware
    setup_hardware();
    printf("Hardware initialized\n");
    
    // Initialize UART protocol (this will handle UART initialization)
    if (!uart_protocol_init(uart, UART_BAUD, UART_TX_PIN, UART_RX_PIN)) {
        printf("Failed to initialize UART protocol!\n");
        return -1;
    }
    printf("UART protocol initialized\n");

    while (true) {
        printf("\nReading sensors...\n");
        read_and_process_sensors();
        process_uart_commands();
        printf("Sleeping for 1 second...\n");
        sleep_ms(1000);
    }
}

void setup_hardware(void) {
    printf("Setting up hardware...\n");
    
    // Initialize onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    printf("LED initialized on pin %d\n", LED_PIN);
    
    // Initialize primary I2C bus
    i2c_init(i2c, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C0 initialized on pins SDA:%d SCL:%d at %d Hz\n", 
           I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    
    // Initialize secondary I2C bus for BME680
    i2c_init(bme680_i2c, I2C_FREQ);
    gpio_set_function(BME680_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BME680_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BME680_SDA_PIN);
    gpio_pull_up(BME680_SCL_PIN);
    printf("I2C1 initialized on pins SDA:%d SCL:%d at %d Hz\n",
           BME680_SDA_PIN, BME680_SCL_PIN, I2C_FREQ);
    
    // Initialize ADC for temperature sensor
    adc_init();
    adc_gpio_init(TEMP_SENSOR_PIN);
    printf("ADC initialized on pin %d\n", TEMP_SENSOR_PIN);
    
    printf("Hardware setup complete\n");
}

void read_and_process_sensors(void) {
    float humidity;
    printf("Attempting to read BME680...\n");
    if (bme680_sample_fetch(&humidity)) {
        printf("BME680 humidity: %.1f%%\n", humidity);

        // Prepare BME680 data structure
        bme680_data_t bme680_data = {
            .sensor_type = SENSOR_BME680,
            .humidity = humidity
        };

        // Send BME680 data over UART
        printf("Sending BME680 data over UART...\n");
        uart_protocol_send_bme680_data(&bme680_data);
        printf("BME680 data sent\n");
    } else {
        printf("Failed to read from BME680 sensor\n");
    }
}

void process_uart_commands(void) {
    if (uart_protocol_data_available()) {
        protocol_packet_t packet;
        
        if (uart_protocol_receive_packet(&packet)) {
            switch (packet.command) {
                case CMD_SENSOR_CONFIG:
                    printf("Received sensor configuration command\n");
                    break;
                    
                case CMD_PING:
                    printf("Received ping, sending ACK\n");
                    // TODO: Implement ACK response
                    break;
                    
                default:
                    printf("Received unknown command: 0x%02X\n", packet.command);
                    break;
            }
        }
    }
}
