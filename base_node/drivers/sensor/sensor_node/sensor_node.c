/**
 * @file sensor_node.c
 * @brief Sensor driver for communicating with sensor node via UART
 */

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>



LOG_MODULE_REGISTER(sensor_node, CONFIG_SENSOR_LOG_LEVEL);

/* Protocol definitions */
#define PROTOCOL_START_BYTE     0xAA
#define PROTOCOL_MAX_DATA_LEN   64
#define PROTOCOL_TIMEOUT_MS     500

/* Command types */
#define CMD_SENSOR_DATA     0x01
#define CMD_ALERT           0x02
#define CMD_PING            0x03
#define CMD_ACK             0x04

/* Sensor types */
#define SENSOR_LIGHT        0x01
#define SENSOR_TEMP         0x02
#define SENSOR_BME680       0x03

/* Buffer sizes */
#define RX_BUFFER_SIZE      256
#define TX_BUFFER_SIZE      128

#define DT_DRV_COMPAT zephyr_sensor_node


/* Custom channels */
enum sensor_node_channels {
    SENSOR_NODE_CHAN_GAS_RESISTANCE = SENSOR_CHAN_PRIV_START
};

/* Packet structure */
typedef struct {
    uint8_t start;
    uint8_t command;
    uint8_t length;
    uint8_t data[PROTOCOL_MAX_DATA_LEN];
    uint8_t checksum;
} packet_t;

/* Driver data */
struct sensor_node_data {
    struct sensor_value temperature;
    struct sensor_value humidity;
    struct sensor_value pressure;
    struct sensor_value light;
    struct sensor_value gas_resistance;
    bool data_ready;
    
    const struct device *uart_dev;
    struct ring_buf rx_rb;
    struct ring_buf tx_rb;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t tx_buffer[TX_BUFFER_SIZE];
    
    struct k_mutex mutex;
    uint32_t rx_bytes;
    uint32_t valid_packets;
    uint32_t packet_errors;
};

/* Calculate checksum */
static uint8_t calculate_checksum(const packet_t *packet)
{
    uint8_t checksum = packet->start ^ packet->command ^ packet->length;
    for (int i = 0; i < packet->length; i++) {
        checksum ^= packet->data[i];
    }
    return checksum;
}

/* UART Interrupt Handler */
static void uart_callback(const struct device *dev, void *user_data)
{
    struct sensor_node_data *data = user_data;
    uint8_t c;

    if (!uart_irq_update(dev)) {
        return;
    }

    /* Handle RX */
    if (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) == 1) {
            if (ring_buf_put(&data->rx_rb, &c, 1) != 1) {
                LOG_WRN("RX buffer full, dropping byte");
            } else {
                data->rx_bytes++;
                LOG_DBG("RX: 0x%02x (%c)", c, (c >= 32 && c <= 126) ? c : '.');
            }
        }
    }

    /* Handle TX */
    if (uart_irq_tx_ready(dev)) {
        uint8_t tx_byte;
        if (ring_buf_get(&data->tx_rb, &tx_byte, 1) == 1) {
            uart_fifo_fill(dev, &tx_byte, 1);
        } else {
            uart_irq_tx_disable(dev);
        }
    }
}

/* Send a byte via UART */
static void uart_write_byte(struct sensor_node_data *data, uint8_t byte)
{
    if (ring_buf_put(&data->tx_rb, &byte, 1) == 1) {
        uart_irq_tx_enable(data->uart_dev);
    } else {
        LOG_WRN("TX buffer full, cannot send byte");
    }
}

/* Process received sensor data */
static void process_sensor_data(struct sensor_node_data *data, const packet_t *packet)
{
    k_mutex_lock(&data->mutex, K_FOREVER);

    switch (packet->data[0]) {
    case SENSOR_LIGHT:
        if (packet->length >= 11) {
            float lux;
            memcpy(&lux, &packet->data[7], sizeof(float));
            sensor_value_from_double(&data->light, lux);
            LOG_INF("Light: %.2f lux", sensor_value_to_double(&data->light));
        }
        break;
        
    case SENSOR_TEMP:
        if (packet->length >= 5) {
            float temp;
            memcpy(&temp, &packet->data[1], sizeof(float));
            sensor_value_from_double(&data->temperature, temp);
            LOG_INF("Temp: %.1f C", sensor_value_to_double(&data->temperature));
        }
        break;
        
    case SENSOR_BME680:
        if (packet->length >= 5) {
            float humidity;
            memcpy(&humidity, &packet->data[1], sizeof(float));
            sensor_value_from_double(&data->humidity, humidity);
            LOG_INF("Humidity: %.1f%%", sensor_value_to_double(&data->humidity));
        }
        break;
        
    default:
        LOG_ERR("Unknown sensor type: 0x%02x", packet->data[0]);
        break;
    }

    data->data_ready = true;
    k_mutex_unlock(&data->mutex);
}

/* Read and validate packet */
static bool read_packet(struct sensor_node_data *data, packet_t *packet)
{
    uint8_t byte;
    
    /* Find start byte */
    while (ring_buf_get(&data->rx_rb, &byte, 1) == 1) {
        if (byte == PROTOCOL_START_BYTE) {
            packet->start = byte;
            break;
        }
        LOG_DBG("Skipping byte: 0x%02x", byte);
    }

    /* Check remaining packet */
    size_t needed = 3 + PROTOCOL_MAX_DATA_LEN + 1; // cmd + len + max data + checksum
    if (ring_buf_size_get(&data->rx_rb) < needed) {
        return false;
    }

    /* Read packet */
    ring_buf_get(&data->rx_rb, &packet->command, 1);
    ring_buf_get(&data->rx_rb, &packet->length, 1);
    
    if (packet->length > PROTOCOL_MAX_DATA_LEN) {
        LOG_ERR("Invalid length: %d", packet->length);
        return false;
    }
    
    ring_buf_get(&data->rx_rb, packet->data, packet->length);
    ring_buf_get(&data->rx_rb, &packet->checksum, 1);

    /* Validate checksum */
    if (calculate_checksum(packet) != packet->checksum) {
        LOG_ERR("Checksum mismatch");
        return false;
    }

    LOG_DBG("Valid packet: cmd=0x%02x len=%d", packet->command, packet->length);
    data->valid_packets++;
    return true;
}

/* Sensor API: Fetch data */
static int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct sensor_node_data *data = dev->data;
    int64_t end_time = k_uptime_get() + PROTOCOL_TIMEOUT_MS;
    packet_t packet;

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->data_ready = false;
    k_mutex_unlock(&data->mutex);

    LOG_DBG("Waiting for sensor data...");
    
    while (k_uptime_get() < end_time) {
        if (read_packet(data, &packet)) {
            if (packet.command == CMD_SENSOR_DATA) {
                process_sensor_data(data, &packet);
                return 0;
            }
        }
        k_sleep(K_MSEC(10));
    }

    LOG_ERR("Timeout waiting for sensor data");
    return -ETIMEDOUT;
}

/* Sensor API: Get channel data */
static int sensor_node_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{
    struct sensor_node_data *data = dev->data;
    int ret = -ENODATA;

    k_mutex_lock(&data->mutex, K_FOREVER);
    
    if (data->data_ready) {
        switch (chan) {
        case SENSOR_CHAN_AMBIENT_TEMP:
            *val = data->temperature;
            ret = 0;
            break;
        case SENSOR_CHAN_HUMIDITY:
            *val = data->humidity;
            ret = 0;
            break;
        case SENSOR_CHAN_LIGHT:
            *val = data->light;
            ret = 0;
            break;
        case SENSOR_NODE_CHAN_GAS_RESISTANCE:
            *val = data->gas_resistance;
            ret = 0;
            break;
        default:
            ret = -ENOTSUP;
        }
    }

    k_mutex_unlock(&data->mutex);
    return ret;
}

/* Driver initialization */
static int sensor_node_init(const struct device *dev)
{
    struct sensor_node_data *data = dev->data;
    struct uart_config uart_cfg;

    /* Initialize data */
    k_mutex_init(&data->mutex);
    ring_buf_init(&data->rx_rb, RX_BUFFER_SIZE, data->rx_buffer);
    ring_buf_init(&data->tx_rb, TX_BUFFER_SIZE, data->tx_buffer);
    data->rx_bytes = 0;
    data->valid_packets = 0;
    data->packet_errors = 0;

    /* Get UART device */
    data->uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(data->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    /* Configure UART */
    if (uart_config_get(data->uart_dev, &uart_cfg) == 0) {
        LOG_INF("UART configured at %d baud", uart_cfg.baudrate);
    }

    /* Setup interrupts */
    uart_irq_callback_user_data_set(data->uart_dev, uart_callback, data);
    uart_irq_rx_enable(data->uart_dev);

    LOG_INF("Sensor node driver initialized");
    LOG_INF("RX buffer: %d bytes", RX_BUFFER_SIZE);
    LOG_INF("TX buffer: %d bytes", TX_BUFFER_SIZE);

    return 0;
}

/* Driver definition */
static const struct sensor_driver_api sensor_node_api = {
    .sample_fetch = sensor_node_sample_fetch,
    .channel_get = sensor_node_channel_get,
};

static struct sensor_node_data sensor_node_data_0;

DEVICE_DT_INST_DEFINE(0,
                     sensor_node_init,
                     NULL,
                     &sensor_node_data_0,
                     NULL,
                     POST_KERNEL,
                     CONFIG_SENSOR_INIT_PRIORITY,
                     &sensor_node_api);
