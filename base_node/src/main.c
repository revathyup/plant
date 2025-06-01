/**
 * @file main.c
 * @brief Main application for smart garden base node
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include<math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Custom channels */
enum {
    SENSOR_NODE_CHAN_GAS_RESISTANCE = SENSOR_CHAN_PRIV_START
};

/* Thread settings */
#define SENSOR_THREAD_STACK_SIZE 2048
#define SENSOR_THREAD_PRIORITY   7

K_THREAD_STACK_DEFINE(sensor_thread_stack, SENSOR_THREAD_STACK_SIZE);
static struct k_thread sensor_thread;

/* Print sensor value */
static void print_sensor_value(const struct device *dev,
                             enum sensor_channel chan,
                             const char *name,
                             const char *unit)
{
    struct sensor_value val;
    int ret = sensor_channel_get(dev, chan, &val);

    if (ret == 0) {
        double dval = sensor_value_to_double(&val);
        if (isfinite(dval)) {
            LOG_INF("%-12s: %8.2f %s", name, dval, unit);
        } else {
            LOG_ERR("%s: Invalid value", name);
        }
    } else if (ret == -ENOTSUP) {
        LOG_WRN("%s: Not supported", name);
    } else {
        LOG_ERR("%s: Error %d", name, ret);
    }
}

/* Sensor monitoring thread */
static void sensor_thread_fn(void *dev_ptr, void *unused1, void *unused2)
{
    const struct device *dev = dev_ptr;
    int ret;
    uint32_t count = 0;

    while (1) {
        ret = sensor_sample_fetch(dev);
        if (ret < 0) {
            LOG_ERR("Sample fetch failed: %d", ret);
            k_sleep(K_SECONDS(1));
            continue;
        }

        LOG_INF("\n=== Sensor Readings [%u] ===", ++count);
        print_sensor_value(dev, SENSOR_CHAN_AMBIENT_TEMP, "Temperature", "°C");
        print_sensor_value(dev, SENSOR_CHAN_HUMIDITY, "Humidity", "%");
        print_sensor_value(dev, SENSOR_CHAN_LIGHT, "Light", "lux");
        print_sensor_value(dev, SENSOR_NODE_CHAN_GAS_RESISTANCE, "Gas Res", "Ohm");
        LOG_INF("===========================");

        k_sleep(K_SECONDS(5));
    }
}

/* Main application */
void main(void)
{
    int ret;

    /* Initialize USB */
    ret = usb_enable(NULL);
    if (ret != 0) {
        LOG_ERR("USB init failed: %d", ret);
        return;
    }
    k_sleep(K_SECONDS(1)); // Wait for USB to stabilize

    /* Get sensor device */
    const struct device *sensor = DEVICE_DT_GET(DT_NODELABEL(sensor_node0));
    if (!device_is_ready(sensor)) {
        LOG_ERR("Sensor device not ready");
        return;
    }

    /* Create sensor thread */
    k_thread_create(&sensor_thread,
                   sensor_thread_stack,
                   K_THREAD_STACK_SIZEOF(sensor_thread_stack),
                   sensor_thread_fn,
                   (void *)sensor,
                   NULL,
                   NULL,
                   SENSOR_THREAD_PRIORITY,
                   0,
                   K_NO_WAIT);
    k_thread_name_set(&sensor_thread, "sensor_monitor");

    LOG_INF("Smart Garden Base Node started");
    LOG_INF("Monitoring sensor data every 5 seconds");

    /* Main loop - just sleep */
    while (1) {
        k_sleep(K_SECONDS(60));
        LOG_INF("System operational");
    }
}
