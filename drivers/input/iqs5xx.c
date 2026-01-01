/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <stdlib.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "iqs5xx.h"

LOG_MODULE_REGISTER(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

static int iqs5xx_read_reg16(const struct device *dev, uint16_t reg, uint16_t *val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[2];
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;

    ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    *val = (buf[0] << 8) | buf[1];
    return 0;
}

static int iqs5xx_write_reg16(const struct device *dev, uint16_t reg, uint16_t val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_read_reg8(const struct device *dev, uint16_t reg, uint8_t *val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};

    return i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), val, 1);
}

static int iqs5xx_write_reg8(const struct device *dev, uint16_t reg, uint8_t val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_end_comm_window(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {IQS5XX_END_COMM_WINDOW >> 8, IQS5XX_END_COMM_WINDOW & 0xFF, 0x00};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static void iqs5xx_button_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, button_release_work);

    // TODO: This loop should only deactivate one button.
    // Log a warning when that is not the case.
    for (int i = 0; i < 3; i++) {
        LOG_INF("Releasing synthetic button");
        if (data->buttons_pressed & BIT(i)) {
            input_report_key(data->dev, INPUT_BTN_0 + i, 0, true, K_NO_WAIT);
            // Turn off the bit.
            // NOTE: This is a potential race.
            data->buttons_pressed &= ~BIT(i);
        }
    }
}

static void iqs5xx_work_handler(struct k_work *work) {
    struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    uint8_t sys_info_0, sys_info_1, gesture_events_0, gesture_events_1, num_fingers;
    int ret;
    static uint32_t success_count = 0;
    static uint32_t fail_count = 0;

    // Read system info registers.
    ret = iqs5xx_read_reg8(dev, IQS5XX_SYSTEM_INFO_0, &sys_info_0);
    if (ret < 0) {
        fail_count++;
        if (fail_count % 20 == 1) {  // Print every 20th failure (once per second)
            printk("*** IQS5XX: Read FAILED (total failures: %u, successes: %u)\n",
                   fail_count, success_count);
        }
        LOG_ERR("Failed to read system info 0: %d", ret);
        goto end_comm;
    }

    success_count++;
    // Only print first success and then every 100th to avoid flooding console
    if (success_count == 1 || success_count % 100 == 0) {
        printk("*** IQS5XX: Read SUCCESS! sys_info_0=0x%02x (total: %u)\n",
               sys_info_0, success_count);
    }

    ret = iqs5xx_read_reg8(dev, IQS5XX_SYSTEM_INFO_1, &sys_info_1);
    if (ret < 0) {
        LOG_ERR("Failed to read system info 1: %d", ret);
        goto end_comm;
    }

    ret = iqs5xx_read_reg8(dev, IQS5XX_GESTURE_EVENTS_0, &gesture_events_0);
    if (ret < 0) {
        LOG_ERR("Failed to read gesture events: %d", ret);
        goto end_comm;
    }

    ret = iqs5xx_read_reg8(dev, IQS5XX_GESTURE_EVENTS_1, &gesture_events_1);
    if (ret < 0) {
        LOG_ERR("Failed to read gesture events 1: %d", ret);
        goto end_comm;
    }

    // Handle reset indication.
    if (sys_info_0 & IQS5XX_SHOW_RESET) {
        LOG_INF("Device reset detected");
        // Acknowledge reset.
        iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);
        goto end_comm;
    }

    bool tp_movement = (sys_info_1 & IQS5XX_TP_MOVEMENT) != 0;
    bool scroll = (gesture_events_1 & IQS5XX_SCROLL) != 0;
    if (!scroll) {
        // Clear accumulators if we're not actively scrolling.
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
    }

    uint16_t button_code;
    bool button_pressed = false;
    if (gesture_events_0 & IQS5XX_SINGLE_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_0;
    } else if (gesture_events_1 & IQS5XX_TWO_FINGER_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_1;
    }

    bool hold_became_active = (gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && !data->active_hold;
    bool hold_released = !(gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && data->active_hold;

    int16_t rel_x, rel_y;
    if (tp_movement || scroll) {
        ret = iqs5xx_read_reg16(dev, IQS5XX_REL_X, (uint16_t *)&rel_x);
        if (ret < 0) {
            LOG_ERR("Failed to read relative X: %d", ret);
            goto end_comm;
        }

        ret = iqs5xx_read_reg16(dev, IQS5XX_REL_Y, (uint16_t *)&rel_y);
        if (ret < 0) {
            LOG_ERR("Failed to read relative Y: %d", ret);
            goto end_comm;
        }
    }

    // Handle movement and gestures.
    //
    // Each one of these branches needs to send the last report it makes as
    // sync to ensure that the input subsystem processes things in order.
    if (hold_became_active) {
        LOG_INF("Hold became active");
        input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_NO_WAIT);
        data->active_hold = true;
    } else if (hold_released) {
        LOG_INF("Hold became inactive");
        input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_NO_WAIT);
        data->active_hold = false;
    } else if (button_pressed) {
        // Cancel any pending release.
        k_work_cancel_delayable(&data->button_release_work);

        // Press the button immediately.
        input_report_key(dev, button_code, 1, true, K_NO_WAIT);
        data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);

        // Schedule release after 100ms.
        k_work_schedule(&data->button_release_work, K_MSEC(100));
    } else if (scroll) {
        // TODO: Expose this divisor.
        int16_t scroll_div = 32;

        // Only one scrolling direction is valid at a time.
        // End the communication right after reporting the movement.
        if (rel_x != 0) {
            // By default the x axis is already "natural".
            if (!config->natural_scroll_x) {
                rel_x *= -1;
            }
            data->scroll_x_acc += rel_x;
            if (abs(data->scroll_x_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div, true,
                                K_NO_WAIT);
                data->scroll_x_acc %= scroll_div;
            }
            goto end_comm;
        }
        if (rel_y != 0) {
            if (config->natural_scroll_y) {
                rel_y *= -1;
            }
            data->scroll_y_acc += rel_y;
            if (abs(data->scroll_y_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_WHEEL, data->scroll_y_acc / scroll_div, true,
                                 K_NO_WAIT);
                data->scroll_y_acc %= scroll_div;
            }

            goto end_comm;
        }
    } else if (tp_movement) {
        ret = iqs5xx_read_reg8(dev, IQS5XX_NUM_FINGERS, &num_fingers);
        if (ret < 0) {
            LOG_ERR("Failed to read number of fingers: %d", ret);
            goto end_comm;
        }

        if (rel_x != 0 || rel_y != 0) {
            // Apply axis flipping if configured
            if (config->flip_x) {
                rel_x *= -1;
            }
            if (config->flip_y) {
                rel_y *= -1;
            }

            printk("*** IQS5XX: About to report movement X=%d Y=%d\n", rel_x, rel_y);

            // Use K_NO_WAIT to avoid deadlock if HID queue is full
            ret = input_report_rel(dev, INPUT_REL_X, rel_x, false, K_NO_WAIT);
            printk("*** IQS5XX: After report X, ret=%d\n", ret);

            if (ret < 0) {
                LOG_WRN("Failed to report X movement: %d (queue full?)", ret);
            }

            ret = input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_NO_WAIT);
            printk("*** IQS5XX: After report Y, ret=%d\n", ret);

            if (ret < 0) {
                LOG_WRN("Failed to report Y movement: %d (queue full?)", ret);
            }

            printk("*** IQS5XX: Movement reporting complete\n");
        }
    }

    printk("*** IQS5XX: After movement block, about to end_comm\n");

end_comm:
    printk("*** IQS5XX: At end_comm label, calling end_comm_window\n");
    // End communication window.
    iqs5xx_end_comm_window(dev);
    printk("*** IQS5XX: After end_comm_window returned\n");

    // Debug: ALWAYS print to confirm work handler returns
    static uint32_t work_count = 0;
    work_count++;
    printk("*** IQS5XX: Work handler EXITING (count: %u)\n", work_count);
}

static void iqs5xx_rdy_handler(const struct device *port, struct gpio_callback *cb,
                               gpio_port_pins_t pins) {
    struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, rdy_cb);

    k_work_submit(&data->work);
}

static void iqs5xx_poll_timer_handler(struct k_timer *timer) {
    struct iqs5xx_data *data = CONTAINER_OF(timer, struct iqs5xx_data, poll_timer);
    static uint32_t timer_count = 0;

    timer_count++;
    // ALWAYS print to see if timer keeps firing
    printk("*** IQS5XX: Timer FIRE (count: %u)\n", timer_count);

    k_work_submit(&data->work);
}

static int iqs5xx_setup_device(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_data *data = dev->data;
    int ret;
    uint8_t value;

    printk("*** IQS5XX: setup_device - QMK-style init\n");
    LOG_INF("Starting QMK-style device setup");

    // Step 1: Disable idle timeout to prevent low power modes (LP1/LP2)
    // This is CRITICAL - without this, trackpad goes to sleep!
    // Retry up to 3 times since device may not be fully awake yet
    value = 255;  // No timeout
    int retries = 3;
    for (int i = 0; i < retries; i++) {
        ret = iqs5xx_write_reg8(dev, IQS5XX_IDLE_MODE_TIMEOUT, value);
        if (ret == 0) break;

        printk("*** IQS5XX: Idle timeout write attempt %d/%d failed: %d\n", i+1, retries, ret);
        if (i < retries - 1) {
            k_msleep(100);  // Wait before retry
        }
    }
    if (ret < 0) {
        printk("*** IQS5XX: Failed to disable idle timeout after %d attempts: %d\n", retries, ret);
        LOG_ERR("Failed to disable idle timeout: %d", ret);
        return ret;
    }
    printk("*** IQS5XX: Disabled idle timeout (no LP modes)\n");
    LOG_INF("Disabled idle timeout");

    // Step 2: Enable REATI for continuous touch detection
    uint8_t config0_value = IQS5XX_SETUP_COMPLETE | IQS5XX_REATI | IQS5XX_MANUAL_CONTROL;
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_0, config0_value);
    if (ret < 0) {
        printk("*** IQS5XX: Failed to set system config 0: %d\n", ret);
        LOG_ERR("Failed to set system config 0: %d", ret);
        return ret;
    }
    printk("*** IQS5XX: Set SYSTEM_CONFIG_0: REATI + MANUAL_CONTROL\n");
    LOG_INF("Configured system config 0");

    // Step 3: Configure for streaming mode (not event mode) for polling
    uint8_t config1_value = IQS5XX_TP_EVENT | IQS5XX_GESTURE_EVENT | IQS5XX_REATI_EVENT;
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_1, config1_value);
    if (ret < 0) {
        printk("*** IQS5XX: Failed to set system config 1: %d\n", ret);
        LOG_ERR("Failed to set system config 1: %d", ret);
        return ret;
    }
    printk("*** IQS5XX: Set SYSTEM_CONFIG_1: streaming mode\n");
    LOG_INF("Configured system config 1");

    // End communication window
    ret = iqs5xx_end_comm_window(dev);
    if (ret < 0) {
        printk("*** IQS5XX: Failed to end comm window: %d\n", ret);
        LOG_ERR("Failed to end comm window: %d", ret);
        return ret;
    }

    printk("*** IQS5XX: setup_device complete - QMK-style init done\n");
    LOG_INF("Device setup complete");
    return 0;
}

static int iqs5xx_init(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_data *data = dev->data;
    int ret;

    printk("\n\n*** IQS5XX INIT STARTING ***\n");
    LOG_ERR("=== IQS5XX INIT STARTING ===");

    if (!i2c_is_ready_dt(&config->i2c)) {
        printk("*** IQS5XX: I2C device not ready\n");
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    printk("*** IQS5XX: I2C device is ready\n");
    LOG_ERR("I2C device is ready");

    data->dev = dev;
    k_work_init(&data->work, iqs5xx_work_handler);
    k_work_init_delayable(&data->button_release_work, iqs5xx_button_release_work_handler);

    // Configure reset GPIO if available.
    if (config->reset_gpio.port) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }

        // Reset the device.
        gpio_pin_set_dt(&config->reset_gpio, 1);
        k_msleep(1);
        gpio_pin_set_dt(&config->reset_gpio, 0);
        k_msleep(10);
    }

    // Configure RDY GPIO if available, otherwise use polling mode.
    if (config->rdy_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&config->rdy_gpio)) {
            LOG_ERR("RDY GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure RDY GPIO: %d", ret);
            return ret;
        }

        gpio_init_callback(&data->rdy_cb, iqs5xx_rdy_handler, BIT(config->rdy_gpio.pin));
        ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add RDY callback: %d", ret);
            return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_RISING);
        if (ret < 0) {
            LOG_ERR("Failed to configure RDY interrupt: %d", ret);
            return ret;
        }

        data->use_polling = false;
        LOG_INF("IQS5xx using interrupt mode (RDY GPIO)");
    } else {
        // No RDY GPIO available, use polling mode
        // Use 50ms interval (20Hz) to reduce I2C bus congestion
        printk("*** IQS5XX: No RDY GPIO, entering polling mode\n");
        k_timer_init(&data->poll_timer, iqs5xx_poll_timer_handler, NULL);
        k_timer_start(&data->poll_timer, K_MSEC(50), K_MSEC(50)); // Poll every 50ms
        data->use_polling = true;
        LOG_INF("IQS5xx using polling mode (no RDY GPIO available, 20Hz)");
    }

    // Extended initialization delay for devices without hardware reset
    // The IQS5XX needs time to complete internal calibration after power-on
    // TPS43 takes ~2 seconds to become responsive based on testing
    if (!config->reset_gpio.port) {
        printk("*** IQS5XX: No reset GPIO, waiting 2.5s for device wake-up\n");
        LOG_INF("No reset GPIO, using extended power-on delay (2.5s)");
        k_msleep(2500);  // Wait for device to become fully responsive
    } else {
        k_msleep(100);
    }

    // Attempt to wake device from any potential sleep mode
    // Send a dummy write to wake up the I2C interface
    printk("*** IQS5XX: Sending wake command\n");
    uint8_t wake_cmd[3] = {0x00, 0x00, 0x00};
    i2c_write_dt(&config->i2c, wake_cmd, sizeof(wake_cmd));
    k_msleep(50);  // Allow wake-up to complete

    // Setup device configuration.
    printk("*** IQS5XX: About to call setup_device\n");
    LOG_ERR("About to call iqs5xx_setup_device");
    ret = iqs5xx_setup_device(dev);
    if (ret < 0) {
        printk("*** IQS5XX: Setup device FAILED: %d - continuing anyway\n", ret);
        LOG_WRN("Failed to setup device: %d - device may work with defaults", ret);
        // Don't return error - device works with defaults, config will apply later
    } else {
        printk("*** IQS5XX: Setup device SUCCESS\n");
        LOG_INF("Device configuration applied successfully");
    }

    data->initialized = true;
    printk("*** IQS5XX INIT COMPLETE (polling active) ***\n\n");
    LOG_ERR("=== IQS5XX INIT COMPLETE ===");

    return 0;
}

// Replace CONFIG_INPUT_INIT_PRIORITY with the azoteq specific value.
#define IQS5XX_INIT(n)                                                                             \
    static struct iqs5xx_data iqs5xx_data_##n;                                                     \
    static const struct iqs5xx_config iqs5xx_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET_OR(n, rdy_gpios, {0}),                                   \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                               \
        .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                                         \
        .press_and_hold = DT_INST_PROP(n, press_and_hold),                                         \
        .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                                         \
        .scroll = DT_INST_PROP(n, scroll),                                                         \
        .natural_scroll_x = DT_INST_PROP(n, natural_scroll_x),                                     \
        .natural_scroll_y = DT_INST_PROP(n, natural_scroll_y),                                     \
        .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),                       \
        .switch_xy = DT_INST_PROP(n, switch_xy),                                                   \
        .flip_x = DT_INST_PROP(n, flip_x),                                                         \
        .flip_y = DT_INST_PROP(n, flip_y),                                                         \
        .bottom_beta = DT_INST_PROP_OR(n, bottom_beta, 5),                                         \
        .stationary_threshold = DT_INST_PROP_OR(n, stationary_threshold, 5),                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, NULL, &iqs5xx_data_##n, &iqs5xx_config_##n, POST_KERNEL, \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);                                              \
    BUILD_ASSERT(1, "IQS5XX device " #n " instantiated");

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)

// Compile-time check: ensure at least one IQS5XX device exists
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "NO IQS5XX DEVICES FOUND IN DEVICE TREE!"
#endif
