/*
 * Copyright (c) 2024 BiologIC Technologies Ltd.
 * Driver for The Lee Company Smart Pump module.
 * Not for Commercial Use
 */

#define DT_DRV_COMPAT zephyr_smartpump

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <../include/zephyr/smartpump.h>

#include <zephyr/drivers/i2c.h>
#include "smartpump_registers.h"

LOG_MODULE_REGISTER(smartpump, CONFIG_SMARTPUMP_LOG_LEVEL);

#define PUMP_MAX_PRESSURE (100000)

struct smartpump_data {
    int state;
    float set_pressure;
};

struct smartpump_config {
    struct i2c_dt_spec i2c;
};


static int pump_i2c_write_int16(const struct device *dev, int register_id, int16_t value_to_write) {
    const struct smartpump_config *config = dev->config;
    uint8_t buf[3];
    uint8_t *ptrToInt;
    ptrToInt = (uint8_t *) &value_to_write;

    buf[0] = register_id;
    buf[1] = ptrToInt[0];
    buf[2] = ptrToInt[1];

    int ret = i2c_write(config->i2c.bus, buf, sizeof(buf), config->i2c.addr);
    if (ret < 0) {
        LOG_ERR("Failed to write to I2C device: %d.\n", ret);
    }
    return ret;
};


static int pump_i2c_write_float(const struct device *dev, int register_id, float value_to_write) {
    const struct smartpump_config *config = dev->config;
    uint8_t buf[5];
    uint8_t *ptrToFloat;
    ptrToFloat = (uint8_t *) &value_to_write;

    buf[0] = register_id;
    buf[1] = ptrToFloat[0];
    buf[2] = ptrToFloat[1];
    buf[3] = ptrToFloat[2];
    buf[4] = ptrToFloat[3];

    int ret = i2c_write(config->i2c.bus, buf, sizeof(buf), config->i2c.addr);
    if (ret < 0) {
        LOG_ERR("Failed to write to I2C device: %d.\n", ret);
    }
    return ret;
};


static int pump_i2c_read_int16(const struct device *dev, int register_id, int16_t *read_value) {
    const struct smartpump_config *config = dev->config;

    uint8_t cmd[1];
    cmd[0] = 128 + register_id;

    int ret = i2c_write(config->i2c.bus, cmd, sizeof(cmd), config->i2c.addr);
    if (ret < 0) {
        LOG_ERR("Failed to write to I2C device: %d.\n", ret);
        return ret;
    }

    uint8_t buf[2];
    buf[0] = register_id;

    ret = i2c_read(config->i2c.bus, buf, sizeof(buf), config->i2c.addr);
    if (ret < 0) {
        LOG_ERR("Failed to read from I2C device: %d.\n", ret);
        return ret;
    }

    uint8_t *ptrToInt;
    ptrToInt = (uint8_t *) read_value;
    ptrToInt[0] = buf[0];
    ptrToInt[1] = buf[1];

    return 0;
}

static int pump_i2c_read_float(const struct device *dev, int register_id, float *read_value) {
    const struct smartpump_config *config = dev->config;

    uint8_t cmd[1];
    cmd[0] = 128 + register_id;
    int ret = i2c_write(config->i2c.bus, cmd, sizeof(cmd), config->i2c.addr);
    if (ret < 0) {
        LOG_ERR("Failed to write to I2C device: %d.\n", ret);
        return ret;
    }
    uint8_t buf[4];

    ret = i2c_read(config->i2c.bus, buf, sizeof(buf), config->i2c.addr);
    if (ret < 0) {
        LOG_ERR("Failed to read from I2C device: %d.\n", ret);
        return ret;
    }

    uint8_t *ptrToFloat;
    ptrToFloat = (uint8_t *) read_value;
    ptrToFloat[0] = buf[0];
    ptrToFloat[1] = buf[1];
    ptrToFloat[2] = buf[2];
    ptrToFloat[3] = buf[3];

    return 0;
}

static int smartpump_set_pump_enabled(const struct device *dev, bool enabled) {
    if (enabled) {
        pump_i2c_write_int16(dev, REGISTER_PUMP_ENABLE, 1);
        LOG_INF("Pump Set: Enabled.\n");
        return 0;
    } else {
        pump_i2c_write_int16(dev, REGISTER_PUMP_ENABLE, 0);
        LOG_INF("Pump Set: Disabled.\n");
        return 0;
    }
}

static bool smartpump_get_pump_enabled(const struct device *dev) {
    int16_t result;
    pump_i2c_read_int16(dev, REGISTER_PUMP_ENABLE, &result);
    if (result) {
        LOG_INF("Get Pump status: Enabled.\n");
        return true;
    } else {
        LOG_INF("Get Pump status: Disabled.\n");
        return false;
    }
}

static int smartpump_set_target_value(const struct device *dev, float power) {
    if (power <= PUMP_MAX_PRESSURE) {
        pump_i2c_write_float(dev, REGISTER_SET_VAL, power);
        LOG_INF("Target Value Set.\n");
        return 0;
    } else {
        LOG_ERR("Power request out of bounds.\n");
        return -EINVAL;
    }
}

static float smartpump_get_target_value(const struct device *dev) {
    float result;
    pump_i2c_read_float(dev, REGISTER_SET_VAL, &result);
    if (CONFIG_SMARTPUMP_LOG_LEVEL > 3) {
        int main_part = (int) result;
        int decimal_part = (result - main_part) * 1000;
        LOG_DBG("%d.%d.\n", main_part, decimal_part);
    }
    return result;
}


static float smartpump_get_measured_pressure(const struct device *dev) {
    float result;
    pump_i2c_read_float(dev, REGISTER_MEAS_DIGITAL_PRESSURE, &result);
    if (CONFIG_SMARTPUMP_LOG_LEVEL > 3) {
        int main_part = (int) result;
        int decimal_part = (result - main_part) * 1000;
        LOG_DBG("%d.%d.\n", main_part, decimal_part);
    }
    return result;
}

static struct measured_vcp smartpump_get_measured_vcp(const struct device *dev) {
    struct measured_vcp vcp;
    pump_i2c_read_float(dev, REGISTER_MEAS_DRIVE_VOLTS, &vcp.volts);
    pump_i2c_read_float(dev, REGISTER_MEAS_DRIVE_MILLIAMPS, &vcp.current);
    pump_i2c_read_float(dev, REGISTER_MEAS_DRIVE_MILLIWATTS, &vcp.power);
    return vcp;
}

static int smartpump_set_manual_mode_external_source(const struct device *dev, bool source) {
    if (source) {
        pump_i2c_write_int16(dev, REGISTER_MANUAL_MODE_SETPOINT_SOURCE, 3);
        LOG_INF("Manual Mode power source set to Analog input.\n");
    } else {
        pump_i2c_write_int16(dev, REGISTER_MANUAL_MODE_SETPOINT_SOURCE, 0);
        LOG_INF("Manual Mode power source set to Digital Register.\n");
    }
    return 0;
}

static int smartpump_set_PID_params(const struct device *dev, struct pid_params params) {
    pump_i2c_write_int16(dev, REGISTER_PID_MODE_SETPOINT_SOURCE, params.setpoint_source);
    pump_i2c_write_int16(dev, REGISTER_PID_MODE_MEAS_SOURCE, params.input_source);
    pump_i2c_write_int16(dev, REGISTER_RESET_PID_ON_TURNON, params.rst_on_pwr);
    pump_i2c_write_float(dev, REGISTER_PID_PROPORTIONAL_COEFF, params.prop_coeff);
    pump_i2c_write_float(dev, REGISTER_PID_INTEGRAL_COEFF, params.integral_coeff);
    pump_i2c_write_float(dev, REGISTER_PID_DIFFERENTIAL_COEFF, params.diff_coeff);
    LOG_INF("PID Params set.\n");
    return 0;
}

static struct pid_params smartpump_get_PID_params(const struct device *dev) {
    struct pid_params params;
    pump_i2c_read_int16(dev, REGISTER_RESET_PID_ON_TURNON, &params.rst_on_pwr);
    pump_i2c_read_int16(dev, REGISTER_PID_MODE_SETPOINT_SOURCE, &params.setpoint_source);
    pump_i2c_read_int16(dev, REGISTER_PID_MODE_MEAS_SOURCE, &params.input_source);
    pump_i2c_read_float(dev, REGISTER_PID_DIFFERENTIAL_COEFF, &params.diff_coeff);
    pump_i2c_read_float(dev, REGISTER_PID_INTEGRAL_COEFF, &params.integral_coeff);
    pump_i2c_read_float(dev, REGISTER_PID_PROPORTIONAL_COEFF, &params.prop_coeff);
    LOG_INF("Fetched PID Params. %d, %d, %d, %d, %d, %d\n", params.rst_on_pwr, params.setpoint_source,
            params.input_source, params.diff_coeff, params.integral_coeff, params.prop_coeff);
    return params;
}

static int smartpump_setup_manual_control(const struct device *dev) {
    pump_i2c_write_int16(dev, REGISTER_PUMP_ENABLE, 0);
    pump_i2c_write_int16(dev, REGISTER_STREAM_MODE_ENABLE, 0);
    pump_i2c_write_int16(dev, REGISTER_CONTROL_MODE, MODE_MANUAL);
    pump_i2c_write_int16(dev, REGISTER_MANUAL_MODE_SETPOINT_SOURCE, SOURCE_SETVAL);
    pump_i2c_write_float(dev, REGISTER_SET_VAL, 0.0f);
    LOG_INF("Pump set to manual control.\n");
    return 0;
}


static int smartpump_setup_PID_pressure_control(const struct device *dev) {
    struct pid_params params;
    params.rst_on_pwr = 1;
    params.setpoint_source = SOURCE_SETVAL;
    params.input_source = SOURCE_DIGITAL_PRESSURE;
    params.diff_coeff = 0;
    params.integral_coeff = 10;
    params.prop_coeff = 5;

    pump_i2c_write_int16(dev, REGISTER_PUMP_ENABLE, 0);
    pump_i2c_write_int16(dev, REGISTER_STREAM_MODE_ENABLE, 0);
    pump_i2c_write_int16(dev, REGISTER_CONTROL_MODE, MODE_PID);
    set_pump_PID_params(dev, params);
    pump_i2c_write_float(dev, REGISTER_SET_VAL, 0.0f);
    LOG_INF("Pump set to PID control.\n");
    return 0;
}


static const struct smartpump_driver_api smartpump_api = {
        .setup_pump_PID_pressure_control = &smartpump_setup_PID_pressure_control,
        .setup_pump_manual_control = &smartpump_setup_manual_control,
        .set_pump_enabled = &smartpump_set_pump_enabled,
        .get_pump_enabled = &smartpump_get_pump_enabled,
        .set_pump_target_value = &smartpump_set_target_value,
        .get_pump_target_value = &smartpump_get_target_value,
        .get_pump_measured_pressure = &smartpump_get_measured_pressure,
        .get_pump_measured_vcp = &smartpump_get_measured_vcp,
        .set_pump_manual_mode_external_source = &smartpump_set_manual_mode_external_source,
        .set_pump_PID_params = &smartpump_set_PID_params,
        .get_pump_PID_params = &smartpump_get_PID_params,
};

static int smartpump_init(const struct device *dev) {
    const struct smartpump_config *config = dev->config;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("Bus device not ready.\n");
        return -ENODEV;
    }
    return 0;
}

#define SMARTPUMP_INIT(i)                               \
    static struct smartpump_data smartpump_data_##i;           \
                                           \
    static const struct smartpump_config smartpump_config_##i = {  \
        .i2c = I2C_DT_SPEC_INST_GET(i),\
    };                                       \
                                           \
    DEVICE_DT_INST_DEFINE(i, smartpump_init, NULL,               \
                  &smartpump_data_##i,                   \
                  &smartpump_config_##i, POST_KERNEL,           \
                  CONFIG_APPLICATION_INIT_PRIORITY, &smartpump_api);

DT_INST_FOREACH_STATUS_OKAY(SMARTPUMP_INIT)