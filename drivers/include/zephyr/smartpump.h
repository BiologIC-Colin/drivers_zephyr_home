#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SMARTPUMP_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SMARTPUMP_H_

#include <zephyr/device.h>
#include <zephyr/toolchain.h>


struct measured_vcp {
    float volts;
    float current;
    float power;
};

/**
 * @struct pid_params
 * @brief The data structure for storing parameters used in PID control.
 */
struct pid_params {
    float prop_coeff;
    float integral_coeff;
    float diff_coeff;
    int16_t rst_on_pwr;
    int16_t setpoint_source;
    int16_t input_source;
};

typedef int (*setup_manual_control_t)(const struct device *dev);

typedef int (*setup_PID_pressure_control_t)(const struct device *dev);

typedef int (*set_target_value_t)(const struct device *dev, float power);

typedef float (*get_target_value_t)(const struct device *dev);

typedef int (*set_pump_enabled_t)(const struct device *dev, bool enabled);

typedef bool(*get_pump_enabled_t)(const struct device *dev);

typedef float (*get_measured_pressure_t)(const struct device *dev);

typedef struct measured_vcp (*get_measured_vcp_t)(const struct device *dev);

typedef int (*set_manual_mode_external_source_t)(const struct device *dev, bool source);

typedef int (*set_PID_params_t)(const struct device *dev, struct pid_params);

typedef struct pid_params (*get_PID_params_t)(const struct device *dev);

__subsystem struct smartpump_driver_api {
    setup_manual_control_t setup_pump_manual_control;
    setup_PID_pressure_control_t setup_pump_PID_pressure_control;
    set_target_value_t set_pump_target_value;
    get_target_value_t get_pump_target_value;
    set_pump_enabled_t set_pump_enabled;
    get_pump_enabled_t get_pump_enabled;
    get_measured_pressure_t get_pump_measured_pressure;
    get_measured_vcp_t get_pump_measured_vcp;
    set_manual_mode_external_source_t set_pump_manual_mode_external_source;
    set_PID_params_t set_pump_PID_params;
    get_PID_params_t get_pump_PID_params;
};

__syscall int setup_pump_manual_control(const struct device *dev);

static inline int z_impl_setup_pump_manual_control(const struct device *dev) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->setup_pump_manual_control(dev);
}

__syscall int setup_pump_PID_pressure_control(const struct device *dev);

static inline int z_impl_setup_pump_PID_pressure_control(const struct device *dev) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->setup_pump_PID_pressure_control(dev);
}

__syscall int set_pump_target_value(const struct device *dev, float power);

static inline int z_impl_set_pump_target_value(const struct device *dev, float power) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->set_pump_target_value(dev, power);
}

__syscall float get_pump_target_value(const struct device *dev);

static inline float z_impl_get_pump_target_value(const struct device *dev) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->get_pump_target_value(dev);
}

__syscall int set_pump_enabled(const struct device *dev, bool enabled);

static inline int z_impl_set_pump_enabled(const struct device *dev, bool enabled) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->set_pump_enabled(dev, enabled);
}

__syscall bool get_pump_enabled(const struct device *dev);

static inline bool z_impl_get_pump_enabled(const struct device *dev) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->get_pump_enabled(dev);
}

__syscall float get_pump_measured_pressure(const struct device *dev);

static inline float z_impl_get_pump_measured_pressure(const struct device *dev) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->get_pump_measured_pressure(dev);
}

__syscall struct measured_vcp get_pump_measured_vcp(const struct device *dev);

static inline struct measured_vcp z_impl_get_pump_measured_vcp(const struct device *dev) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->get_pump_measured_vcp(dev);
}

__syscall int set_pump_manual_mode_external_source(const struct device *dev, bool source);

static inline int z_impl_set_pump_manual_mode_external_source(const struct device *dev, bool source) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->set_pump_manual_mode_external_source(dev, source);
}

__syscall int set_pump_PID_params(const struct device *dev, struct pid_params params);

static inline int z_impl_set_pump_PID_params(const struct device *dev, struct pid_params params) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->set_pump_PID_params(dev, params);
}

__syscall struct pid_params get_pump_PID_params(const struct device *dev);

static inline struct pid_params z_impl_get_pump_PID_params(const struct device *dev) {
    const struct smartpump_driver_api *api = (const struct smartpump_driver_api *) dev->api;
    return api->get_pump_PID_params(dev);
}


#include <syscalls/smartpump.h>

#endif // ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SMARTPUMP_H_