/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Dipak Shetty
 * SPDX-FileCopyrightText: Copyright (c) 2025 Jilay Sandeep Pandya
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include "tmc51xx.h"
#include <zephyr/stepper/cb_tmc51xx.h>
#include <adi_tmc5xxx_common.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(cb_tmc51xx, CONFIG_STEPPER_LOG_LEVEL);

#define DT_DRV_COMPAT flintstone_tmc51xx_stepper_ctrl

struct tmc51xx_stepper_ctrl_config {
	const bool is_sg_enabled;
	const uint32_t sg_velocity_check_interval_ms;
	const uint32_t sg_threshold_velocity;
	/* CB-FORK: SW_MODE reference switch configuration, built from the
	 * stop-l-enable / pol-stop-l / ... devicetree properties.
	 */
	const uint32_t sw_mode;
#ifdef CONFIG_CB_STEPPER_TMC51XX_RAMP_GEN
	const struct tmc_ramp_generator_data default_ramp_config;
#endif
	/* parent controller required for bus communication */
	const struct device *controller;
};

struct tmc51xx_stepper_ctrl_data {
	struct k_work_delayable stallguard_dwork;
	/* Work item to run the callback in a thread context. */
	const struct device *dev;
	stepper_ctrl_event_callback_t callback;
	void *event_cb_user_data;
};

void tmc51xx_stepper_ctrl_trigger_cb(const struct device *dev,
				      const enum stepper_ctrl_event event)
{
	struct tmc51xx_stepper_ctrl_data *data = dev->data;

	if (!data->callback) {
		LOG_WRN_ONCE("No motion controller callback registered");
		return;
	}
	data->callback(dev, event, data->event_cb_user_data);
}

static int read_vactual(const struct device *dev, int32_t *actual_velocity)
{
	__ASSERT(actual_velocity != NULL, "actual_velocity pointer must not be NULL");
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;
	uint32_t raw_value;

	err = tmc51xx_read(controller, TMC51XX_VACTUAL, &raw_value);
	if (err) {
		LOG_ERR("Failed to read VACTUAL register");
		return err;
	}

	*actual_velocity = sign_extend(raw_value, TMC_RAMP_VACTUAL_SHIFT);
	if (*actual_velocity) {
		LOG_DBG("actual velocity: %d", *actual_velocity);
	}
	return 0;
}

int tmc51xx_stepper_ctrl_set_max_velocity(const struct device *dev, uint32_t velocity)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const uint32_t clock_frequency = tmc51xx_get_clock_frequency(config->controller);
	uint32_t velocity_fclk;
	int err;

	velocity_fclk = tmc5xxx_calculate_velocity_from_hz_to_fclk(velocity, clock_frequency);

	err = tmc51xx_write(config->controller, TMC51XX_VMAX, velocity_fclk);
	if (err != 0) {
		LOG_ERR("%s: Failed to set max velocity", dev->name);
		return -EIO;
	}
	return 0;
}

int tmc51xx_stepper_ctrl_set_ramp(const struct device *dev,
				  const struct tmc_ramp_generator_data *ramp_data);

int tmc51xx_stepper_ctrl_set_soft_ramp(const struct device *dev,
				       const struct tmc51xx_soft_ramp *ramp)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	const uint32_t fclk = tmc51xx_get_clock_frequency(controller);
	/*
	 * Start from the board's own ramp and overwrite only the fields this
	 * call actually owns.
	 *
	 * set_ramp() writes EVERY field of this struct to silicon, so building
	 * a fresh one with a designated initialiser does not mean "leave the
	 * rest alone" - it means "set the rest to zero". That cost us three
	 * registers before anyone noticed:
	 *
	 *   IHOLD_IRUN  -> IRUN 0. On a TMC5130 current scale 0 is 1/32 of full
	 *                  scale, not off, so the motor kept turning at roughly
	 *                  3% torque and presented as a mechanical fault.
	 *   TPWMTHRS    -> 0. With en_pwm_mode set, TSTEP >= TPWMTHRS at every
	 *                  velocity, so stealthChop never hands over to
	 *                  spreadCycle and the carriage stalls at speed under
	 *                  load - the exact failure the board's tpwmthrs = 155
	 *                  was chosen to prevent.
	 *   TPOWER_DOWN -> 0, collapsing to the hold current the instant
	 *                  standstill is flagged rather than ~199 ms later.
	 *
	 * All three were invisible at boot, because init writes the complete
	 * devicetree config; they only appeared on the first ramp change, which
	 * is why a reboot looked like a fix. Copying the board config first
	 * means any field added to this struct in future defaults to what the
	 * board asked for rather than silently to zero.
	 */
	struct tmc_ramp_generator_data data = config->default_ramp_config;

	data.vstart = tmc5xxx_calculate_velocity_from_hz_to_fclk(ramp->vstart, fclk);
	data.v1 = tmc5xxx_calculate_velocity_from_hz_to_fclk(ramp->v1, fclk);
	data.vmax = tmc5xxx_calculate_velocity_from_hz_to_fclk(ramp->vmax, fclk);
	data.vstop = tmc5xxx_calculate_velocity_from_hz_to_fclk(ramp->vstop, fclk);
	data.a1 = tmc5xxx_calculate_acceleration_from_hz_to_fclk(ramp->a1, fclk);
	data.amax = tmc5xxx_calculate_acceleration_from_hz_to_fclk(ramp->amax, fclk);
	data.d1 = tmc5xxx_calculate_acceleration_from_hz_to_fclk(ramp->d1, fclk);
	data.dmax = tmc5xxx_calculate_acceleration_from_hz_to_fclk(ramp->dmax, fclk);
	data.tzerowait = ramp->tzerowait;

	/* VSTOP must be at least 1 or the ramp generator never finishes a
	 * positioning move.
	 */
	if (data.vstop < TMC_RAMP_VSTOP_MIN) {
		data.vstop = TMC_RAMP_VSTOP_MIN;
	}
	if (data.d1 < TMC_RAMP_D1_MIN) {
		data.d1 = TMC_RAMP_D1_MIN;
	}

	return tmc51xx_stepper_ctrl_set_ramp(dev, &data);
}

int tmc51xx_stepper_ctrl_set_limits(const struct device *dev, uint32_t vmax_hz,
				    uint32_t amax_hz2)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	const uint32_t fclk = tmc51xx_get_clock_frequency(controller);
	uint32_t v = tmc5xxx_calculate_velocity_from_hz_to_fclk(vmax_hz, fclk);
	uint32_t a = tmc5xxx_calculate_acceleration_from_hz_to_fclk(amax_hz2, fclk);
	int err;

	/* AMAX of zero stalls the ramp generator outright: it can never reach
	 * any new velocity, so the carriage simply stops responding to further
	 * commands. A curve legitimately passes through zero acceleration at
	 * an inflection point, so clamp rather than reject.
	 */
	if (a == 0) {
		a = 1;
	}

	err = tmc51xx_write(controller, TMC51XX_AMAX, a);
	if (err == 0) {
		err = tmc51xx_write(controller, TMC51XX_DMAX, a);
	}
	if (err == 0) {
		err = tmc51xx_write(controller, TMC51XX_VMAX, v);
	}
	if (err != 0) {
		LOG_ERR("%s: failed to set limits (%d)", dev->name, err);
	}
	return err;
}

int tmc51xx_stepper_ctrl_set_target(const struct device *dev, int32_t position)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	err = tmc51xx_write(controller, TMC51XX_RAMPMODE,
			    TMC5XXX_RAMPMODE_POSITIONING_MODE);
	if (err == 0) {
		err = tmc51xx_write(controller, TMC51XX_XTARGET, position);
	}
	if (err != 0) {
		LOG_ERR("%s: failed to set target (%d)", dev->name, err);
	}
	return err;
}

int tmc51xx_stepper_ctrl_read_reg(const struct device *dev, uint8_t reg, uint32_t *value)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;

	__ASSERT(value != NULL, "value pointer must not be NULL");

	if (tmc51xx_read(config->controller, reg, value) != 0) {
		LOG_ERR("%s: failed to read register 0x%02x", dev->name, reg);
		return -EIO;
	}
	return 0;
}

int tmc51xx_stepper_ctrl_read_drv_status(const struct device *dev, uint32_t *drv_status)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	int err;

	__ASSERT(drv_status != NULL, "drv_status pointer must not be NULL");

	err = tmc51xx_read(config->controller, TMC51XX_DRVSTATUS, drv_status);
	if (err != 0) {
		LOG_ERR("%s: Failed to read DRVSTATUS register", dev->name);
		return -EIO;
	}
	return 0;
}

int tmc51xx_stepper_ctrl_read_chopconf(const struct device *dev, uint32_t *chopconf)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	int err;

	__ASSERT(chopconf != NULL, "chopconf pointer must not be NULL");

	err = tmc51xx_read(config->controller, TMC51XX_CHOPCONF, chopconf);
	if (err != 0) {
		LOG_ERR("%s: Failed to read CHOPCONF register", dev->name);
		return -EIO;
	}
	return 0;
}

int tmc51xx_stepper_ctrl_read_ramp_stat(const struct device *dev, uint32_t *ramp_stat)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	int err;

	__ASSERT(ramp_stat != NULL, "ramp_stat pointer must not be NULL");

	err = tmc51xx_read(config->controller, TMC51XX_RAMPSTAT, ramp_stat);
	if (err != 0) {
		LOG_ERR("%s: Failed to read RAMPSTAT register", dev->name);
		return -EIO;
	}
	return 0;
}

int tmc51xx_stepper_ctrl_stallguard_enable(const struct device *dev, const bool enable)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc51xx_read(controller, TMC51XX_SWMODE, &reg_value);
	if (err) {
		LOG_ERR("Failed to read SWMODE register");
		return -EIO;
	}

	if (enable) {
		reg_value |= TMC5XXX_SW_MODE_SG_STOP_ENABLE;

		int32_t actual_velocity;

		err = read_vactual(dev, &actual_velocity);
		if (err) {
			return -EIO;
		}
		if (abs(actual_velocity) < config->sg_threshold_velocity) {
			return -EAGAIN;
		}
	} else {
		reg_value &= ~TMC5XXX_SW_MODE_SG_STOP_ENABLE;
	}
	err = tmc51xx_write(controller, TMC51XX_SWMODE, reg_value);
	if (err) {
		LOG_ERR("Failed to write SWMODE register");
		return -EIO;
	}

	LOG_DBG("Stallguard %s", enable ? "enabled" : "disabled");
	return 0;
}

static void stallguard_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tmc51xx_stepper_ctrl_data const *data =
		CONTAINER_OF(dwork, struct tmc51xx_stepper_ctrl_data, stallguard_dwork);
	const struct device *dev = data->dev;
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	int err;

	err = tmc51xx_stepper_ctrl_stallguard_enable(dev, true);
	if (err == -EAGAIN) {
		k_work_reschedule(dwork, K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (err == -EIO) {
		LOG_ERR("Failed to enable stallguard because of I/O error");
	}
}

static int tmc51xx_stepper_ctrl_set_event_cb(const struct device *dev,
						    stepper_ctrl_event_callback_t callback,
						    void *user_data)
{
	struct tmc51xx_stepper_ctrl_data *data = dev->data;

	data->callback = callback;
	data->event_cb_user_data = user_data;

	return 0;
}

static int tmc51xx_stepper_ctrl_is_moving(const struct device *dev, bool *is_moving)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc51xx_read(controller, TMC51XX_DRVSTATUS, &reg_value);

	if (err != 0) {
		LOG_ERR("%s: Failed to read DRVSTATUS register", dev->name);
		return -EIO;
	}

	*is_moving = (FIELD_GET(TMC5XXX_DRV_STATUS_STST_BIT, reg_value) != 1U);
	LOG_DBG("Stepper motor controller %s is moving: %d", dev->name, *is_moving);
	return 0;
}

static int tmc51xx_stepper_ctrl_set_reference_position(const struct device *dev,
							const int32_t position)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	err = tmc51xx_write(controller, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_HOLD_MODE);
	if (err != 0) {
		return -EIO;
	}

	err = tmc51xx_write(controller, TMC51XX_XACTUAL, position);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("Stepper motor controller %s set actual position to %d", dev->name, position);
	return 0;
}

static int tmc51xx_stepper_ctrl_get_actual_position(const struct device *dev, int32_t *position)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	int err;

	err = tmc51xx_read_actual_position(config->controller, position);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("%s actual position: %d", dev->name, *position);
	return 0;
}

static int tmc51xx_stepper_ctrl_move_to(const struct device *dev, const int32_t micro_steps)
{
	LOG_DBG("%s set target position to %d", dev->name, micro_steps);
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	struct tmc51xx_stepper_ctrl_data *data = dev->data;
	const struct device *comm_device = config->controller;
	int err;

	if (config->is_sg_enabled) {
		tmc51xx_stepper_ctrl_stallguard_enable(dev, false);
	}

	err = tmc51xx_write(comm_device, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_POSITIONING_MODE);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(comm_device, TMC51XX_XTARGET, micro_steps);
	if (err != 0) {
		return -EIO;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (data->callback) {
		/* For SPI with DIAG0 pin, we use interrupt-driven approach */

		if (tmc51xx_is_interrupt_driven(config->controller)) {
			return 0;
		}
		/* For UART or SPI without DIAG0, reschedule RAMPSTAT polling */
#ifdef CONFIG_CB_STEPPER_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC
		tmc51xx_reschedule_rampstat_callback(config->controller);
#endif /* CONFIG_CB_STEPPER_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC */
	}
	return 0;
}

static int tmc51xx_stepper_ctrl_move_by(const struct device *dev, const int32_t micro_steps)
{
	int err;
	int32_t position;

	err = tmc51xx_stepper_ctrl_get_actual_position(dev, &position);
	if (err != 0) {
		return -EIO;
	}
	int32_t target_position = position + micro_steps;

	LOG_DBG("%s moved to %d by steps: %d", dev->name, target_position, micro_steps);

	return tmc51xx_stepper_ctrl_move_to(dev, target_position);
}

static int tmc51xx_stepper_ctrl_run(const struct device *dev,
				     const enum stepper_ctrl_direction direction)
{
	LOG_DBG("Stepper motor controller %s run", dev->name);
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	struct tmc51xx_stepper_ctrl_data *data = dev->data;
	const struct device *controller = config->controller;
	int err;

	if (config->is_sg_enabled) {
		err = tmc51xx_stepper_ctrl_stallguard_enable(dev, false);
		if (err != 0) {
			return -EIO;
		}
	}

	switch (direction) {
	case STEPPER_CTRL_DIRECTION_POSITIVE:
		err = tmc51xx_write(controller, TMC51XX_RAMPMODE,
				    TMC5XXX_RAMPMODE_POSITIVE_VELOCITY_MODE);
		if (err != 0) {
			return -EIO;
		}
		break;

	case STEPPER_CTRL_DIRECTION_NEGATIVE:
		err = tmc51xx_write(controller, TMC51XX_RAMPMODE,
				    TMC5XXX_RAMPMODE_NEGATIVE_VELOCITY_MODE);
		if (err != 0) {
			return -EIO;
		}
		break;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (data->callback) {
		/* For SPI with DIAG0 pin, we use interrupt-driven approach */
		if (tmc51xx_is_interrupt_driven(controller)) {
			return 0;
		}

		/* For UART or SPI without DIAG0, reschedule RAMPSTAT polling */
#ifdef CONFIG_CB_STEPPER_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC
		tmc51xx_reschedule_rampstat_callback(config->controller);
#endif /* CONFIG_CB_STEPPER_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC */
	}
	return 0;
}

#ifdef CONFIG_CB_STEPPER_TMC51XX_RAMP_GEN

int tmc51xx_stepper_ctrl_set_ramp(const struct device *dev,
				   const struct tmc_ramp_generator_data *ramp_data)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	LOG_DBG("Stepper motor controller %s set ramp", dev->name);

	err = tmc51xx_write(controller, TMC51XX_VSTART, ramp_data->vstart);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_A1, ramp_data->a1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_AMAX, ramp_data->amax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_D1, ramp_data->d1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_DMAX, ramp_data->dmax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_V1, ramp_data->v1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_VMAX, ramp_data->vmax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_VSTOP, ramp_data->vstop);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_TZEROWAIT, ramp_data->tzerowait);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_THIGH, ramp_data->thigh);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_TCOOLTHRS, ramp_data->tcoolthrs);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_TPWMTHRS, ramp_data->tpwmthrs);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_TPOWER_DOWN, ramp_data->tpowerdown);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_IHOLD_IRUN, ramp_data->iholdrun);
	if (err != 0) {
		return -EIO;
	}
	return 0;
}

#endif

static int tmc51xx_stepper_ctrl_init(const struct device *dev)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	struct tmc51xx_stepper_ctrl_data *data = dev->data;
	const struct device *controller = config->controller;
	uint32_t sw_mode;
	int err;

	data->dev = dev;

	if (config->is_sg_enabled) {
		k_work_init_delayable(&data->stallguard_dwork, stallguard_work_handler);
	}

	/* CB-FORK CHANGE 1 of 2 (the reason this fork exists).
	 *
	 * Upstream never writes SW_MODE bits 0-3, so the TMC's REFL/REFR
	 * reference switch inputs stay disabled: the motor does not stop on an
	 * end stop and the LEFT/RIGHT_END_STOP_DETECTED events never fire.
	 * On this board the hall switches are wired to the TMC, not to the
	 * MCU, so without this there is no homing at all.
	 *
	 * Upstream also wrote SW_MODE = BIT(10) unconditionally when
	 * stallguard was enabled, which would have clobbered these bits.
	 * Both are merged into a single write instead.
	 */
	sw_mode = config->sw_mode;
	if (config->is_sg_enabled) {
		sw_mode |= TMC5XXX_SW_MODE_SG_STOP_ENABLE;
	}

	if (sw_mode != 0) {
		LOG_DBG("SW_MODE: 0x%x", sw_mode);
		err = tmc51xx_write(controller, TMC51XX_SWMODE, sw_mode);
		if (err != 0) {
			return -EIO;
		}
	}

	if (config->is_sg_enabled) {
		LOG_DBG("stallguard delay %d ms", config->sg_velocity_check_interval_ms);

		k_work_reschedule(&data->stallguard_dwork, K_NO_WAIT);
	}

#ifdef CONFIG_CB_STEPPER_TMC51XX_RAMP_GEN
	err = tmc51xx_stepper_ctrl_set_ramp(dev, &config->default_ramp_config);
	if (err != 0) {
		return -EIO;
	}
#endif
	return 0;
}

static int tmc51xx_stepper_ctrl_stop(const struct device *dev)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	err = tmc51xx_write(controller, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_POSITIVE_VELOCITY_MODE);
	if (err != 0) {
		return -EIO;
	}

	err = tmc51xx_write(controller, TMC51XX_VMAX, 0);
	if (err != 0) {
		return -EIO;
	}

	return 0;
}

static int tmc51xx_stepper_ctrl_configure_ramp(const struct device *dev,
					       const struct stepper_ctrl_ramp *ramp)
{
	const struct tmc51xx_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	const uint32_t clock_frequency = tmc51xx_get_clock_frequency(config->controller);
	uint32_t velocity_fclk =
		tmc5xxx_calculate_velocity_from_hz_to_fclk(ramp->speed_max, clock_frequency);
	uint32_t accel_fclk = tmc5xxx_calculate_acceleration_from_hz_to_fclk(ramp->acceleration_max,
									     clock_frequency);
	uint32_t decel_fclk = tmc5xxx_calculate_acceleration_from_hz_to_fclk(ramp->deceleration_max,
									     clock_frequency);
	int err;

	LOG_DBG("Stepper motor controller %s set ramp", dev->name);

	err = tmc51xx_write(controller, TMC51XX_A1, accel_fclk);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_AMAX, accel_fclk);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_VMAX, velocity_fclk);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_DMAX, decel_fclk);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_D1, decel_fclk);
	if (err != 0) {
		return -EIO;
	}

	return 0;
}

static DEVICE_API(stepper_ctrl, tmc51xx_stepper_ctrl_api) = {
	.is_moving = tmc51xx_stepper_ctrl_is_moving,
	.move_by = tmc51xx_stepper_ctrl_move_by,
	.set_reference_position = tmc51xx_stepper_ctrl_set_reference_position,
	.get_actual_position = tmc51xx_stepper_ctrl_get_actual_position,
	.move_to = tmc51xx_stepper_ctrl_move_to,
	.run = tmc51xx_stepper_ctrl_run,
	.stop = tmc51xx_stepper_ctrl_stop,
	.set_event_cb = tmc51xx_stepper_ctrl_set_event_cb,
	.configure_ramp = tmc51xx_stepper_ctrl_configure_ramp,
};

#define TMC51XX_STEPPER_CTRL_DEFINE(inst)                                                         \
	IF_ENABLED(CONFIG_CB_STEPPER_TMC51XX_RAMP_GEN,	(CHECK_RAMP_DT_DATA(inst)));               \
	static const struct tmc51xx_stepper_ctrl_config tmc51xx_stepper_ctrl_cfg_##inst = {      \
		.controller = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),                         \
		.sg_threshold_velocity = DT_INST_PROP(inst, stallguard_threshold_velocity),        \
		.sg_velocity_check_interval_ms =                                                   \
			DT_INST_PROP(inst, stallguard_velocity_check_interval_ms),                 \
		.is_sg_enabled = DT_INST_PROP(inst, activate_stallguard2),                         \
		.sw_mode = ((DT_INST_PROP(inst, stop_l_enable)                                     \
				     ? TMC5XXX_SW_MODE_STOP_L_ENABLE : 0) |                        \
			    (DT_INST_PROP(inst, stop_r_enable)                                     \
				     ? TMC5XXX_SW_MODE_STOP_R_ENABLE : 0) |                        \
			    (DT_INST_PROP(inst, pol_stop_l)                                        \
				     ? TMC5XXX_SW_MODE_POL_STOP_L : 0) |                           \
			    (DT_INST_PROP(inst, pol_stop_r)                                        \
				     ? TMC5XXX_SW_MODE_POL_STOP_R : 0) |                           \
			    (DT_INST_PROP(inst, swap_lr)                                           \
				     ? TMC5XXX_SW_MODE_SWAP_LR : 0) |                              \
			    (DT_INST_PROP(inst, en_softstop)                                       \
				     ? TMC5XXX_SW_MODE_EN_SOFTSTOP : 0)),                          \
		IF_ENABLED(CONFIG_CB_STEPPER_TMC51XX_RAMP_GEN,                                    \
			(.default_ramp_config = TMC_RAMP_DT_SPEC_GET_TMC51XX(inst))) };            \
	static struct tmc51xx_stepper_ctrl_data tmc51xx_stepper_ctrl_data_##inst;                \
	DEVICE_DT_INST_DEFINE(inst, tmc51xx_stepper_ctrl_init, NULL,                              \
			      &tmc51xx_stepper_ctrl_data_##inst,                                  \
			      &tmc51xx_stepper_ctrl_cfg_##inst, POST_KERNEL,                      \
			      CONFIG_STEPPER_INIT_PRIORITY, &tmc51xx_stepper_ctrl_api);

DT_INST_FOREACH_STATUS_OKAY(TMC51XX_STEPPER_CTRL_DEFINE)
