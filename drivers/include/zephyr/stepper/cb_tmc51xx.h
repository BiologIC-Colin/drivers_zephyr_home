/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Extra API for the CB fork of the Trinamic TMC51xx stepper driver.
 *
 * Everything else about the fork is source-compatible with the in-tree driver,
 * so keep using <zephyr/drivers/stepper/stepper.h>, stepper_ctrl.h and
 * stepper_trinamic.h as normal. This header only carries what the fork adds.
 */

#ifndef CB_DRIVERS_STEPPER_CB_TMC51XX_H_
#define CB_DRIVERS_STEPPER_CB_TMC51XX_H_

#include <zephyr/device.h>
#include <zephyr/types.h>

/**
 * @brief Full six-point ramp, in real units.
 *
 * The generic stepper_ctrl_configure_ramp() writes A1 = AMAX and D1 = DMAX,
 * which flattens the TMC's ramp into a plain trapezoid and throws away the one
 * feature that makes it gentle: a separate, lower acceleration below V1. On a
 * slider carrying a top-heavy camera that step change in acceleration is what
 * twists the payload on its mount.
 *
 * Setting a1 and d1 well below amax and dmax softens both ends of the move,
 * where the load is most sensitive, without slowing the middle.
 *
 * Velocities are microsteps/s, accelerations microsteps/s^2.
 */
struct tmc51xx_soft_ramp {
	uint32_t vstart;
	uint32_t a1;        /**< acceleration below v1 - keep this gentle */
	uint32_t v1;        /**< knee between the soft and firm phases */
	uint32_t amax;
	uint32_t vmax;
	uint32_t dmax;
	uint32_t d1;        /**< deceleration below v1 - keep this gentle */
	uint32_t vstop;
	uint16_t tzerowait; /**< raw, in units of 512/fCLK */
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read the raw RAMP_STAT register
 *
 * Added by the fork. On this board the hall end stops are wired to the TMC's
 * REFL/REFR pins rather than to MCU GPIOs, so this is the only way to see the
 * live switch state. Bit 0 is status_stop_l and bit 1 is status_stop_r, both
 * reported after the pol_stop_* inversion; bits 4 and 5 are the latched
 * event_stop_l / event_stop_r flags.
 *
 * This read does not clear the latched event bits - the driver clears those
 * from its own RAMP_STAT handling.
 *
 * @param dev Pointer to the stepper motion controller instance
 * @param ramp_stat Output for the raw register value
 *
 * @retval -EIO General input / output error
 * @retval 0 Success
 */
int tmc51xx_stepper_ctrl_read_ramp_stat(const struct device *dev, uint32_t *ramp_stat);

/**
 * @brief Read DRV_STATUS - open load, short, over temperature, standstill
 *
 * The one register that says why a motor with an enabled driver is producing no
 * torque. ola/olb (bits 29/30) mean the coil is not connected.
 */
int tmc51xx_stepper_ctrl_read_drv_status(const struct device *dev, uint32_t *drv_status);

/**
 * @brief Read any TMC register by address
 *
 * Bring-up escape hatch. GSTAT (0x01) in particular carries uv_cp, the
 * charge-pump undervoltage flag, which is the difference between "the motor is
 * too weak" and "the motor has no usable supply".
 */
int tmc51xx_stepper_ctrl_read_reg(const struct device *dev, uint8_t reg, uint32_t *value);

/*
 * Set only the velocity and acceleration limits, leaving every other ramp
 * register alone, and point the ramp generator at an absolute target.
 *
 * CB-FORK ADDITION. The cue engine re-shapes the ramp at 100 Hz while a move
 * is running, which is what lets a chip whose ramp generator only knows
 * trapezoids follow an arbitrary curve. Going through set_soft_ramp() for
 * that would write thirteen registers per tick when two of them change; this
 * writes three, and leaves the chopper and current configuration that the
 * rest of the ramp struct carries untouched.
 *
 * vmax, amax and dmax are in Hz and Hz/s - microsteps per second, and per
 * second squared. The conversion to the chip's internal clock units happens
 * here.
 *
 * AMAX and DMAX are deliberately separate. They were originally set from one
 * value taken from the curve's instantaneous acceleration, which is a trap:
 * wherever a curve runs at constant velocity its acceleration is zero, so
 * DMAX went to zero with it and the ramp generator lost the ability to
 * decelerate at all. It could then never stop at XTARGET. At the AMAX LSB of
 * fCLK^2/2^41 (about 79 usteps/s^2, 0.093 mm/s^2 on this machine) the
 * stopping distance from a mere 20 mm/s is over two metres - twice the rail.
 * Observed on the bench: a move with a long constant-velocity tail drove into
 * the end stop with 9 s still on the clock.
 */
int tmc51xx_stepper_ctrl_set_limits(const struct device *dev, uint32_t vmax_hz,
				    uint32_t amax_hz2, uint32_t dmax_hz2);

/* Absolute target for the ramp generator, without touching the ramp shape.
 * Selects positioning mode as a side effect, exactly as move_to does.
 */
int tmc51xx_stepper_ctrl_set_target(const struct device *dev, int32_t position);

/** @brief Read CHOPCONF - TOFF in bits 3:0 is zero when the chopper is off. */
int tmc51xx_stepper_ctrl_read_chopconf(const struct device *dev, uint32_t *chopconf);

/**
 * @brief Write the full six-point ramp, converting from real units
 *
 * Conversion needs fCLK, which only the driver knows, so it happens here
 * rather than in the application.
 *
 * @param dev Pointer to the stepper motion controller instance
 * @param ramp Ramp in microsteps/s and microsteps/s^2
 *
 * @retval -EIO General input / output error
 * @retval 0 Success
 */
int tmc51xx_stepper_ctrl_set_soft_ramp(const struct device *dev,
				       const struct tmc51xx_soft_ramp *ramp);

#ifdef __cplusplus
}
#endif

#endif /* CB_DRIVERS_STEPPER_CB_TMC51XX_H_ */
