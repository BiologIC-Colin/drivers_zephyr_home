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
