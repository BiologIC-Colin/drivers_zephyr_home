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

#ifdef __cplusplus
}
#endif

#endif /* CB_DRIVERS_STEPPER_CB_TMC51XX_H_ */
