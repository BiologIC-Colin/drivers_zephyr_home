# SPDX-License-Identifier: Apache-2.0
#
# Local fork of the Zephyr ADI Trinamic TMC51xx stepper driver.
# See README.md in this directory for what was changed and why.

add_subdirectory(bus)
add_subdirectory(common)
add_subdirectory_ifdef(CONFIG_CB_STEPPER_TMC51XX tmc51xx)
