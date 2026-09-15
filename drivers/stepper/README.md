# TMC51xx stepper driver — local fork

A copy of Zephyr's in-tree `drivers/stepper/adi_tmc` TMC51xx driver, taken from
**Zephyr v4.4.2**, with two fixes that the Ki_Slider board needs.

It lives here rather than as a patch against `zephyr/` because `west update`
silently reverts anything modified in that tree.

## Why fork at all

The board's hall end stops are wired to the **TMC5130's own REFL/REFR pins**,
not to MCU GPIOs (J8 → R26/R27 10K pull-ups → R28/R29 22K → U7 pins 8 and 9).
The STM32 therefore has no way to see them except through the TMC. The upstream
driver never configures those inputs, so with it there is no homing at all.

## The two changes

Both are marked in the source with `CB-FORK CHANGE n of 2`, so
`grep -rn CB-FORK .` shows everything that differs from upstream.

### 1. SW_MODE was never written

Upstream only ever touches SW_MODE bit 10 (`sg_stop`, for stallGuard). Bits 0–3
— `stop_l_enable`, `stop_r_enable`, `pol_stop_l`, `pol_stop_r` — are left at
reset, so the reference switches do nothing: the motor does not stop on an end
stop and `STEPPER_CTRL_EVENT_LEFT/RIGHT_END_STOP_DETECTED` never fire.

Added:

- SW_MODE bit definitions in `common/include/adi_tmc_reg.h`
- devicetree properties `stop-l-enable`, `stop-r-enable`, `pol-stop-l`,
  `pol-stop-r`, `swap-lr`, `en-softstop` on the `*-stepper-ctrl` binding
- one combined SW_MODE write in `tmc51xx_stepper_ctrl_init()`

The write is merged with the stallGuard bit deliberately: upstream wrote
`SW_MODE = BIT(10)` unconditionally when stallGuard was enabled, which would
have wiped the switch configuration.

`pol_stop_*` is set on this board because the hall switches pull their input
**low** when triggered, against a pull-up. With the bit clear the polarity is
high-active and an idle switch would read as permanently triggered.

### 2. RAMPSTAT flags decoded as if they were an enum

`rampstat_work_handler()` extracted RAMPSTAT bits 4–9 as a field and
`switch`ed on the whole value. Those bits are independent flags that latch
together, so any combination without a matching `case` fell through to
`LOG_ERR("Illegal ramp stat bit field")` and **the event was dropped**.

Seen on this board: homing onto the right end stop latches `event_stop_r`
(bit 5) and `velocity_reached` (bit 8) at the same time — field value `0x12` —
and the following `move_by()` never reported completion.

Now each flag is tested individually, which also lets one interrupt report more
than one thing, as the hardware actually does.

### Also added (not a fix)

`tmc51xx_stepper_ctrl_read_ramp_stat()`, declared in
`drivers/include/zephyr/stepper/cb_tmc51xx.h`. Since the switches are invisible
to the MCU, this is the only way to inspect their live state — invaluable during
bring-up. It is what the app's `refsw` shell command uses.

## How it is kept separate from upstream

The upstream driver must not also build, or both would claim the same
devicetree nodes. Separation is by compatible string:

| | upstream | fork |
|---|---|---|
| compatible | `adi,tmc51xx` | `flintstone,tmc51xx` |
| Kconfig | `CONFIG_STEPPER_ADI_TMC51XX` | `CONFIG_CB_STEPPER_TMC51XX` |
| log module | `tmc51xx` | `cb_tmc51xx` |

The board DTS uses the `flintstone,*` compatibles, so `DT_HAS_ADI_TMC51XX_ENABLED`
is false and the in-tree driver is not compiled. The distinct log module name is
the quickest runtime check that the fork is what is running.

Internal function names are deliberately **left identical to upstream** to keep
the diff small and re-basing onto a newer Zephyr practical.

## Re-basing onto a newer Zephyr

1. Diff this directory against `zephyr/drivers/stepper/adi_tmc` to see the drift.
2. Re-copy the upstream files.
3. Re-apply the rename table above.
4. Re-apply the two `CB-FORK` changes.

Check first whether either fix has landed upstream — if both have, delete this
fork and go back to `adi,tmc51xx`.

## Not forked

Public headers (`stepper.h`, `stepper_ctrl.h`, `stepper_trinamic.h`) and the
shared bindings this one includes (`adi,trinamic-gconf.yaml`,
`adi,trinamic-ramp-generator.yaml`, `adi,trinamic-stallguard.yaml`) are still
used from the Zephyr tree unmodified.

## Gotcha this fork does *not* change

`stepper_ctrl_stop()` stops the motor by writing `VMAX = 0`, and nothing
restores it. Any later `run()` or `move_by()` is then clamped to zero speed and
never completes — it looks like a hang, not an error. Re-apply the ramp with
`stepper_ctrl_configure_ramp()` before each move; `apps/cam_slider` does this in
`apply_ramp()`. Left as upstream behaviour because it is how the TMC performs a
controlled stop in velocity mode.
