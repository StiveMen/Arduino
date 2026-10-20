# Sim_Rul V1 — requested fixes (work in progress)

This document records the requested behavior before modifying the working firmware. The existing pin assignments must not be changed without the user's explicit approval.

## Existing confirmed pin assignments

- Gas potentiometer: A1
- Brake potentiometer: A2
- Clutch potentiometer: A3
- Gearbox X potentiometer: A8
- Gearbox Y potentiometer: A9
- Encoder A: D0
- Encoder B: D1
- Ignition button: D5
- Steering center/zero limit switch: D6
- High-gear button: D7 (intended by the latest requested revision)

## Requested controls

- 12 physical buttons through a 3×4 matrix
- 1 ignition button
- 1 physical high-gear button
- 6 virtual gear buttons derived from gearbox X/Y potentiometer coordinates
- Total HID buttons: 20
- Gearbox X/Y are internal sensors and must not be published as HID axes.
- Exposed HID axes remain steering, gas, brake, and clutch.

## High-gear behavior

1. Pressing the physical high-gear button while entering a gear latches high gear on.
2. Releasing the physical button while remaining in that gear does not turn high gear off.
3. Leaving the gear turns the latched high gear off if the physical button is no longer held.
4. If the user is still holding the physical high-gear button when leaving the gear, high gear remains active.
5. Entering a gear without pressing the physical high-gear button must not activate high gear.

## Monitor mode

- Holding Ignition during startup after calibration selects serial monitor mode.
- The monitor continues reading and processing steering, pedals, gearbox coordinates, ignition, high-gear button and virtual gear states.
- No joystick HID state may be sent while monitor mode is active.
- Serial output should show steering, gas/brake/clutch values and calibration limits, gearbox X/Y coordinates, ignition state, physical high-gear button state and GEAR 1–6 states.

## Calibration and implementation constraints

- Preserve the current steering zero-calibration and first-run calibration flow unless a correction is necessary for the requested behavior.
- Do not assign or change matrix pins until the user confirms the actual three row pins and four column pins.
- The matrix wiring is still unconfirmed, so the complete matrix/HID implementation is not ready to commit to the production sketch.
- Compile and hardware testing have not yet been verified.
