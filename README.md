Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

The Klipper firmware controls 3d-Printers. It combines the power of a
general purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use the Klipper software.

Start by [installing Klipper software](https://www.klipper3d.org/Installation.html).

Klipper software is Free Software. See the [license](COPYING) or read
the [documentation](https://www.klipper3d.org/Overview.html). We
depend on the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).


# Load Cell Community Testing Branch

If you are testing load cells in klipper this is the branch to use!

Please review the documentation: [Load Cell](./docs/Load_Cell.md)

There is a debugging tool that will let you see the load cell output and probing taps:[Klipper Load Cell Debugging Tool](https://observablehq.com/@garethky/klipper-load-cell-debugging-tool). If you are testing you should get this set up to see how things are working.

#### Progress Tracker

This is being slowly shipped to klipper's mainline branch. It's a very large change and has to be broken up into multiple PRs to make reviewing easier:

##### PRs to klipper Mainline

1. ✅ [Bulk ADC Sensors](https://github.com/Klipper3d/klipper/pull/6555) - Shipped
2. ✅ [Add input_mux and vref options to ADS1220 sensor](https://github.com/Klipper3d/klipper/pull/6713) - Shipped
3. ☑️ [PR: Enable multiple z_thermal_adjust sections](https://github.com/Klipper3d/klipper/pull/6855) - Active
4. ✅ [Load cell gram scale](https://github.com/Klipper3d/klipper/pull/6729) - Shipped
5. ✅ [PR: Load Cell Probe](https://github.com/Klipper3d/klipper/pull/6871) - Shipped
6. ☑️ [PR: Load Cell Tap Analysis](https://github.com/Klipper3d/klipper/pull/) - Active
7. ☑️ [PR: Adaptive horizontal_move_z](https://github.com/Klipper3d/klipper/pull/6933]) - Active

##### PRs not yet submitted

2. `average_delta` metric for `PROBE_ACCURACY` command

---

## Feature Updates

### Default Relative Extruder Mode (04.08.2025)
Changed the default extruder coordinate system to use relative mode (M83) at startup instead of absolute mode. 

### CartesianXY3Z Safety Module Direct Import (04.08.2025)
The CartesianXY3Z kinematics now directly imports the safety module from extras/cartesianxy3z_safety.py without requiring a [cartesianxy3z_safety] section in printer.cfg. This allows safety checks to run automatically when the kinematics loads. The module logs which parameters are found in the config and which are using default values, making it easier to diagnose configuration issues. -- Need to be tested

### Z_tilt CartesianXY3Z Support (04.08.2025)
Enhanced Z_tilt module to properly support CartesianXY3Z kinematics by including A and B axis steppers in the Z tilt adjustment process. This allows for proper bed leveling on printers with multiple Z-axis motors including those on A and B axes. Added comprehensive debug logging to help diagnose stepper detection issues. -- Need to be tested

### Extruder Position Reset (04.08.2025)
Modified extruder movement handling to reset the last_position to 0 after processing each move. This prevents position accumulation issues and ensures proper relative extrusion behavior, fixing cases where extruder moves would update position tracking but not physically move the extruder motor. -- Need to be tested

### Post Homing Z Cancel (30.07.2025)
We '#' the post homing z lift so this will be done in the script.

### Extruder Movement Fix (30.07.2025)
Fixed an issue where extruder-only moves (e.g., G1 E50) would update position tracking but not physically move the extruder motor. The problem was caused by an early-return condition in the extruder's `check_move` function that incorrectly filtered out valid extruder movements when their axis ratio was very small. Force moves were unaffected as they bypass this check.

### IDEX Dual Carriage Position Preservation Fix (30.07.2025) -- Not resolved
Fixed a critical issue where toggling between T0 and T1 extruders in dual carriage setups would inadvertently reset the A axis position to zero. The position array handling in `toggle_active_dc_rail` was improved to properly preserve all axis positions (XYZABE) when switching carriages, ensuring that extruder-only moves maintain correct positions for all axes.

### IDEX Dual Carriage Homing Improvements (30.07.2025)
Added detailed debug logging for dual carriage homing to diagnose and fix issues with T0/T1 extruder homing in CartesianXY3Z kinematics.
Post Homing Z lift is now 60.

### CartesianXY3Z Coordinated Z Homing Enhancement (29.07.2025)
Implemented two-phase coordinated Z homing for Z/A/B axes with fast initial move, retraction, and individual precision homing, plus configurable post-homing Z lift.

### ADS1220 Driver Improvements (29.07.2025)

- Added robust SPI communication with retry logic and improved error handling
- Added detailed debug logging (enabled by default) to help diagnose communication issues
- Reduced default SPI speed for better reliability with problematic hardware
- Improved reset sequence with proper delays and verification
- Added configurable SPI speed option (`spi_speed` parameter)

---