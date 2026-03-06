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

### Probe Move Safety Bypass + Travel Limit (06.03.2026)
**Status: ✅ COMPLETE** — Safety module was blocking probe moves because the travel target (Z=A=B=−60) looked like a collision. The load cell stops the bed well before that point. Two fixes applied:

1. **Safety bypass during probing** — `cartesianxy3z.py` now skips collision checks while a probe move is active (same pattern as the existing homing bypass).
2. **Configurable travel limit** — probe target is capped by `probe_travel_limit` (default `−1.5`) instead of the full `z_min_position` (−60). Both `probe.py` and `load_cell_probe.py` respect this value.

**Config** — add to your `[load_cell_probe]` section in `printer.cfg`:
```ini
[load_cell_probe]
probe_travel_limit: -1.5   # max absolute Z/A/B target during a probe move (default: -1.5)
```

---

### CartesianXY3Z Safety — Extended Boundary Conditions & Extruder Geometry Parameters (23.02.2026)
**Status: ✅ COMPLETE** — Extended `cartesianxy3z_safety.py` with new secondary boundary checks and physical geometry parameters for more precise collision prevention.

**New Config Parameters (all configurable in `[CartesianXY3ZSafety]`):**

| Parameter | Key | Default | Description |
|---|---|---|---|
| `x_in` | `x_railspace_ab_inwards_neutral` | 50.0 mm | A/B rail inward travel limit at neutral bed position |
| `z_rail_ab` | `z_rail_ab_height_to_top` | 14.0 mm | Height from A/B rotation centre to top of rail |
| `y_rail_ab` | `y_rail_ab_innermost` | 43.5 mm | Innermost Y-point of A and B rails |
| `ex_x_a` | `extruder_width_tip_to_minus_x` | 15.5 mm | Extruder body width from tip in −X direction |
| `ex_x_b` | `extruder_width_tip_to_plus_x` | 5.5 mm | Extruder body width from tip in +X direction |
| `ex_y` | `extruder_width_tip_to_y` | 6.5 mm | Extruder body half-width in ±Y direction |
| `clear` | `safety_z_clearance` | 2.0 mm | Safety Z clearance above rail top |

**New Checks Added:**

- **A-wagon inner rail limit** — `inner_xa = csx/2 − d_x/2 − z_mz·z_ab/csx > x_in` stops the move when the A wagon would travel too far inward along its rail.
- **B-wagon inner rail limit** — `inner_xb = csx/2 − d_x/2 + z_mz·z_ab/csx > x_in` same check for the B wagon.
- **Extruder/rail proximity check** (replaces previous hardcoded Y ≥ 50 rule) — blocks moves where A or B is within `z_rail_ab + clear` of the rail top **and** the extruder body overhangs the rail zone in X **and** Y is closer than `y_rail_ab + ex_y` to the inner rail edge. This makes the restriction geometry-aware instead of using fixed constants.

**Bug fix:** The original inner-B check formula mistakenly evaluated `inner_xa` twice; corrected to evaluate `inner_xb` for the B-wagon check.

---

### CartesianXY3Z Safety Code Organization & Documentation (Current)
**Status: ✅ COMPLETE** — Reorganized and comprehensively documented the CartesianXY3Z safety module with 8 clearly labeled sections (`PRELIMINARY CALCULATIONS`, `PRIMARY BOUNDARY CONDITIONS [Level 0]`, `SECONDARY PRELIMINARY CALCULATIONS`, `PRIMARY MECHANICAL CONSTRAINTS [Level 1]`, `SECONDARY BOUNDARY CONDITIONS [Level 2]`, `TERTIARY CALCULATIONS`, `TERTIARY BOUNDARY CONDITION [Level 3]`, `ALL SAFETY CHECKS PASSED`) featuring visual dividers, inline comments, and step-by-step documentation. Created 7 documentation files: `CARTESIANXY3Z_SAFETY_QUICK_REFERENCE.md`, `CARTESIANXY3Z_SAFETY_CODE_ORGANIZATION.md`, `CARTESIANXY3Z_SAFETY_VISUAL_MAP.md`, `CARTESIANXY3Z_SAFETY_SECTION_POSTER.md`, `CARTESIANXY3Z_SAFETY_IMPLEMENTATION_SUMMARY.md`, `CARTESIANXY3Z_SAFETY_DOCUMENTATION_INDEX.md`, and `IMPLEMENTATION_COMPLETE.md`. Key features include self-documenting code with visual section headers, performance optimization through strategic check ordering (Position boundaries → Mechanical constraints → Wagon rail limits → 3D collision detection), fast-fail design for safety, production-ready code with no syntax errors, and comprehensive documentation enabling developers to quickly understand safety structure, easily locate and modify specific checks, and debug failures with clear error messages.

---

### Multi-Extruder Tool Change Position Synchronization Fix (24.11.2025)
Fixed a critical bug in `extruder.py` where switching back to a previously used extruder (e.g., T0 → T1 → T0) would cause "Internal error in stepcompress" failures.

**Root Cause:** When activating an extruder via `ACTIVATE_EXTRUDER` command, the toolhead's commanded position was updated but the extruder stepper's internal position was not synchronized with the extruder's `last_position`. This caused a position mismatch that led to invalid step calculations.

**The Bug Sequence:**
1. T0 active, extrudes to E=10mm → T0.last_position = 10, stepper position = 10
2. Switch to T1 → T0.last_position preserved at 10, but stepper position not updated
3. Switch back to T0 → toolhead commanded position set to 10, stepper still at old position
4. Next extrusion → **stepcompress error** due to position mismatch

**Why T1 Didn't Have This Issue:**
- T1 starts at last_position = 0 (never used before)
- Stepper internal position = 0 (default initialization)
- Both positions match, so first T1 activation works fine
- Only extruders being **reactivated** after previous use experienced the bug

**The Fix:**
```python
# In cmd_ACTIVATE_EXTRUDER (line 346-347)
if self.extruder_stepper is not None:
    self.extruder_stepper.stepper.set_position([self.last_position, 0., 0., 0., 0.])
```

This ensures the stepper's internal position is synchronized with `last_position` before every extruder activation, preventing position mismatches. Also updated `sync_to_extruder` to use 5 coordinates (XYZAB) for consistency with the 5-axis system.

### 5-Axis XYZAB Probe and Z-Tilt Coordination (11.09.2025)
Enhanced probe and Z-tilt systems to properly handle 5-axis XYZAB kinematics where A and B axes function as additional Z-axes for tilting bed systems.

**Key Improvements:**
- **Probe Movement Coordination**: Modified `probe.py` to automatically coordinate Z/A/B axes during all probe operations
- **Z-Tilt Enhancement**: Updated `z_tilt.py` to ensure coordinated movement of all Z-type axes during bed leveling
- **Automatic Detection**: System detects CartesianXY3Z kinematics (5+ rails) and applies coordination automatically
- **Movement Translation**: When Z movement is commanded (e.g., `[None, None, 5]`), it's automatically extended to `[None, None, 5, 5, 5]` for Z/A/B coordination

**Technical Details:**
- Fixed issue where probe movements showed `axes_d=[0,0,5,0,0,0]` instead of expected `axes_d=[0,0,5,5,5,0]`
- All probe lifting operations (`_raise_tool`, `_raise_tool_manual`) now move Z/A/B together
- Z-tilt stepper adjustments coordinate all Z-type axes to maintain bed level
- Added comprehensive debug logging to track coordination events

This ensures proper synchronized movement for tilting bed systems where Z, A, and B axes must move together to maintain bed orientation during probing and leveling operations.

### Improved Z-Axis Homing with Dynamic Position Limits (02.09.2025)
Enhanced the Z-axis homing process in CartesianXY3Z kinematics to use printer.cfg position limits:
- Initial Z/A/B positions now set to their respective `position_min` values from printer.cfg
- Force move distances now dynamically calculated using `position_max`/`position_min` values
- Proper handling of inverted Z-axis setups where the probe is fixed at the top

### Z_tilt CartesianXY3Z Support (02.09.2025)
Enhanced Z_tilt module to properly support CartesianXY3Z kinematics by including A and B axis steppers in the Z tilt adjustment process. This allows for proper bed leveling on printers with multiple Z-axis motors including those on A and B axes. Added comprehensive debug logging to help diagnose stepper detection issues. Implemented a temporary fix to detect steppers by name (stepper_z, stepper_a, stepper_b) when is_active_axis() method fails to correctly identify them.

**Fixed axis-specific movement issue (02.09.2025):**
- Fixed critical bug where Z_TILT_ADJUST would move Z axis positively but not A and B axes, causing out-of-range errors
- Added axis detection system that maps each stepper to its correct coordinate index (Z=2, A=3, B=4)
- Modified movement logic to update the appropriate coordinate for each stepper based on its axis
- Preserved original position during adjustment to avoid accumulating changes to the wrong axes
- Added detailed axis-specific logging for easier troubleshooting

This ensures all Z, A, and B steppers move correctly during tilt adjustment, preventing position errors and improving calibration reliability for printers with inverted Z-axis configurations.


### Move Object Safety Improvements (24.08.2025)
Fixed an AttributeError in CartesianXY3Z kinematics where safety check failures could cause crashes due to missing timing attributes:

**Problem:** When a move failed safety checks (e.g., "Extruder tip below printing plate - collision detected"), the move would be rejected before `set_junction` was called, causing an `AttributeError: 'Move' object has no attribute 'accel_t'` when the toolhead tried to process the move.

**Solution:**
1. Added default initialization of timing attributes in the Move constructor:
   ```python
   # Initialize timing attributes to safe defaults
   self.start_v = 0.
   self.cruise_v = 0.
   self.end_v = 0.
   self.accel_t = 0.
   self.cruise_t = 0.
   self.decel_t = 0.
   self._junction_set = False  # Track if set_junction was called
   ```
2. Modified CartesianXY3Z safety check handling to allow debugging by skipping safety checks while logging their status
3. Added detection and logging for moves using default timing values:
   ```python
   # Check for moves that never had set_junction called
   for i, move in enumerate(queue[:flush_count]):
       if not move._junction_set:
           gcode.respond_info(
               f"WARNING: Move using default timing values (set_junction never called). "
               f"Move {i+1}/{flush_count}, axes_d={[round(d, 4) for d in move.axes_d]}"
           )
   ```

This ensures that even when moves are rejected by safety checks, they still have the necessary attributes for safe processing, preventing crashes in the motion planning system. The added logging helps identify which moves are using default timing values instead of properly calculated ones.

### Debug Output Cleanup (20.08.2025)
Cleaned up verbose debug output across multiple modules to reduce console noise while preserving essential information:

**Files Modified:**
- `toolhead.py` - Commented out detailed move debug messages, kept "Processing x moves" 
- `gcode_move.py` - Commented out GCodeMove debug messages
- `homing.py` - Commented out detailed endstop/position debug messages
- `cartesianxy3z.py` - Commented out verbose homing debug messages  
- `extruder.py` - Commented out extruder move debug messages

**Console Output Now Shows Only:**
- "TOOLHEAD PROCESS_MOVES DEBUG: Processing x moves"
- "Starting/Completed individual precision homing for [axis]"
- Essential error messages

This significantly reduces log verbosity while maintaining visibility into key operations and error conditions.

### CartesianXY3Z Kinematic - Coordinated Movement Fix (20.08.2025)
Fixed sequential axis movement issue in CartesianXY3Z kinematics where Z, A, B axes would move separately from X, Y axes. The root cause was incorrect move distance calculation in `toolhead.py` that was including all axes in the distance formula.

**The Fix:** Modified `move_d` calculation to use the maximum distance among:
- XY combined distance: `sqrt(X² + Y²)`  
- Individual Z, A, B axis distances: `abs(Z)`, `abs(A)`, `abs(B)`

```python
move_d = max(xy_distance, z_distance, a_distance, b_distance)
```

**Additional improvement:** For moves where XY distance is less than 20% of the dominant axis, acceleration is reduced proportionally to ensure a visible cruise phase. This prevents small XY movements from being dominated by acceleration/deceleration.

This ensures:
- **Proper timing**: The axis with the largest movement dictates the move duration
- **Synchronized movement**: All axes move together with visible cruise phases
- **Smooth motion**: Small XY movements are no longer jerky or appear to happen only at the end

Perfect for tilting bed systems where Z, A, B axes often travel dominant distances while XY makes small adjustments.

### Reduced Logging / Ztilt Logging (06.08.2025)
- Reduced verbose debug logging in toolhead.py and extruder.py modules while preserving essential start/end logs for key code flows.
- Enhanced Ztilt logging about the Ztilt process, including the initial parameters, final parameters, and per-position adjustments.

### Reset Z Tilt (05.08.2025)
- Fixed stepper detection for Z, A, and B axes using name-based identification when `is_active_axis()` fails
- Reset Z-tilt to use conventional method, just improve the stepper detection.

### Division by Zero Protection in Motion Planning (04.08.2025)
- Added safety checks in toolhead.py to prevent division by zero errors in motion timing calculations

### Default Relative Extruder Mode (04.08.2025)
Changed the default extruder coordinate system to use relative mode (M83) at startup instead of absolute mode. 

### CartesianXY3Z Safety Module Direct Import (04.08.2025)
The CartesianXY3Z kinematics now directly imports the safety module from extras/cartesianxy3z_safety.py without requiring a [cartesianxy3z_safety] section in printer.cfg. This allows safety checks to run automatically when the kinematics loads. The module logs which parameters are found in the config and which are using default values, making it easier to diagnose configuration issues. Fixed compatibility issues with ConfigWrapper to properly detect parameters using Klipper's sentinel pattern instead of has_option(). -- Need to be tested - latest update

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