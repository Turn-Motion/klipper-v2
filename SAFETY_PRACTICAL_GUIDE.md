# CartesianXY3Z Safety - Practical Guide & Recommendations

## Quick Reference

### Current Status
| Check | Status | Details |
|-------|--------|---------|
| **Safety module loads?** | ? Yes | Successfully instantiated during kinematics init |
| **Safety module functions exist?** | ? Yes | All collision detection methods implemented |
| **Safety checks execute?** | ? **NO** | **Line 204 in cartesianxy3z.py is commented out** |
| **Can accept dangerous coordinates?** | ? Yes | Unsafe moves are NOT rejected |
| **Manual testing available?** | ? Yes | `CHECK_CARTESIANXY3Z_SAFETY` command works |

---

## Why Safety Checks Are Disabled

### Code Location: `klippy/kinematics/cartesianxy3z.py`, lines 204-207

```python
try:
    # DEBUG: Skip actual safety check but log what would have happened
    # self.safety_module.check_move(move)  ? **DISABLED**
    logging.info("CartesianXY3Z: Safety check SKIPPED for debugging")
    gcode = self.printer.lookup_object('gcode')
    gcode.respond_info("CartesianXY3Z Safety: Check SKIPPED (debug mode)")
except Exception as e:
    logging.warning("CartesianXY3Z: Unexpected error in skipped safety check: %s", str(e))
```

### What This Means
- **Intentional**: Marked as "DEBUG" - this is NOT a bug, it's deliberate
- **Temporary**: Suggests this is for development/testing
- **Non-blocking**: The code logs but doesn't prevent moves
- **Configuration-independent**: Even with `[cartesianxy3z_safety]` configured, checks don't run

---

## How the Safety System Works (Detailed)

### Mathematical Model

Your 5-axis printer is modeled as an inverted Stewart platform:

```
Fixed frame:
    ?????????????????????????????????????????
  ?             ?
    ?    Z-Joint (P_Z)      ?
    ?    at center-top (250, 250, 445mm)    ?
    ?           ? ?
    ?         / ? \        ?
    ?        /  ?  \       ?
    ?       / d_y?d_y\  (521mm each)     ?
    ?      /     ?     \        ?
    ?  P_A ????????????? P_B        ?
    ?   ?    ?    ?          ?
  ?   ?  d_x=516mm ?      ?
    ?   ?        ?? ?
    ?   ???????????????????     ?
    ? Tilting Plate         ?
    ?(with P_E below)       ?
?               ?
    ?????????????????????????????????????????

Key measurements:
  • d_x = 516mm (distance between A and B joints, X-direction)
  • d_y = 521mm (distance from Z to A/B line, Y-direction)
  • Maximum tilt limited by joint reach
  • Extruder tip (P_E) must stay above the plate
```

### What Gets Checked

#### **1. Joint Distance Constraints**
```
A-B Joint Separation Limit:
  |b - a| ? d_x (516mm)
  
  If violated: "A-B distance exceeded"
  
  Reason: Physical limit of linkage, ball joints can't
          separate beyond design distance

Z-A Distance Limit:
  |z - a| ? sqrt(d_y² - (d_y - y_in)²)
  
  If violated: "Z-A distance exceeded"
  
  Reason: Stewart platform geometry constraint

Z-B Distance Limit:
  |z - b| ? sqrt(d_y² - (d_y - y_in)²)
  
  If violated: "Z-B distance exceeded"
  
  Reason: Mirror constraint for B-axis
```

#### **2. Rail Boundary Constraints**
```
A/B Wagon Rail Limits:
  Wagons can't move beyond rail ends
  (x_in = 117.5mm inward, x_out = 59.5mm outward)
  
  If violated: "A/B wagon approaching rail limit"
  
Z Wagon Rail Limits:
  Z wagon can't move beyond rail ends
  (y_in = 101.1mm from center)
  
  If violated: "Z wagon approaching rail limit"
```

#### **3. Mechanical Collision Constraints**
```
Plate-to-Rail Collision:
  Printing plate can't hit the X-rail of A/B joints
  Check combines:
    • Z tilt angle (z_mz)
    • A-B separation (z_ab)
    • Required clearance with offsets (L_off, y_off)
  
  If violated: "Potential plate collision with x-rail"
  
Extruder-to-Plate Collision:
  Extruder tip (P_E) can't go below the printing plate
  Uses 3D vector dot product to detect penetration:
    
    1. Calculate plate normal vector: z_now
    2. Calculate vector from plate point to P_E
    3. Dot product tells if P_E is below plate
    4. If dot_product < 0: collision!
  
  If violated: "Extruder tip below printing plate - collision detected"
```

---

## Practical Examples

### Example 1: Safe Move ?

```gcode
G28        ; Home all axes
G1 X250 Y250 Z300 A150 B350 F6000

Analysis:
  • X=250, Y=250: Center of print area (safe)
  • Z=300: Mid-range height
  • A=150, B=350: Reasonable tilt configuration

Safety checks (if enabled):
  ? |B - A| = 200 < 516 (OK)
  ? Z is reachable from A and B
  ? Wagons within rail limits
  ? No plate collisions detected
  ? Extruder above plate
  
Result: MOVE ACCEPTED ?
```

### Example 2: Out of Reach ?

```gcode
G1 X250 Y250 Z500 A100 B100 F6000

Analysis:
  • Z=500: Very high
  • A=100, B=100: Very close together
  • Trying to lift plate above maximum reach
  
Safety checks (if enabled):
  ? |Z - A| = 400 > max_reach ? 315
  
Result: MOVE REJECTED ?
Error: "Z-A distance exceeded"
```

### Example 3: Plate Collision ?

```gcode
G1 X250 Y250 Z200 A100 B500 F6000

Analysis:
  • Z=200: Too low
  • A=100, B=500: Maximum separation tilts bed
  • This would drive plate down into Z joint area
  
Safety checks (if enabled):
  ? Collision detected between plate and x-rail
  
Result: MOVE REJECTED ?
Error: "Potential plate collision with x-rail"
```

### Example 4: Extruder Below Plate ?

```gcode
G1 X250 Y250 Z100 A200 B350 F6000

Analysis:
  • Z=100: Too low (extruder tip at z_out = 10mm)
  • With tilting, extruder would go below plate level
  
Safety checks (if enabled):
  ? Dot product < 0 (extruder penetrating plate)
  
Result: MOVE REJECTED ?
Error: "Extruder tip below printing plate - collision detected"
```

---

## Configuration Requirements

### What Parameters Do What

```ini
[cartesianxy3z_safety]

; PHYSICAL HARDWARE PARAMETERS
; These describe your actual machine dimensions

joint_ball_diameter = 12.0
  ? Size of ball joints connecting linkage
  ? Used in: Collision detection clearances

wagon_length = 45.0
  ? Length of sliding wagons on rails
  ? Used in: Rail constraint calculations

wagon_rail_z_offset = 5.0
  ? Height offset of Z-axis wagon rails
  ? Used in: Z joint position calculation

x_distance_between_joints_ab = 516.0
  ? Distance between A and B ball joints (X-axis)
  ? Used in: Maximum A-B separation check
  ? CRITICAL: Must match your actual hardware!

y_distance_plate_to_joint_z = 521.0
  ? Distance from plate center to Z joint (Y-axis)
  ? Used in: Reachability calculations
  ? CRITICAL: Must match your actual hardware!

y_overhang_plate_to_joints_ab = 5.0
  ? How far plate extends past A/B joints toward Z
  ? Used in: Collision detection calculations

plate_thickness = 9.9
  ? Thickness of printing plate
  ? Used in: Collision detection z-offset


; DERIVED PARAMETERS (from your printer mechanical design)
; These are usually fixed once calibrated

joint_ball_inset = 4.0
  ? How deep ball sits into ball cup on plate
  ? Used in: Plate collision calculations

alu_mount_plate_thickness = 6.0
  ? Thickness of aluminum mounting plate
  ? Used in: Z joint height calculation

joint_mount_thickness = 5.0
  ? Thickness of joint mounting bracket
  ? Used in: Z joint height calculation

y_distance_joints_ab_to_rail = 14.0
  ? Clearance from A/B joints to their mounting rail
  ? Used in: Plate collision detection

x_railspace_ab_inwards = 117.5
  ? How far A/B wagons can move inward (toward center)
  ? Used in: Rail limit checks

x_railspace_ab_outwards = 59.5
  ? How far A/B wagons can move outward (away from center)
  ? Used in: Rail limit checks

y_railspace_z_inwards = 101.1
  ? How far Z wagon can move inward
  ? Used in: Rail limit checks

total_plate_height = 44.15
  ? Total height of moving plate assembly
  ? Used in: Collision detection calculations
```

### What Happens Without Configuration

If you **don't** add `[cartesianxy3z_safety]` section:
1. Module loads with hardcoded defaults
2. Defaults are close to example values
3. Logger reports which params are defaults
4. Even with defaults, safety is **still disabled** (line 204 issue)

### Minimum Configuration

```ini
[cartesianxy3z_safety]
; At minimum, specify your hardware-specific values
joint_ball_diameter = 12.0
x_distance_between_joints_ab = 516.0    ; YOUR ACTUAL VALUE
y_distance_plate_to_joint_z = 521.0   ; YOUR ACTUAL VALUE
```

---

## Enabling Safety (Step-by-Step)

### Step 1: Edit cartesianxy3z.py

**File**: `klippy/kinematics/cartesianxy3z.py`  
**Line**: 204

Change from:
```python
# DEBUG: Skip actual safety check but log what would have happened
# self.safety_module.check_move(move)
logging.info("CartesianXY3Z: Safety check SKIPPED for debugging")
```

To:
```python
# Safety checks enabled
self.safety_module.check_move(move)
logging.info("CartesianXY3Z: Safety check ENABLED")
```

### Step 2: Add Configuration (Optional but Recommended)

Create or edit your `printer.cfg`:

```ini
[cartesianxy3z_safety]
# Your hardware-specific parameters
joint_ball_diameter = 12.0
wagon_length = 45.0
wagon_rail_z_offset = 5.0
x_distance_between_joints_ab = 516.0
y_distance_plate_to_joint_z = 521.0
y_overhang_plate_to_joints_ab = 5.0
plate_thickness = 9.9
joint_ball_inset = 4.0
alu_mount_plate_thickness = 6.0
joint_mount_thickness = 5.0
y_distance_joints_ab_to_rail = 14.0
x_railspace_ab_inwards = 117.5
x_railspace_ab_outwards = 59.5
y_railspace_z_inwards = 101.1
total_plate_height = 44.15
```

### Step 3: Restart Klipper

```bash
sudo systemctl restart klipper
```

### Step 4: Verify Activation

Check logs:
```bash
grep -i "safety" /tmp/klippy.log | tail -20
```

You should see:
```
CartesianXY3Z: Initializing safety module
CartesianXY3Z Safety: Found X parameters in config
CartesianXY3Z Safety: Using defaults for Y parameters
CartesianXY3Z: Safety module loaded successfully
```

### Step 5: Test Manually

In Mainsail console, send:
```gcode
CHECK_CARTESIANXY3Z_SAFETY
```

Expected output:
```
Current position is safe
Intermediate values:
  z_mz: 0.000, z_ab: 0.000
  csz: 1.000, csx: 516.000
  i_ab: 1, x_ab: 0.000

Point positions:
  P_Z: (250.000, 250.000, 445.000)
  P_A: (125.000, 150.000, 0.000)
  P_B: (375.000, 150.000, 0.000)
  P_E: (250.000, 250.000, 10.000)
```

### Step 6: Test with Unsafe Coordinates

Try a move that should fail:
```gcode
G1 X250 Y250 Z100 A100 B100 F6000
```

Expected: Move rejected with error message
```
Error: Z-A distance exceeded
```

---

## Debugging Safety Issues

### Check 1: Is Module Loaded?

```bash
grep "CartesianXY3Z" /tmp/klippy.log | grep -i "safety\|load"
```

Expected output:
```
CartesianXY3Z: Initializing safety module
CartesianXY3Z: Safety module loaded successfully
```

If NOT present: Module failed to load (check Python syntax in your edits)

### Check 2: Is Configuration Read?

```bash
grep "CartesianXY3Z Safety:" /tmp/klippy.log
```

Expected output:
```
CartesianXY3Z Safety: Found 17 parameters in config
CartesianXY3Z Safety: Using defaults for 0 parameters
```

If defaults showing: Your `[cartesianxy3z_safety]` section not found

### Check 3: Are Checks Running?

```bash
grep "Safety check" /tmp/klippy.log | tail -5
```

Current (debug mode):
```
CartesianXY3Z: Safety check SKIPPED for debugging
```

After fix:
```
Should see no message (checks run silently if safe)
```

### Check 4: Manual Safety Status

In Mainsail console:
```gcode
G28  ; Home everything
CHECK_CARTESIANXY3Z_SAFETY
```

This will show:
- Is current position safe?
- Detailed intermediate values
- All calculated point positions
- All vectors and normals

Use these values to debug geometric issues.

---

## Mainsail Integration

### Display Safety Status in Mainsail

Add to your macro:
```jinja2
[gcode_macro SAFETY_CHECK]
gcode:
    CHECK_CARTESIANXY3Z_SAFETY
    {% if printer.cartesianxy3z_safety.is_safe %}
        M117 ? Position is SAFE
    {% else %}
        M117 ? UNSAFE: {{ printer.cartesianxy3z_safety.last_error }}
    {% endif %}
```

### Pre-Print Safety Verification

```jinja2
[gcode_macro START_PRINT]
gcode:
 G28  ; Home
    CHECK_CARTESIANXY3Z_SAFETY
    {% if not printer.cartesianxy3z_safety.is_safe %}
  RESPOND TYPE=error MSG="Safety check failed: cannot start print"
        M112  ; Emergency stop
    {% endif %}
    ; ... rest of start routine ...
```

### Emergency Safety Override (Development Only)

If you need to temporarily bypass checks for troubleshooting:

```gcode
; Move to position while SKIPPING safety (dangerous!)
; Uncomment line 204 to re-enable checks
G1 X100 Y100 Z500 A100 B100 F6000
```

**WARNING**: Only do this if:
1. You understand the geometry
2. You've verified safe manually
3. You're actively monitoring motion
4. This is temporary/testing only

---

## When Safety Triggers

### What Happens When a Move is Unsafe

1. **Move requested**: User sends `G1 X500 Y500 Z10 A100 B100 F6000`

2. **Safety check runs** (if enabled):
   - Calculates joint positions
   - Checks geometric constraints
   - Evaluates collision scenarios
   - Gets to "Extruder tip below printing plate" ?

3. **Exception raised**: `move.move_error("Extruder tip below printing plate - collision detected")`

4. **Error propagates**:
   - Caught by Klipper error handler
   - Reported to user
   - Move NOT executed
   - No stepper commands sent to MCU

5. **User sees**: 
   - In Mainsail: Red error notification
   - In logs: `Error: Extruder tip below printing plate - collision detected`
   - Printer remains at safe position

6. **Recovery**: User must enter a safe coordinate instead

---

## Summary Table

| Feature | Status | Enabled? | Can Fix? |
|---------|--------|----------|----------|
| Safety module code | ? Present | N/A | N/A |
| Safety calculations | ? Implemented | N/A | N/A |
| Boundary checks | ? Implemented | ? No | ? Yes |
| Collision detection | ? Implemented | ? No | ? Yes |
| Configuration parsing | ? Implemented | ? Yes | N/A |
| Manual check command | ? Implemented | ? Yes | N/A |
| Automatic move rejection | ? Disabled | ? No | ? Yes |
| Mainsail integration | ? Ready | ? No* | ? Yes |

*Requires enabling safety checks first

---

## Recommended Next Steps

### Immediate (Without Code Changes)

1. ? Review your hardware specifications
2. ? Configure `[cartesianxy3z_safety]` with accurate parameters
3. ? Test with `CHECK_CARTESIANXY3Z_SAFETY` command
4. ? Understand the intermediate values and point positions

### Short Term (Enable Safety)

1. ? Edit line 204 in `cartesianxy3z.py` to uncomment the safety check
2. ? Restart Klipper
3. ? Test with safe coordinates first
4. ? Test with unsafe coordinates - should be rejected

### Medium Term (Full Integration)

1. ? Create macro in Mainsail to display safety status
2. ? Add pre-print safety verification
3. ? Document your hardware-specific parameters
4. ? Create emergency recovery procedures

### Long Term (Optimization)

1. ? Tune safety margins if needed
2. ? Test edge cases in your print area
3. ? Monitor logged safety events
4. ? Consider upstream PR to Klipper (remove debug comment)

---

