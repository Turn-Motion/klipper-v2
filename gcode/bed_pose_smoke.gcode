; ---------------------------------------------------------------------------
; bed_pose_smoke.gcode
;
; Exercises the bed-pose motion path.  Uses only X Y Z A B E, so the SAME file
; runs against both kinematics profiles:
;
;   cartesianxy3z     - 5 axes, no horizontal carriages
;   cartesianxy3z2h   - same 5 axes, U/V derived internally from Z/A/B
;
; That is the point: adding the derived carriages does not change the G-code.
;
; Axes: X Y (gantry)  Z A B (vertical bed actuators)
; Derived (cartesianxy3z2h only): U V (horizontal bed-joint carriages)
; ---------------------------------------------------------------------------

G28                      ; home Z/A/B (coordinated), then X, then Y
G90
M83

; --- park the bed clear of the nozzle -------------------------------------
G1 Z50 A50 B50 F300

; --- 1. pure XY move (bed pose unchanged; U/V must not move) --------------
G1 X100 Y100 F3000
G1 X200 Y150 F3000

; --- 2. coordinated Z/A/B (pure height change, no tilt; U/V stay put) -----
G1 Z40 A40 B40 F300
G1 Z30 A30 B30 F300

; --- 3. PITCH only (A == B, Z differs) ------------------------------------
;     Plate rigidity says the horizontal carriages do NOT move for pitch.
G1 Z35 A30 B30 F300
G1 Z25 A30 B30 F300
G1 Z30 A30 B30 F300

; --- 4. ROLL (A != B) -- this is what actually drives U/V -----------------
G1 Z30 A20 B40 F300
G1 Z30 A10 B50 F300
G1 Z30 A30 B30 F300

; --- 5. combined pitch + roll ---------------------------------------------
G1 Z38 A22 B34 F300
G1 Z30 A30 B30 F300

; --- 6. all five axes in one move -----------------------------------------
G1 X250 Y180 Z28 A24 B32 F1200

; --- 7. mixed move with extrusion -----------------------------------------
G1 X260 Y185 Z27.8 A23.9 B31.9 E0.5 F1200
G1 X270 Y190 Z27.6 A23.8 B31.8 E0.5 F1200

; --- 8. arc in the XY plane, with the bed held tilted ---------------------
G1 X200 Y200 F3000
G2 X220 Y200 I10 J0 Z27 A23 B31 E0.3 F1200

; --- 9. extrude-only move --------------------------------------------------
G1 E1.0 F300

; --- 10. relative mode round-trip -----------------------------------------
G91
G1 X5 Y5 Z1 A1 B1 F1200
G90

; --- 11. G92 --------------------------------------------------------------
G92 E0
G1 E0.2 F300

; --- report ----------------------------------------------------------------
M114
GET_POSITION

; --- return to level and retreat ------------------------------------------
G1 Z60 A60 B60 F300
M400
