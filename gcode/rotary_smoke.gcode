; ---------------------------------------------------------------------------
; rotary_smoke.gcode
;
; Exercises the rotary R axis of the cartesianxy3zr kinematics: R on its own,
; R with the gantry, R with the bed, and R with everything at once.
;
; Runs only on a profile whose kinematics has an R axis.
; ---------------------------------------------------------------------------

G28                      ; home the bed (coordinated), then X, then Y
G90
M83
SET_KINEMATIC_POSITION R=0

; --- park the bed clear of the nozzle -------------------------------------
G1 Z50 A50 B50 F300

; --- 1. R alone -----------------------------------------------------------
G1 R90 F1200
G1 R-90 F1200
G1 R0 F1200

; --- 2. R with the gantry -------------------------------------------------
G1 X100 Y100 F3000
G1 X200 R45 F3000
G1 X100 R0 F3000

; --- 3. R with the bed ----------------------------------------------------
G1 Z60 A60 B60 R30 F300
G1 Z50 A50 B50 R0 F300

; --- 4. everything together -----------------------------------------------
G1 X150 Y120 Z55 A52 B58 R60 F1500
G1 X120 Y100 Z50 A50 B50 R0 F1500

; --- 5. a small rotation during an extrusion move -------------------------
G1 X130 Y110 E2 R10 F1200
G1 X140 Y120 E2 R20 F1200

; --- 6. back to a known pose ----------------------------------------------
G1 R0 F1200
G1 Z50 A50 B50 F300
M400
GET_POSITION
