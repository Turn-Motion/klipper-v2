; Proves the R axis actually travels, rather than ending where it started.
; Unlike rotary_smoke.gcode this deliberately finishes at a NON-ZERO angle,
; so the reported final position and the stepper count both have to show it.
;
; stepper_r: rotation_distance 360, gear_ratio 6:1, microsteps 16 on 200
; full steps = 19200 steps per 360 degrees = 53.3333 steps/degree.
; Finishing at R=45 must therefore report stepper_r:2400.
G28
G90
M83
SET_KINEMATIC_POSITION R=0
G1 Z50 A50 B50 F300
G1 R90 F1200
G1 R-90 F1200
G1 X100 Y100 F3000
G1 X200 R45 F3000
M400
GET_POSITION
