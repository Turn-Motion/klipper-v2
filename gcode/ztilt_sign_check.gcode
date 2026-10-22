; ---------------------------------------------------------------------------
; ztilt_sign_check.gcode
;
; Drives a real Z_TILT_ADJUST through the manual probing path and checks
; that the correction removes the tilt instead of doubling it.
;
; Runs against sim/printer.sim.ztilt.cfg, which has [z_tilt] and no
; [probe].  Without a probe, Z_TILT_ADJUST asks for each trigger height
; interactively, so the heights below ARE the simulated bed: this file
; states what the bed looks like and the only question left is what the
; code does about it.
;
; Every probe point starts at horizontal_move_z, which the probe helper
; sets to exactly 5.000 before each point, so TESTZ can descend by a
; fixed amount and land on a known height.
;
; PASS 1 describes a bed whose B corner is 0.8 mm out of level.  The
; plane through the three heights below, evaluated at the three screw
; positions, is Z 0.000 / A 0.000 / B 0.800.  Removing the mean, the
; correction should be:
;
;     Z -0.266667   A -0.266667   B +0.533333
;
; PASS 2 describes the bed that correction produces: level, so every
; point triggers at the same height.  A correct implementation then has
; nothing left to do and the second correction should be zero.  With the
; sign inverted, pass 1 doubles the tilt instead and pass 2 is nowhere
; near zero.
;
; Read the result from the "z_tilt: physical ... gcode offsets ..." lines
; in klippy.log, and from the GET_POSITION reports here.
; ---------------------------------------------------------------------------

G28
G90
M83

; --- where we start from ---------------------------------------------------
GET_POSITION

; --- PASS 1: a bed 0.8 mm out of level at B --------------------------------
Z_TILT_ADJUST
; rear centre (300, 250) -> trigger at 0.096529
TESTZ Z=-4.903471
ACCEPT
; front left (100, 80) -> trigger at 0.073644
TESTZ Z=-4.926356
ACCEPT
; front right (500, 80) -> trigger at 0.569000
TESTZ Z=-4.431000
ACCEPT

GET_POSITION

; --- PASS 2: the bed that correction should have produced ------------------
Z_TILT_ADJUST
TESTZ Z=-5.000000
ACCEPT
TESTZ Z=-5.000000
ACCEPT
TESTZ Z=-5.000000
ACCEPT

GET_POSITION
M400
