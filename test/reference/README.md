# Reference kinematics (vendored)

`printerkin/` is a verbatim copy of the Turn-Motion reference kinematics
package (`turn-kinematics/printerkin.zip`).  It is the authority on the
machine geometry and on the passive-joint solve.

It is vendored here **only as a test oracle** — nothing in `klippy/` imports
it, and it is never loaded at runtime.  `test/test_uv_derived.py` uses it to
verify that the derived U/V carriage positions computed by
`klippy/chelper/kin_cartesianxy3z2h.c` match the reference implementation.

Pure standard library, no numpy, no file I/O required.

If the machine geometry changes, replace this directory with the new
reference package and re-run `test/test_uv_derived.py`.
