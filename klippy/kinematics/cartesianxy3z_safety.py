# Safety checks for cartesianxy3z kinematics with tilting bed
import logging
import math


def add(a, b):
    return [x + y for x, y in zip(a, b)]

def sub(a, b):
    return [x - y for x, y in zip(a, b)]

def mul(v, s):
    return [x * s for x in v]

def div(v, s):
    return [x / s for x in v]

def dot(a, b):
    return sum(x * y for x, y in zip(a, b))

def cross(a, b):
    ax, ay, az = a
    bx, by, bz = b
    return [ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx]

def norm(a):
    return math.sqrt(dot(a, a))


class CartesianXY3ZSafety:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.is_safe = True
        self.last_error = ""
        # Read parameters from [CartesianXY3ZSafety] section if defined in
        # printer.cfg; otherwise every parameter keeps its hardcoded default.
        self.disable_safety_checks = False
        if config.has_section('CartesianXY3ZSafety'):
            sc = config.getsection('CartesianXY3ZSafety')
            self.disable_safety_checks = sc.getboolean('disable_safety_checks', False)
            if self.disable_safety_checks:
                logging.warning("CartesianXY3ZSafety: Safety checks are DISABLED via config")
            self.d_y       = sc.getfloat('y_distance_between_ab_center_and_z', 304.6)
            self.h_A       = sc.getfloat('a_joint_probe_object_height', 28.0)
            self.z_probe_A = sc.getfloat('a_joint_probe_value', 7.38125)
            self.z_probe_P = sc.getfloat('bed_probe_value', -0.35625)
            self.p_x       = sc.getfloat('printing_area_x_length', 600.0)
            self.p_y       = sc.getfloat('printing_area_y_length', 300.0)
            self.j_out     = sc.getfloat('joint_plate_horizontal_distance', 23.0)
            self.x_out    = sc.getfloat('x_railspace_ab_outwards_neutral', 255.0)
            self.x_in     = sc.getfloat('x_railspace_ab_inwards_neutral', 50.0)
            self.y_out    = sc.getfloat('y_railspace_z_inwards_neutral', 132.19)
            self.r_ex     = sc.getfloat('extruder_radius', 0.5)
            self.z_rail_ab= sc.getfloat('z_rail_ab_height_to_top', 14.0)
            self.y_rail_ab= sc.getfloat('y_rail_ab_innermost', 43.5)
            self.ex_x_a   = sc.getfloat('extruder_width_tip_to_minus_x', 15.5)
            self.ex_x_b   = sc.getfloat('extruder_width_tip_to_plus_x', 5.5)
            self.ex_y     = sc.getfloat('extruder_width_tip_to_y', 33.5)
            self.clear    = sc.getfloat('safety_z_clearance', 2.0)
            #self.d_z_ab = sc.getfloat('height_distance_between_z_ab', 250.0)
        else:
            self.d_y       = 304.6
            self.h_A       = 28.0
            self.z_probe_A = 7.38125
            self.z_probe_P = -0.35625
            self.p_x       = 600.0
            self.p_y       = 300.0
            self.j_out     = 23.0
            self.x_out    = 255.0
            self.x_in     = 50.0
            self.y_out    = 132.19
            self.r_ex     = 0.5
            self.z_rail_ab= 14.0
            self.y_rail_ab= 43.5
            self.ex_x_a   = 15.5
            self.ex_x_b   = 5.5
            self.ex_y     = 6.5
            self.clear    = 2.0
            #self.d_z_ab = 250.0
        self.d_x      = self.p_x + 2 * self.j_out
        self.z_tot    = self.h_A / 2 - (self.z_probe_A - self.z_probe_P)
        self.y_overAB = self.d_y - (self.p_y + self.j_out)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.gcode.register_command('CHECK_CARTESIANXY3Z_SAFETY',
            self.cmd_CHECK_CARTESIANXY3Z_SAFETY,
            desc=self.cmd_CHECK_CARTESIANXY3Z_SAFETY_help)
        self.gcode.register_command('PREVIEW_MOVE',
            self.cmd_PREVIEW_MOVE,
            desc=self.cmd_PREVIEW_MOVE_help)

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move', None)

    def _gcode_pos(self, pos):
        """Convert motor-space position back to gcode-space by stripping
        all accumulated gcode offsets (probe z_offset, z_tilt, SET_GCODE_OFFSET).
        Safety thresholds are calibrated in gcode-coordinate space, so this
        must be applied before any check_safety() call.
        The actual move sent to the firmware is NOT touched.
        """
        if self.gcode_move is None:
            return list(pos)
        origin = self.gcode_move.get_status()['homing_origin']
        adjusted = list(pos)
        adjusted[0] -= origin.x
        adjusted[1] -= origin.y
        adjusted[2] -= origin.z
        adjusted[3] -= origin.a
        adjusted[4] -= origin.b
        return adjusted

    def calc_intermediate_values(self, pos):
        # ============================================================================
        # Real-time CALCULATIONS
        # ============================================================================
        # Extract position coordinates
        x, y = pos[0], pos[1]
        z, a, b = pos[2], pos[3], pos[4]
        z_mz = z - (a + b) / 2
        z_ab = b - a
        if abs(z_ab) >= self.d_x:
            csx = 0.0
        else:
            csx = math.sqrt(self.d_x**2 - z_ab**2)
        i_ab = -1 if a > b else 1
        if csx == 0:
            x_ab = 0.0
        else:
            x_ab = (csx - self.d_x) / 2 - i_ab * z_mz * z_ab / csx
        return z_mz, z_ab, csx, i_ab, x_ab

    def calc_point_positions(self, pos):
        x, y = pos[0], pos[1]
        z, a, b = pos[2], pos[3], pos[4]
        z_mz, z_ab, csx, i_ab, x_ab = self.calc_intermediate_values(pos)
        P_Z_x = self.p_x / 2
        if abs(z_ab) >= self.d_x:
            P_Z_y = -self.y_overAB
        else:
            denom = 1 - (z_ab**2) / (self.d_x**2)
            if denom <= 0:
                P_Z_y = -self.y_overAB
            else:
                P_Z_y = math.sqrt(self.d_y**2 - z_mz**2 / denom) - self.y_overAB
        P_Z_z = -z
        if csx == 0:
            P_A_x = self.p_x / 2
        else:
            P_A_x = z_mz * z_ab / csx - csx / 2 + self.p_x / 2
        P_A_y = -self.y_overAB
        P_A_z = -a
        if csx == 0:
            P_B_x = self.p_x / 2
        else:
            P_B_x = z_mz * z_ab / csx + csx / 2 + self.p_x / 2
        P_B_y = -self.y_overAB
        P_B_z = -b
        P_E_x = x
        P_E_y = y
        P_E_z = self.z_tot
        return {
            'P_Z': (P_Z_x, P_Z_y, P_Z_z),
            'P_A': (P_A_x, P_A_y, P_A_z),
            'P_B': (P_B_x, P_B_y, P_B_z),
            'P_E': (P_E_x, P_E_y, P_E_z)
        }

    def calc_vectors(self, points):
        P_A = points['P_A']
        P_B = points['P_B']
        P_Z = points['P_Z']
        x_now = sub(P_B, P_A)
        y_now = sub(P_Z, div(add(P_A, P_B), 2))
        z_now = cross(x_now, y_now)
        z_norm = div(z_now, norm(z_now))
        return x_now, y_now, z_now, z_norm

    def check_safety(self, pos):
        """Check all safety conditions for the given position."""
        try:
            # ============================================================================
            # PRELIMINARY CALCULATIONS
            # ============================================================================
            # Extract position coordinates
            x, y = pos[0], pos[1]
            z, a, b = pos[2], pos[3], pos[4]

            # Note: X/Y boundary checks are handled by _check_endstops()
            # in cartesianxy3z.py using printer.cfg axis limits.
            # This module only checks Z/A/B mechanical constraints.

            # ============================================================================
            # PRELIMINARY CALCULATIONS
            # ============================================================================
            # Calculate intermediate values needed for remaining checks
            z_mz, z_ab, csx, i_ab, x_ab = self.calc_intermediate_values(pos)

            # ============================================================================
            # PRIMARY MECHANICAL CONSTRAINTS
            # ============================================================================
            # Level 1: Primary boundary conditions - mechanical joint distances
            if abs(z - a) > math.sqrt(self.d_y**2 + self.d_x**2 / 4):
                return False, "Z-A distance exceeded"
            if abs(z - b) > math.sqrt(self.d_y**2 + self.d_x**2 / 4):
                return False, "Z-B distance exceeded"
            if abs(z - a) > math.sqrt(self.d_y**2 + self.d_x**2 / 4):
                return False, "Z-A distance exceeded"
               
            # ============================================================================
            # SECONDARY BOUNDARY CONDITIONS - GOING OFF RAILS
            # ============================================================================
            # Level 2: Wagon rail limit checks
            # Check A wagon X-position limit
            if csx > 0:
                max_xa = abs(z_mz * z_ab / csx - csx / 2 + self.d_x / 2)
                if max_xa > self.x_out:
                    return False, "A wagon approaching rail limit"

            # Check B wagon X-position limit
            if csx > 0:
                max_xb = abs(z_mz * z_ab / csx + csx / 2 - self.d_x / 2)
                if max_xb > self.x_out:
                    return False, "B wagon approaching rail limit"

            # Check A-rail wagon inner X-position limit
            if csx > 0:
                inner_xa = csx / 2 - self.d_x / 2 - z_mz * z_ab / csx
                if inner_xa > self.x_in:
                    return False, "A wagon approaching inner rail limit"

            # Check B-rail wagon inner X-position limit
            if csx > 0:
                inner_xb = csx / 2 - self.d_x / 2 + z_mz * z_ab / csx
                if inner_xb > self.x_in:
                    return False, "B wagon approaching inner rail limit"

            # Check Z wagon Y-position limit
            denom_z = 1 - (z_ab**2) / (self.d_x**2)
            if denom_z > 0:
                max_y = abs(math.sqrt(self.d_y**2 - z_mz**2 / denom_z) - self.d_y)
            else:
                max_y = float('inf')
            if max_y > self.y_out:
                return False, "Z wagon approaching rail limit"
            
            # ============================================================================
            # TERTIARY CALCULATIONS - COLLISION DETECTION SETUP
            # ============================================================================
            # Calculate point positions (P_Z, P_A, P_B, P_E)
            points = self.calc_point_positions(pos)
            # Calculate orientation vectors of the tilted bed plane
            _, _, _, z_norm = self.calc_vectors(points)

            # ============================================================================
            # TERTIARY BOUNDARY CONDITION: COLLISION OF EXTRUDER TIP WITH BED
            # ============================================================================
            # Level 3: Extruder tip collision detection with tilting print bed
            P_E = points['P_E']
            P_A = points['P_A']
            # dist = dot(sub(P_E, add(P_A, mul(z_norm, z_tot))), z_norm)
            #        - r_ex * sin(acos(z_norm[2]))
            dist = (dot(sub(P_E, add(P_A, mul(z_norm, self.z_tot))), z_norm)
                    - self.r_ex * math.sin(math.acos(z_norm[2])))
            if dist < 0:
                return False, "Extruder collision with print bed detected"
            # ============================================================================
            # A/B DISCREPANCY LIMIT WHEN Z IS OFF-CENTRE
            # ============================================================================
            # z_mz = z - (a+b)/2; if Z is not at the midpoint of A and B,
            # the A-B discrepancy (z_ab = b - a) must stay under 15 mm.
            if abs(z_mz) > 0.1 and abs(z_ab) >= 15:
                return False, (
                    "A-B discrepancy too large when Z is off-centre "
                    "(|A-B|=%.2f >= 15, z_mz=%.2f)" % (abs(z_ab), z_mz))

            # ============================================================================
            # PRINTING AREA Y RESTRICTION
            # ============================================================================
            # When A or B is near rail top and extruder overhangs in X,
            # Y must stay clear of the innermost rail edge.
            if (a < (self.z_rail_ab + self.clear) or b < (self.z_rail_ab + self.clear)) \
                    and y < (self.y_rail_ab + self.ex_y):
                return False, (
                    "Extruder too close to A/B rail: A=%.2f, B=%.2f, X=%.2f, Y=%.2f "
                    "(limit: z_rail+clear=%.2f, y_rail+ex_y=%.2f)"
                    % (a, b, x, y,
                       self.z_rail_ab + self.clear,
                       self.y_rail_ab + self.ex_y))

            # ============================================================================
            # ALL SAFETY CHECKS PASSED
            # ============================================================================
            return True, ""
        except Exception as e:
            return False, f"Safety check error: {str(e)}"

    def check_move(self, move):
        """Check if a move is safe."""
        if self.disable_safety_checks:
            return True
        try:
            if not any(move.axes_d[i] for i in [2, 3, 4]):
                # Pure XY move - check Y restriction using current A/B positions
                is_safe, error = self.check_safety(self._gcode_pos(move.end_pos))
                if not is_safe:
                    self.is_safe = False
                    self.last_error = error
                    logging.warning(f"CartesianXY3Z Safety: {error} (XY move end position)")
                    self.gcode.respond_info(f"Safety Error: {error}")
                    return False
                return True
            # A/B Discrepancy Check — coupling:
            # A and B must always move together; one cannot move without the other.
            a_moving = abs(move.axes_d[3]) >= 1.0
            b_moving = abs(move.axes_d[4]) >= 1.0
            if a_moving != b_moving:
                solo_dist = max(abs(move.axes_d[3]), abs(move.axes_d[4]))
                if solo_dist > 2.0:
                    error = "A and B must move together (cannot move one without the other)"
                    self.is_safe = False
                    self.last_error = error
                    logging.warning(f"CartesianXY3Z Safety: {error}")
                    self.gcode.respond_info(f"Safety Error: {error}")
                    return False
                ab_gap = abs(move.end_pos[3] - move.end_pos[4])
                if ab_gap > 4.0:
                    error = f"A/B gap after solo move would be {ab_gap:.2f}mm (max 4mm)"
                    self.is_safe = False
                    self.last_error = error
                    logging.warning(f"CartesianXY3Z Safety: {error}")
                    self.gcode.respond_info(f"Safety Error: {error}")
                    return False
            # Skip start_pos check: the machine is already there (e.g. after a
            # probe touch), so blocking moves FROM that position is wrong.
            is_safe, error = self.check_safety(self._gcode_pos(move.end_pos))
            if not is_safe:
                self.is_safe = False
                self.last_error = error
                logging.warning(f"CartesianXY3Z Safety: {error} (end position)")
                self.gcode.respond_info(f"Safety Error: {error}")
                return False
            if move.move_d > 1.0:
                midpos = [(move.start_pos[i] + move.end_pos[i]) / 2 for i in range(len(move.start_pos))]
                is_safe, error = self.check_safety(self._gcode_pos(midpos))
                if not is_safe:
                    self.is_safe = False
                    self.last_error = error
                    logging.warning(f"CartesianXY3Z Safety: {error} (intermediate position)")
                    self.gcode.respond_info(f"Safety Error: {error}")
                    return False
            self.is_safe = True
            self.last_error = ""
            return True
        except Exception as e:
            self.is_safe = False
            self.last_error = str(e)
            logging.error(f"CartesianXY3Z Safety: Unexpected error: {str(e)}")
            self.gcode.respond_info(f"Safety System Error: {str(e)}")
            return False

    cmd_CHECK_CARTESIANXY3Z_SAFETY_help = "Check if current position meets safety constraints"
    def cmd_CHECK_CARTESIANXY3Z_SAFETY(self, gcmd):
        """Manual safety check command"""
        pos = self.toolhead.get_position()
        gpos = self._gcode_pos(pos)
        is_safe, error = self.check_safety(gpos)
        if is_safe:
            self.is_safe = True
            self.last_error = ""
            gcmd.respond_info("Current position is SAFE")
        else:
            self.is_safe = False
            self.last_error = error
            gcmd.respond_info(f"Safety check FAILED: {error}")
        z_mz, z_ab, csx, i_ab, x_ab = self.calc_intermediate_values(gpos)
        points = self.calc_point_positions(gpos)
        x_now, y_now, z_now, z_norm = self.calc_vectors(points)
        gcmd.respond_info(
            f"Intermediate values:\n"
            f"  z_mz: {z_mz:.3f}, z_ab: {z_ab:.3f}\n"
            f"  csx: {csx:.3f}\n"
            f"  i_ab: {i_ab}, x_ab: {x_ab:.3f}\n\n"
            f"Point positions:\n"
            f"  P_Z: ({points['P_Z'][0]:.3f}, {points['P_Z'][1]:.3f}, {points['P_Z'][2]:.3f})\n"
            f"  P_A: ({points['P_A'][0]:.3f}, {points['P_A'][1]:.3f}, {points['P_A'][2]:.3f})\n"
            f"  P_B: ({points['P_B'][0]:.3f}, {points['P_B'][1]:.3f}, {points['P_B'][2]:.3f})\n"
            f"  P_E: ({points['P_E'][0]:.3f}, {points['P_E'][1]:.3f}, {points['P_E'][2]:.3f})\n\n"
            f"Vectors:\n"
            f"  x_now: ({x_now[0]:.3f}, {x_now[1]:.3f}, {x_now[2]:.3f})\n"
            f"  y_now: ({y_now[0]:.3f}, {y_now[1]:.3f}, {y_now[2]:.3f})\n"
            f"  z_now: ({z_now[0]:.3f}, {z_now[1]:.3f}, {z_now[2]:.3f})\n"
            f"  z_norm: ({z_norm[0]:.3f}, {z_norm[1]:.3f}, {z_norm[2]:.3f})"
        )

    cmd_PREVIEW_MOVE_help = "Preview a move without executing - check safety only"
    def cmd_PREVIEW_MOVE(self, gcmd):
        """Preview a move and check safety without sending to firmware"""
        try:
            # Get current position as starting point
            current_pos = list(self.toolhead.get_position())
            
            gcmd.respond_info(
                f"Current position: X={current_pos[0]:.2f} Y={current_pos[1]:.2f} "
                f"Z={current_pos[2]:.2f} A={current_pos[3]:.2f} B={current_pos[4]:.2f}"
            )
            
            # Parse parameters (PREVIEW_MOVE X=10 Y=20 Z=5 A=3 B=3)
            # Unspecified axes keep the current position value.
            target_pos = current_pos[:]
            if gcmd.get('X', None) is not None:
                target_pos[0] = gcmd.get_float('X')
            if gcmd.get('Y', None) is not None:
                target_pos[1] = gcmd.get_float('Y')
            if gcmd.get('Z', None) is not None:
                target_pos[2] = gcmd.get_float('Z')
            if gcmd.get('A', None) is not None:
                target_pos[3] = gcmd.get_float('A')
            if gcmd.get('B', None) is not None:
                target_pos[4] = gcmd.get_float('B')
            
            gcmd.respond_info(
                f"Target position:  X={target_pos[0]:.2f} Y={target_pos[1]:.2f} "
                f"Z={target_pos[2]:.2f} A={target_pos[3]:.2f} B={target_pos[4]:.2f}"
            )
            
            # Check safety at target position
            gpos = self._gcode_pos(target_pos)
            is_safe, error = self.check_safety(gpos)
            
            if is_safe:
                gcmd.respond_info("✓ Move is SAFE - would be allowed")
            else:
                gcmd.respond_info(f"✗ Move is BLOCKED: {error}")
            
            # Show detailed analysis
            z_mz, z_ab, csx, i_ab, x_ab = self.calc_intermediate_values(gpos)
            points = self.calc_point_positions(gpos)
            x_now, y_now, z_now, z_norm = self.calc_vectors(points)

            # Rail limit values (same logic as check_safety)
            max_xa = abs(z_mz * z_ab / csx - csx / 2 + self.d_x / 2) if csx > 0 else 0.0
            max_xb = abs(z_mz * z_ab / csx + csx / 2 - self.d_x / 2) if csx > 0 else 0.0
            inner_xa = (csx / 2 - self.d_x / 2 - z_mz * z_ab / csx) if csx > 0 else 0.0
            inner_xb = (csx / 2 - self.d_x / 2 + z_mz * z_ab / csx) if csx > 0 else 0.0
            denom_z = 1 - (z_ab**2) / (self.d_x**2)
            max_y = abs(math.sqrt(self.d_y**2 - z_mz**2 / denom_z) - self.d_y) \
                    if denom_z > 0 else float('inf')

            gcmd.respond_info(
                f"\nIntermediate values:\n"
                f"  z_mz: {z_mz:.3f}, z_ab: {z_ab:.3f}\n"
                f"  csx: {csx:.3f}\n"
                f"  i_ab: {i_ab}, x_ab: {x_ab:.3f}\n\n"
                f"Rail limits (x_out:{self.x_out:.1f}, x_in:{self.x_in:.1f}, y_out:{self.y_out:.1f}):\n"
                f"  max_xa (outer): {max_xa:.3f}\n"
                f"  max_xb (outer): {max_xb:.3f}\n"
                f"  inner_xa:       {inner_xa:.3f}\n"
                f"  inner_xb:       {inner_xb:.3f}\n"
                f"  max_y:          {max_y:.3f}\n\n"
                f"Point positions:\n"
                f"  P_Z: ({points['P_Z'][0]:.3f}, {points['P_Z'][1]:.3f}, {points['P_Z'][2]:.3f})\n"
                f"  P_A: ({points['P_A'][0]:.3f}, {points['P_A'][1]:.3f}, {points['P_A'][2]:.3f})\n"
                f"  P_B: ({points['P_B'][0]:.3f}, {points['P_B'][1]:.3f}, {points['P_B'][2]:.3f})\n"
                f"  P_E: ({points['P_E'][0]:.3f}, {points['P_E'][1]:.3f}, {points['P_E'][2]:.3f})\n\n"
                f"Bed orientation:\n"
                f"  z_norm: ({z_norm[0]:.3f}, {z_norm[1]:.3f}, {z_norm[2]:.3f})"
            )
            
        except Exception as e:
            gcmd.respond_info(f"Preview error: {str(e)}")

    def get_status(self, eventtime):
        """Return safety status for use in macros"""
        return {
            'is_safe': self.is_safe,
            'last_error': self.last_error
        }


def load_config(config):
    return CartesianXY3ZSafety(config)