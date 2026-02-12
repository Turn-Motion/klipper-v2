# Safety checks for cartesianxy3z kinematics with tilting bed
import logging
import math


class CartesianXY3ZSafety:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.is_safe = True
        self.last_error = ""
        self.d_x = config.getfloat('x_distance_between_joints_ab', 650.0)
        self.d_y = config.getfloat('y_distance_between_ab_center_and_z', 605.0)
        self.h_tot = config.getfloat('total_measured_height', 8.0)
        self.d_1r = config.getfloat('offset_measure_point_1_to_rotation_center', 4.0)
        self.d_2p = config.getfloat('offset_measure_point_2_to_plate_surface', 4.0)
        self.p_x = config.getfloat('printing_area_x_length', 600.0)
        self.p_y = config.getfloat('printing_area_y_length', 300.0)
        self.d_flex = config.getfloat('flex_plate_thickness', 6.0)
        self.x_out = config.getfloat('x_railspace_ab_outwards_neutral', 59.5)
        self.y_in = config.getfloat('y_railspace_z_inwards_neutral', 101.1)
        self.z_tot = self.h_tot - self.d_1r - self.d_2p + self.d_flex
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.gcode.register_command('CHECK_CARTESIANXY3Z_SAFETY',
            self.cmd_CHECK_CARTESIANXY3Z_SAFETY,
            desc=self.cmd_CHECK_CARTESIANXY3Z_SAFETY_help)
        self.gcode.register_command('PREVIEW_MOVE',
            self.cmd_PREVIEW_MOVE,
            desc=self.cmd_PREVIEW_MOVE_help)

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def calc_intermediate_values(self, pos):
        x, y = pos[0], pos[1]
        z, a, b = pos[2], pos[3], pos[4]
        z_mz = z - (a + b) / 2
        z_ab = b - a
        if abs(z_ab) >= self.d_x:
            csx = 0.0
        else:
            csx = math.sqrt(self.d_x**2 - z_ab**2)
        if abs(z_mz) >= self.d_y:
            csz = 0.0
        else:
            csz = math.sqrt(1 - (z_mz**2) / (self.d_y**2))
        i_ab = -1 if a > b else 1
        if csx == 0:
            x_ab = 0.0
        else:
            x_ab = (csx - self.d_x) / 2 - i_ab * z_mz * z_ab / csx
        return z_mz, z_ab, csx, csz, i_ab, x_ab

    def calc_point_positions(self, pos):
        x, y = pos[0], pos[1]
        z, a, b = pos[2], pos[3], pos[4]
        z_mz, z_ab, csx, csz, i_ab, x_ab = self.calc_intermediate_values(pos)
        P_Z_x = self.p_x / 2
        if abs(z_ab) >= self.d_x:
            P_Z_y = (self.p_y - self.d_y) / 2
        else:
            denom = 1 - (z_ab**2) / (self.d_x**2)
            if denom <= 0:
                P_Z_y = (self.p_y - self.d_y) / 2
            else:
                P_Z_y = math.sqrt(self.d_y**2 - z_mz**2 / denom) + (self.p_y - self.d_y) / 2
        P_Z_z = z
        if csx == 0:
            P_A_x = self.p_x / 2
        else:
            P_A_x = z_mz * z_ab / csx - csx / 2 + self.p_x / 2
        P_A_y = (self.p_y - self.d_y) / 2
        P_A_z = a
        if csx == 0:
            P_B_x = self.p_x / 2
        else:
            P_B_x = z_mz * z_ab / csx + csx / 2 + self.p_x / 2
        P_B_y = (self.p_y - self.d_y) / 2
        P_B_z = b
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
        x_now = (P_B[0] - P_A[0], P_B[1] - P_A[1], P_B[2] - P_A[2])
        mid_x = (P_A[0] + P_B[0]) / 2
        mid_y = (P_A[1] + P_B[1]) / 2
        mid_z = (P_A[2] + P_B[2]) / 2
        y_now = (P_Z[0] - mid_x, P_Z[1] - mid_y, P_Z[2] - mid_z)
        z_now = (
            x_now[1] * y_now[2] - x_now[2] * y_now[1],
            x_now[2] * y_now[0] - x_now[0] * y_now[2],
            x_now[0] * y_now[1] - x_now[1] * y_now[0]
        )
        z_norm_magnitude = math.sqrt(z_now[0]**2 + z_now[1]**2 + z_now[2]**2)
        if z_norm_magnitude > 0:
            z_norm = (z_now[0] / z_norm_magnitude, z_now[1] / z_norm_magnitude, z_now[2] / z_norm_magnitude)
        else:
            z_norm = (0, 0, 1)
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

            # ============================================================================
            # PRIMARY BOUNDARY CONDITIONS
            # ============================================================================
            # Level 0: X and Y position boundaries
            # The extruder tip must be within the printing area
            if x < 0 or x > self.p_x:
                return False, f"X position {x:.2f} outside printing area (0 to {self.p_x})"
            if y < 0 or y > self.p_y:
                return False, f"Y position {y:.2f} outside printing area (0 to {self.p_y})"

            # ============================================================================
            # SECONDARY PRELIMINARY CALCULATIONS
            # ============================================================================
            # Calculate intermediate values needed for remaining checks
            z_mz, z_ab, csx, csz, i_ab, x_ab = self.calc_intermediate_values(pos)
            max_z_distance = math.sqrt(self.d_y**2 - self.d_x**2 / 4)
   
            # ============================================================================
            # PRIMARY MECHANICAL CONSTRAINTS
            # ============================================================================
            # Level 1: Primary boundary conditions - mechanical joint distances
            if abs(z - a) > max_z_distance:
                return False, "Z-A distance exceeded"
            if abs(z - b) > max_z_distance:
                return False, "Z-B distance exceeded"
            if abs(a - b) > self.d_x:
                return False, "A-B distance exceeded"
   
            # ============================================================================
            # SECONDARY BOUNDARY CONDITIONS � GOING OFF RAILS
            # ============================================================================
            # Level 2: Wagon rail limit checks
            # Check A/B wagon X-position limits
            if abs(z_ab) > 0:
                max_z_mz = min(
                    self.x_out + (self.d_x - csx) / 2,
                    self.d_x - (self.d_x + csx) / 2
                ) * csx / abs(z_ab)
                if abs(z_mz) > max_z_mz:
                    return False, "A/B wagon approaching rail limit"
            
            # Check Z wagon Y-position limits
            if abs(z_mz) > math.sqrt(self.d_y**2 - (self.d_y - self.y_in)**2):
                return False, "Z wagon approaching rail limit"
  
            # ============================================================================
            # TERTIARY CALCULATIONS - COLLISION DETECTION SETUP
            # ============================================================================
            # Calculate point positions (P_Z, P_A, P_B, P_E)
            points = self.calc_point_positions(pos)
            
            # Calculate orientation vectors of the tilted bed plane
            x_now, y_now, z_now, z_norm = self.calc_vectors(points)
    
            # ============================================================================
            # TERTIARY BOUNDARY CONDITION: COLLISION OF EXTRUDER TIP WITH BED
            # ============================================================================
            # Level 3: Extruder tip collision detection with tilting print bed
  
            # Extract key point positions
            P_E = points['P_E']      # Extruder tip position (x, y, z_tot)
            P_A = points['P_A']      # A-axis joint position
            
            # Calculate the reference point on the bed surface
            # P_A_offset = P_A + z_tot * z_norm
            # This represents a point on the bed surface directly above P_A
            P_A_offset = (
                P_A[0] + self.z_tot * z_norm[0],
                P_A[1] + self.z_tot * z_norm[1],
                P_A[2] + self.z_tot * z_norm[2]
            )
    
            # Calculate vector from reference point to extruder tip
            vec_P_E_to_offset = (
                P_E[0] - P_A_offset[0],
                P_E[1] - P_A_offset[1],
                P_E[2] - P_A_offset[2]
            )
            
            # Project extruder position onto bed normal vector using dot product
            # This gives the signed distance from the bed plane to the extruder tip
            # If dist < 0: extruder is below bed plane = COLLISION
            # If dist > 0: extruder is above bed plane = SAFE
            dist = (vec_P_E_to_offset[0] * z_now[0] + 
                    vec_P_E_to_offset[1] * z_now[1] + 
                    vec_P_E_to_offset[2] * z_now[2])

            if dist < 0:
                return False, "Extruder collision with print bed detected"
    
            # ============================================================================
            # ALL SAFETY CHECKS PASSED
            # ============================================================================
            return True, ""
        except Exception as e:
            return False, f"Safety check error: {str(e)}"

    def check_move(self, move):
        """Check if a move is safe."""
        try:
            if not any(move.axes_d[i] for i in [2, 3, 4]):
                return True
            is_safe, error = self.check_safety(move.start_pos)
            if not is_safe:
                self.is_safe = False
                self.last_error = error
                logging.warning(f"CartesianXY3Z Safety: {error} (start position)")
                self.gcode.respond_info(f"Safety Error: {error}")
                return False
            is_safe, error = self.check_safety(move.end_pos)
            if not is_safe:
                self.is_safe = False
                self.last_error = error
                logging.warning(f"CartesianXY3Z Safety: {error} (end position)")
                self.gcode.respond_info(f"Safety Error: {error}")
                return False
            if move.move_d > 1.0:
                midpos = [(move.start_pos[i] + move.end_pos[i]) / 2 for i in range(len(move.start_pos))]
                is_safe, error = self.check_safety(midpos)
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
        is_safe, error = self.check_safety(pos)
        if is_safe:
            self.is_safe = True
            self.last_error = ""
            gcmd.respond_info("Current position is SAFE")
        else:
            self.is_safe = False
            self.last_error = error
            gcmd.respond_info(f"Safety check FAILED: {error}")
        z_mz, z_ab, csx, csz, i_ab, x_ab = self.calc_intermediate_values(pos)
        points = self.calc_point_positions(pos)
        x_now, y_now, z_now, z_norm = self.calc_vectors(points)
        gcmd.respond_info(
            f"Intermediate values:\n"
            f"  z_mz: {z_mz:.3f}, z_ab: {z_ab:.3f}\n"
            f"  csx: {csx:.3f}, csz: {csz:.3f}\n"
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
            
            # Parse G-code parameters (X, Y, Z, A, B)
            target_pos = current_pos[:]
            
            if gcmd.get('X') is not None:
                target_pos[0] = gcmd.get_float('X')
            if gcmd.get('Y') is not None:
                target_pos[1] = gcmd.get_float('Y')
            if gcmd.get('Z') is not None:
                target_pos[2] = gcmd.get_float('Z')
            if gcmd.get('A') is not None:
                target_pos[3] = gcmd.get_float('A')
            if gcmd.get('B') is not None:
                target_pos[4] = gcmd.get_float('B')
            
            gcmd.respond_info(
                f"Target position:  X={target_pos[0]:.2f} Y={target_pos[1]:.2f} "
                f"Z={target_pos[2]:.2f} A={target_pos[3]:.2f} B={target_pos[4]:.2f}"
            )
            
            # Check safety at target position
            is_safe, error = self.check_safety(target_pos)
            
            if is_safe:
                gcmd.respond_info("✓ Move is SAFE - would be allowed")
            else:
                gcmd.respond_info(f"✗ Move is BLOCKED: {error}")
            
            # Show detailed analysis
            z_mz, z_ab, csx, csz, i_ab, x_ab = self.calc_intermediate_values(target_pos)
            points = self.calc_point_positions(target_pos)
            x_now, y_now, z_now, z_norm = self.calc_vectors(points)
            
            gcmd.respond_info(
                f"\nIntermediate values:\n"
                f"  z_mz: {z_mz:.3f}, z_ab: {z_ab:.3f}\n"
                f"  csx: {csx:.3f}, csz: {csz:.3f}\n"
                f"  i_ab: {i_ab}, x_ab: {x_ab:.3f}\n\n"
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