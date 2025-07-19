# Safety checks for cartesianxy3z kinematics with tilting bed
#
# Copyright (C) 2024 
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

class CartesianXY3ZSafety:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Safety status tracking
        self.is_safe = True
        self.last_error = ""
        
        # Register configuration parameters
        # Joint ball parameters
        self.s = config.getfloat('joint_ball_diameter', 12.0)
        self.w = config.getfloat('wagon_length', 45.0)
        self.z_w = config.getfloat('wagon_rail_z_offset', 5.0)
        self.d_x = config.getfloat('x_distance_between_joints_ab', 516.0)
        self.L_y = config.getfloat('y_distance_plate_to_joint_z', 521.0)
        self.L_off = config.getfloat('y_overhang_plate_to_joints_ab', 5.0)
        self.h_plate = config.getfloat('plate_thickness', 9.9)
        
        # Printer.cfg parameters
        self.s_in = config.getfloat('joint_ball_inset', 4.0)
        self.z_alu = config.getfloat('alu_mount_plate_thickness', 6.0)
        self.z_mount = config.getfloat('joint_mount_thickness', 5.0)
        self.y_off = config.getfloat('y_distance_joints_ab_to_rail', 14.0)
        self.x_in = config.getfloat('x_railspace_ab_inwards', 117.5)
        self.x_out = config.getfloat('x_railspace_ab_outwards', 59.5)
        self.y_in = config.getfloat('y_railspace_z_inwards', 101.1)
        self.h_tot = config.getfloat('total_plate_height', 44.15)
        
        # Pre-calculate constant values
        self.d_y = self.L_y - self.L_off
        self.z_out = self.h_tot - self.s/2
        self.z_rail = self.z_w + self.z_alu + self.z_mount - self.s_in + self.s/2
        
        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        
        # Register commands
        self.gcode.register_command('CHECK_CARTESIANXY3Z_SAFETY',
                                    self.cmd_CHECK_CARTESIANXY3Z_SAFETY,
                                    desc=self.cmd_CHECK_CARTESIANXY3Z_SAFETY_help)
    
    def handle_connect(self):
        # Get toolhead object
        self.toolhead = self.printer.lookup_object('toolhead')
    
    def calc_intermediate_values(self, pos):
        # Extract current positions
        x, y = pos[0], pos[1]
        z, a, b = pos[2], pos[3], pos[4]
        
        # Calculate intermediate values
        z_mz = z - (a + b)/2
        z_ab = b - a
        
        # Avoid division by zero
        if abs(z_mz) >= self.d_y:
            csz = 0
        else:
            csz = math.sqrt(1 - (z_mz**2)/(self.d_y**2))
        
        if abs(z_ab) >= self.d_x:
            csx = 0
        else:
            csx = math.sqrt(self.d_x**2 - z_ab**2)
        
        i_ab = -1 if a > b else 1
        
        # Avoid division by zero
        if csx == 0:
            x_ab = 0
        else:
            x_ab = (csx - self.d_x)/2 - i_ab * z_mz * z_ab/csx
        
        return z_mz, z_ab, csz, csx, i_ab, x_ab
    
    def check_safety(self, pos, move):
        """Check all safety conditions for the given position"""
        x, y = pos[0], pos[1]
        z, a, b = pos[2], pos[3], pos[4]
        
        # Calculate intermediate values
        z_mz, z_ab, csz, csx, i_ab, x_ab = self.calc_intermediate_values(pos)
        
        # Primary Boundary Conditions - Out Of Bounds
        # Check if Z is too far from A or B
        max_z_distance = math.sqrt(self.d_y**2 - self.d_x**2/4)
        if abs(z - a) > max_z_distance:
            raise move.move_error("Z-A distance exceeded")
        if abs(z - b) > max_z_distance:
            raise move.move_error("Z-B distance exceeded")
        
        # Check if A and B are too far apart
        if abs(a - b) > self.d_x:
            raise move.move_error("A-B distance exceeded")
        
        # Secondary Boundary Conditions - Going Off Rails
        # Check Z-axis wagon rail limits
        if abs(z_ab) > 0:  # Avoid division by zero
            max_z_mz = min(
                self.x_out + (self.d_x - csx)/2,
                self.x_in - (self.d_x + csx)/2
            ) * csx / abs(z_ab)
            
            if z_mz > max_z_mz:
                raise move.move_error("Z wagon approaching rail limit")
        
        # Check Y-axis rail limits
        if abs(z_mz) > math.sqrt(self.d_y**2 - (self.d_y - self.y_in)**2):
            raise move.move_error("Y rail limit exceeded")
        
        # Tertiary Boundary Conditions - Collisions
        # Check for collision between printing plate and x-rail of A/B joints
        condition1 = abs(z_mz) >= self.d_y/self.z_out * (self.y_off - self.L_off * csz)
        
        # Avoid division by zero
        if (self.x_in + self.w/2 + x_ab) == 0:
            condition2 = False
        else:
            condition2 = abs(z_ab) >= self.d_x/(self.x_in + self.w/2 + x_ab) * (
                (self.z_out - self.h_plate) * csz - 
                abs(z_mz) * self.L_off/self.d_y + 
                self.z_rail * csx/self.d_x
            )
        
        if condition1 and condition2:
            raise move.move_error("Potential plate collision with x-rail")
    
    def check_move(self, move):
        """Check if a move is safe - called directly from kinematics"""
        try:
            # Only check if any of Z, A, or B axes are moving
            if any(move.axes_d[i] for i in [2, 3, 4]):  # z, a, b axes
                # Check both start and end positions for safety
                self.check_safety(move.start_pos, move)
                self.check_safety(move.end_pos, move)
                
                # For longer moves, check an intermediate point too
                if move.move_d > 1.0:  # For moves longer than 1mm
                    # Check a midpoint
                    midpos = [(move.start_pos[i] + move.end_pos[i])/2 for i in range(len(move.start_pos))]
                    self.check_safety(midpos, move)
                
                # If we get here, the move is safe
                self.is_safe = True
                self.last_error = ""
            
            # Return True to indicate the move is safe
            return True
        except Exception as e:
            # Update safety status
            self.is_safe = False
            self.last_error = str(e)
            # Re-raise the exception to stop the move
            raise
    
    cmd_CHECK_CARTESIANXY3Z_SAFETY_help = "Check if current position meets safety constraints"
    def cmd_CHECK_CARTESIANXY3Z_SAFETY(self, gcmd):
        # Get current position
        pos = self.toolhead.get_position()
        
        # Create a dummy move for error reporting
        class DummyMove:
            def move_error(self, msg):
                return msg
        
        dummy_move = DummyMove()
        
        # Check safety at current position
        try:
            self.check_safety(pos, dummy_move)
            self.is_safe = True
            self.last_error = ""
            message = "Current position is safe"
        except Exception as e:
            self.is_safe = False
            self.last_error = str(e)
            message = str(e)
        
        # Report results
        if self.is_safe:
            gcmd.respond_info(message)
        else:
            gcmd.respond_info("Safety check failed: " + message)
        
        # Calculate and report intermediate values for debugging
        z_mz, z_ab, csz, csx, i_ab, x_ab = self.calc_intermediate_values(pos)
        gcmd.respond_info("Intermediate values:\n"
                          f"z_mz: {z_mz:.3f}, z_ab: {z_ab:.3f}\n"
                          f"csz: {csz:.3f}, csx: {csx:.3f}\n"
                          f"i_ab: {i_ab}, x_ab: {x_ab:.3f}")
    
    def get_status(self, eventtime):
        """Return safety status for use in macros"""
        return {
            'is_safe': self.is_safe,
            'last_error': self.last_error
        }

def load_config(config):
    return CartesianXY3ZSafety(config)
