#!/usr/bin/env python3
import math
import threading
import random
import time
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Int32
from tf_transformations import euler_from_quaternion

# Configuration
INITIAL_TURN_ANGLE = 45
INITIAL_DRIVE_DISTANCE = 5.0
BOUNDARY_BUFFER = 1.0  # Buffer for hard boundary detection
MAP_SIZE = 15.0
MAP_CENTER = (MAP_SIZE/2, MAP_SIZE/2)

# New: Lidar Obstacle Avoidance Parameters
OBSTACLE_BUFFER = 0.6  # Increased slightly for more reaction time
EMERGENCY_STOP_DISTANCE = 0.3  # Full stop if lidar obstacle is closer than this
# FOV for the front sector (e.g., 30 deg to left and 30 deg to right of front)
LIDAR_FRONT_ANGLE_FOV = math.radians(60)
# FOV for side sectors (e.g., 30 deg segments)
LIDAR_SIDE_ANGLE_FOV = math.radians(30)
OBSTACLE_REVERSE_DURATION = 0.7
OBSTACLE_TURN_DURATION = 1.5

# Predictive avoidance parameters
PREDICTIVE_BUFFER = 4.0  # Distance to trigger predictive avoidance

# Speeds
TURN_SPEED_DEG = 70  # Increased for faster turns
FORWARD_SPEED = 0.8   # Reduced for better control during wandering
REVERSE_SPEED = -0.7
ANGLE_TOLERANCE = 7.0  # Increased for faster turn completion

# Transition delay
TRANSITION_STOP_DURATION = 0.3
REVERSE_DURATION = 2.0

# Turning parameters for smoother control
MIN_TURN_SPEED_FACTOR = 0.2
MAX_TURN_ANGLE_FOR_FULL_SPEED = 45.0

# Wandering behavior tuning
MIN_DRIVE_DISTANCE = 2.0
MAX_DRIVE_DISTANCE = 5.0
MAX_WANDERING_TURN_ANGLE = 120.0

# Boundary Avoidance Specifics (for hard collision reaction)
BOUNDARY_ESCAPE_DRIVE_DISTANCE = 3.0
CENTER_BIAS_STRENGTH = 0.7
MIN_CENTER_ANGLE = -45
MAX_CENTER_ANGLE = 45
MIN_HARD_ESCAPE_ANGLE = 90  # Minimum random turn angle for hard escape
MAX_HARD_ESCAPE_ANGLE = 270  # Maximum random turn angle for hard escape

# New: Return to Center specific parameters
RETURN_TO_CENTER_SPEED = 0.5  # Slower speed when returning to center
RETURN_TO_CENTER_TURN_SPEED_DEG = 45  # Moderate turn speed
# Minimum distance from center to consider "returned"
RETURN_TO_CENTER_MIN_DISTANCE = 0.5


class DRIVE_MODE(Enum):
    MANUAL = 1
    AUTO = 2


class AUTO_STATE(Enum):
    INITIAL_TURN = 0
    INITIAL_DRIVE = 1
    WANDERING_TURN = 2
    WANDERING_DRIVE = 3
    BOUNDARY_REVERSE = 4
    BOUNDARY_ESCAPE_TURN = 5
    BOUNDARY_ESCAPE_DRIVE = 6
    OBSTACLE_REVERSE = 7  # New state for lidar obstacle avoidance
    OBSTACLE_TURN = 8  # New state for lidar obstacle avoidance turn
    RETURN_TO_CENTER = 9  # New state to drive back to center


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.mutex = threading.Lock()

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_team10', 10)
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)

        self.drive_mode = DRIVE_MODE.AUTO
        self.auto_state = AUTO_STATE.INITIAL_TURN
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_yaw = None
        self.start_position = (0.0, 0.0)
        self.drive_distance = 0.0
        self.wandering_target_yaw = None

        self.turn_start_time = None
        self.reverse_start_time = 0.0
        self.obstacle_maneuver_start_time = 0.0

        self.transition_stop_end_time = 0.0

        self.last_linear = 0.0
        self.last_angular = 0.0
        self.trigger = False  # For deadman switch

        self.lidar_data = None
        self.front_min_distance = float('inf')
        self.left_min_distance = float('inf')
        self.right_min_distance = float('inf')

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Starting exploration routine")

    def joy_cb(self, msg: Joy):
        with self.mutex:
            # --- Mode switching ---
            if msg.buttons[0]:  # A button (Xbox) / Cross (PS4)
                if self.drive_mode != DRIVE_MODE.AUTO:
                    self.drive_mode = DRIVE_MODE.AUTO
                    self.auto_state = AUTO_STATE.INITIAL_TURN
                    self.target_yaw = None
                    self.get_logger().info("Auto mode activated. Restarting initial turn.")

            elif msg.buttons[1]:  # B button (Xbox) / Circle (PS4)
                if self.drive_mode != DRIVE_MODE.MANUAL:
                    self.drive_mode = DRIVE_MODE.MANUAL
                    self.publish_twist(0.0, 0.0)
                    self.get_logger().info("Manual mode activated")

            # --- Deadman (R2/L2) ---
            # R2 trigger (fully pressed ~ -1.0), adjust based on your joystick's axis range
            deadman_pressed = msg.axes[5] < - \
                0.9 if len(msg.axes) > 5 else False

            if not deadman_pressed and self.drive_mode == DRIVE_MODE.MANUAL:
                if self.trigger:  # If it was previously pressed and now released
                    self.publish_twist(0.0, 0.0)
                    self.get_logger().warn("Deadman released! Stopping.")
                self.trigger = False
                return  # Do not process other manual inputs if deadman is not pressed

            if self.drive_mode == DRIVE_MODE.MANUAL:
                self.trigger = True  # Deadman is pressed

                lin_input = msg.axes[1]  # Left stick vertical
                # Right stick horizontal (common for Xbox/PS4)
                ang_input = msg.axes[3]

                # Cubic scaling for finer control
                linear = -(lin_input ** 3) * 0.8
                angular = -(ang_input ** 3) * 1.5

                # Ramping (optional, for smoother manual control)
                if abs(linear - self.last_linear) > 0.1:
                    linear = self.last_linear + \
                        math.copysign(0.1, linear - self.last_linear)
                if abs(angular - self.last_angular) > 0.15:
                    angular = self.last_angular + \
                        math.copysign(0.15, angular - self.last_angular)

                self.last_linear = linear
                self.last_angular = angular

                # Assuming positive joystick input means reverse for linear. Adjust if needed.
                self.publish_twist(-linear, -angular)

    def lidar_cb(self, msg: LaserScan):
        with self.mutex:
            self.lidar_data = msg
            if not self.lidar_data or not self.lidar_data.ranges:
                self.front_min_distance = float('inf')
                self.left_min_distance = float('inf')
                self.right_min_distance = float('inf')
                return

            ranges = self.lidar_data.ranges
            angle_min = self.lidar_data.angle_min
            angle_increment = self.lidar_data.angle_increment
            num_readings = len(ranges)

            # --- Front Sector ---
            front_start_angle = -LIDAR_FRONT_ANGLE_FOV / 2
            front_end_angle = LIDAR_FRONT_ANGLE_FOV / 2

            center_idx = num_readings // 2
            half_front_fov_idx = int(
                (LIDAR_FRONT_ANGLE_FOV / 2) / angle_increment)

            front_indices = []
            for i in range(center_idx - half_front_fov_idx, center_idx + half_front_fov_idx + 1):
                front_indices.append(i % num_readings)

            front_scan_values = [
                ranges[idx] for idx in front_indices
                if not math.isinf(ranges[idx]) and ranges[idx] > 0.01
            ]
            self.front_min_distance = min(
                front_scan_values) if front_scan_values else float('inf')

            # --- Left Sector ---
            left_start_angle = LIDAR_FRONT_ANGLE_FOV / 2
            left_end_angle = left_start_angle + LIDAR_SIDE_ANGLE_FOV

            left_start_idx = int(
                (left_start_angle - angle_min) / angle_increment)
            left_end_idx = int((left_end_angle - angle_min) / angle_increment)

            left_indices = []
            for i in range(left_start_idx, left_end_idx + 1):
                left_indices.append(i % num_readings)

            left_scan_values = [
                ranges[idx] for idx in left_indices
                if not math.isinf(ranges[idx]) and ranges[idx] > 0.01
            ]
            self.left_min_distance = min(
                left_scan_values) if left_scan_values else float('inf')

            # --- Right Sector ---
            right_end_angle = -LIDAR_FRONT_ANGLE_FOV / 2
            right_start_angle = right_end_angle - LIDAR_SIDE_ANGLE_FOV

            right_start_idx = int(
                (right_start_angle - angle_min) / angle_increment)
            right_end_idx = int(
                (right_end_angle - angle_min) / angle_increment)

            right_indices = []
            for i in range(right_start_idx, right_end_idx + 1):
                right_indices.append(i % num_readings)

            right_scan_values = [
                ranges[idx] for idx in right_indices
                if not math.isinf(ranges[idx]) and ranges[idx] > 0.01
            ]
            self.right_min_distance = min(
                right_scan_values) if right_scan_values else float('inf')

    def odom_cb(self, msg):
        with self.mutex:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y

            orientation = msg.pose.pose.orientation
            _, _, yaw_rad = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            self.current_yaw = (math.degrees(yaw_rad) + 360) % 360

    def control_loop(self):
        with self.mutex:
            # If manual mode, do nothing further in auto control_loop
            if self.drive_mode != DRIVE_MODE.AUTO:
                return

            # Check for emergency stop due to close obstacle first (highest priority)
            if self.front_min_distance < EMERGENCY_STOP_DISTANCE:
                self.get_logger().warn(
                    f"EMERGENCY STOP! Obstacle at {self.front_min_distance:.2f}m.")
                self.publish_twist(0.0, 0.0)
                if self.auto_state not in [AUTO_STATE.OBSTACLE_REVERSE, AUTO_STATE.OBSTACLE_TURN,
                                           AUTO_STATE.BOUNDARY_REVERSE, AUTO_STATE.BOUNDARY_ESCAPE_TURN, AUTO_STATE.BOUNDARY_ESCAPE_DRIVE,
                                           AUTO_STATE.RETURN_TO_CENTER]:  # Exclude RETURN_TO_CENTER from resetting to OBSTACLE_REVERSE immediately, as it's a high-level state
                    self.auto_state = AUTO_STATE.OBSTACLE_REVERSE
                    self.obstacle_maneuver_start_time = time.time()
                return  # Stop all other operations, just brake

            # Handle obstacle avoidance states (these states are for active maneuvering)
            if self.auto_state == AUTO_STATE.OBSTACLE_REVERSE:
                self.handle_obstacle_reverse()
                return
            elif self.auto_state == AUTO_STATE.OBSTACLE_TURN:
                self.handle_obstacle_turn()
                return

            # If still within transition stop duration, publish stop and return
            if time.time() < self.transition_stop_end_time:
                self.publish_twist(0.0, 0.0)
                return

            # --- New: Check if robot is outside the main map boundaries ---
            # IMPORTANT: Changed this to use the MAP_SIZE directly for boundary check,
            # as current_x and current_y are relative to the map origin (0,0).
            # This ensures we enter RETURN_TO_CENTER when *any* coordinate is outside [0, MAP_SIZE]
            if not (0 <= self.current_x <= MAP_SIZE and 0 <= self.current_y <= MAP_SIZE) and \
               self.auto_state != AUTO_STATE.RETURN_TO_CENTER:
                self.get_logger().warn(
                    f"Robot outside map bounds at ({self.current_x:.2f}, {self.current_y:.2f}). Returning to center.")
                self.auto_state = AUTO_STATE.RETURN_TO_CENTER
                self.target_yaw = None  # Reset target yaw for new state
                self.publish_twist(0.0, 0.0)
                self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
                return

            # Main Auto Mode Logic (order of precedence matters)
            # 1. Hard Boundary Collision Detection (highest priority after emergency stop/obstacle maneuver, and return to center)
            if self.auto_state not in [AUTO_STATE.INITIAL_TURN, AUTO_STATE.INITIAL_DRIVE, AUTO_STATE.RETURN_TO_CENTER] and \
               self.near_boundary(BOUNDARY_BUFFER) and \
               self.auto_state not in [AUTO_STATE.BOUNDARY_REVERSE,
                                       AUTO_STATE.BOUNDARY_ESCAPE_TURN,
                                       AUTO_STATE.BOUNDARY_ESCAPE_DRIVE]:
                self.get_logger().warn(
                    f"HARD BOUNDARY hit at ({self.current_x:.2f}, {self.current_y:.2f}) with yaw {self.current_yaw:.1f}°. Initiating boundary REVERSE.")
                self.auto_state = AUTO_STATE.BOUNDARY_REVERSE
                self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
                self.reverse_start_time = time.time()
                return

            # 2. Predictive Obstacle Avoidance (LIDAR based)
            # Trigger avoidance if something is within OBSTACLE_BUFFER in front AND not already handling boundary or returning to center
            if self.front_min_distance < OBSTACLE_BUFFER and \
               self.auto_state not in [AUTO_STATE.BOUNDARY_REVERSE, AUTO_STATE.BOUNDARY_ESCAPE_TURN, AUTO_STATE.BOUNDARY_ESCAPE_DRIVE, AUTO_STATE.RETURN_TO_CENTER]:
                self.get_logger().info(
                    f"Obstacle detected in front (LIDAR) at {self.front_min_distance:.2f}m. Initiating avoidance.")
                self.auto_state = AUTO_STATE.OBSTACLE_REVERSE
                self.obstacle_maneuver_start_time = time.time()
                # Immediately stop before reversing
                self.publish_twist(0.0, 0.0)
                self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION  # Brief stop
                return

            # State handling for normal auto navigation
            if self.auto_state == AUTO_STATE.INITIAL_TURN:
                self.handle_initial_turn()
            elif self.auto_state == AUTO_STATE.INITIAL_DRIVE:
                self.handle_initial_drive()
            elif self.auto_state == AUTO_STATE.WANDERING_TURN:
                self.handle_wandering_turn()
            elif self.auto_state == AUTO_STATE.WANDERING_DRIVE:
                self.handle_wandering_drive()
            elif self.auto_state == AUTO_STATE.BOUNDARY_REVERSE:
                self.handle_boundary_reverse()
            elif self.auto_state == AUTO_STATE.BOUNDARY_ESCAPE_TURN:
                self.handle_boundary_escape_turn()
            elif self.auto_state == AUTO_STATE.BOUNDARY_ESCAPE_DRIVE:
                self.handle_boundary_escape_drive()
            elif self.auto_state == AUTO_STATE.RETURN_TO_CENTER:
                self.handle_return_to_center()

    def handle_obstacle_reverse(self):
        if time.time() - self.obstacle_maneuver_start_time < OBSTACLE_REVERSE_DURATION:
            self.publish_twist(REVERSE_SPEED, 0.0)
            self.get_logger().debug(
                f"Obstacle Reverse. Remaining: {OBSTACLE_REVERSE_DURATION - (time.time() - self.obstacle_maneuver_start_time):.2f}s")
        else:
            self.publish_twist(0.0, 0.0)
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.auto_state = AUTO_STATE.OBSTACLE_TURN
            self.obstacle_maneuver_start_time = time.time()

            if self.left_min_distance > self.right_min_distance:
                turn_angle = 90
            else:
                turn_angle = -90
            self.target_yaw = (self.current_yaw + turn_angle) % 360
            self.get_logger().info(
                f"Finished obstacle reverse. Initiating obstacle turn to {self.target_yaw:.1f}° (Left:{self.left_min_distance:.2f}, Right:{self.right_min_distance:.2f}).")

    def handle_obstacle_turn(self):
        angle_diff = (self.target_yaw - self.current_yaw + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360

        abs_angle_diff = abs(angle_diff)

        if abs_angle_diff < ANGLE_TOLERANCE or (time.time() - self.obstacle_maneuver_start_time > OBSTACLE_TURN_DURATION):
            self.publish_twist(0.0, 0.0)
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.auto_state = AUTO_STATE.WANDERING_DRIVE
            self.target_yaw = None
            self.get_logger().info("Finished obstacle turn. Resuming wandering drive.")
        else:
            angular_speed = math.copysign(
                math.radians(TURN_SPEED_DEG), angle_diff)
            self.publish_twist(0.0, angular_speed)
            self.get_logger().debug(
                f"Obstacle Turn. Current Yaw: {self.current_yaw:.1f}°, Target Yaw: {self.target_yaw:.1f}°, Diff: {angle_diff:.1f}°")

    def handle_initial_turn(self):
        if self.target_yaw is None:
            self.target_yaw = (self.current_yaw + INITIAL_TURN_ANGLE) % 360
            self.get_logger().info(
                f"Initial turn to {self.target_yaw:.1f}° from {self.current_yaw:.1f}°")
            self.publish_twist(0.0, 0.0)
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            return
        self.execute_smooth_turn(AUTO_STATE.INITIAL_DRIVE, FORWARD_SPEED)

    def handle_initial_drive(self):
        distance = math.hypot(self.current_x - self.start_position[0],
                              self.current_y - self.start_position[1])

        if distance >= INITIAL_DRIVE_DISTANCE:
            self.auto_state = AUTO_STATE.WANDERING_TURN
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.target_yaw = None
            self.get_logger().info("Initial drive complete. Starting wandering turn.")
        else:
            self.publish_twist(FORWARD_SPEED, 0.0)

    def handle_wandering_turn(self):
        if self.target_yaw is None:
            if self.wandering_target_yaw is not None:
                self.target_yaw = self.wandering_target_yaw
                self.wandering_target_yaw = None
                self.get_logger().info(
                    f"Wandering turn (predictive) to {self.target_yaw:.1f}° from {self.current_yaw:.1f}°.")
            else:
                random_angle = random.uniform(-MAX_WANDERING_TURN_ANGLE,
                                              MAX_WANDERING_TURN_ANGLE)
                self.target_yaw = (self.current_yaw + random_angle) % 360
                self.get_logger().info(
                    f"Wandering turn (random) to {self.target_yaw:.1f}° from {self.current_yaw:.1f}°.")

            self.drive_distance = random.uniform(
                MIN_DRIVE_DISTANCE, MAX_DRIVE_DISTANCE)
            self.publish_twist(0.0, 0.0)
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            return

        self.execute_smooth_turn(AUTO_STATE.WANDERING_DRIVE, FORWARD_SPEED)

    def handle_wandering_drive(self):
        # Predictive check during wandering drive
        if self.predictive_boundary_check():
            self.get_logger().info("Predictive avoidance: Adjusting course")
            avoidance_angle = self._calculate_predictive_avoidance_yaw()

            self.wandering_target_yaw = avoidance_angle

            self.auto_state = AUTO_STATE.WANDERING_TURN
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.publish_twist(0.0, 0.0)
            return

        distance = math.hypot(self.current_x - self.start_position[0],
                              self.current_y - self.start_position[1])

        if distance >= self.drive_distance:
            self.auto_state = AUTO_STATE.WANDERING_TURN
            self.target_yaw = None
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.get_logger().info("Wandering drive distance met. Starting next wandering turn.")
        else:
            self.publish_twist(FORWARD_SPEED, 0.0)

    def handle_boundary_reverse(self):
        if time.time() - self.reverse_start_time < REVERSE_DURATION:
            self.publish_twist(REVERSE_SPEED, 0.0)
            self.get_logger().info(
                f"Reversing from boundary. Remaining: {REVERSE_DURATION - (time.time() - self.reverse_start_time):.2f}s")
        else:
            self.publish_twist(0.0, 0.0)
            self.auto_state = AUTO_STATE.BOUNDARY_ESCAPE_TURN
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.target_yaw = None
            self.get_logger().info("Reverse complete. Initiating boundary escape turn.")

    def execute_aggressive_turn(self, next_state):
        """More aggressive turning with full speed for boundary or obstacle escapes"""
        angle_diff = (self.target_yaw - self.current_yaw + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360

        abs_angle_diff = abs(angle_diff)

        if abs_angle_diff < ANGLE_TOLERANCE:
            self.publish_twist(0.0, 0.0)
            if self.auto_state != AUTO_STATE.OBSTACLE_TURN:
                self.auto_state = next_state
                self.start_position = (self.current_x, self.current_y)
                self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
                self.target_yaw = None
                self.get_logger().info(
                    f"Aggressive turn complete. Transitioning to {next_state.name}.")
            return

        angular_speed = math.copysign(math.radians(TURN_SPEED_DEG), angle_diff)
        self.publish_twist(0.0, angular_speed)

    def handle_boundary_escape_turn(self):
        if self.target_yaw is None:
            boundaries = {
                "east": MAP_SIZE - self.current_x,
                "west": self.current_x,
                "north": MAP_SIZE - self.current_y,
                "south": self.current_y
            }
            closest_boundary = min(boundaries, key=boundaries.get)

            if closest_boundary == "east":
                base_angle = 180
            elif closest_boundary == "west":
                base_angle = 0
            elif closest_boundary == "north":
                base_angle = 270
            else:  # south
                base_angle = 90

            center_yaw = self._calculate_general_inward_direction()
            angle_offset = random.uniform(MIN_CENTER_ANGLE, MAX_CENTER_ANGLE)
            self.target_yaw = (base_angle * (1 - CENTER_BIAS_STRENGTH) +
                               center_yaw * CENTER_BIAS_STRENGTH + angle_offset) % 360

            self.get_logger().info(
                f"Boundary escape: Facing {closest_boundary} boundary, turning to {self.target_yaw:.1f}° (Center bias: {center_yaw:.1f}°)")
            self.publish_twist(0.0, 0.0)
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            return

        self.execute_aggressive_turn(AUTO_STATE.BOUNDARY_ESCAPE_DRIVE)

    def handle_boundary_escape_drive(self):
        distance = math.hypot(self.current_x - self.start_position[0],
                              self.current_y - self.start_position[1])

        if distance >= BOUNDARY_ESCAPE_DRIVE_DISTANCE:
            self.auto_state = AUTO_STATE.WANDERING_TURN
            self.target_yaw = None
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.get_logger().info(
                "Boundary escape drive complete. Now returning to wandering.")
        else:
            self.publish_twist(FORWARD_SPEED, 0.0)

    def handle_return_to_center(self):
        distance_to_center = math.hypot(
            self.current_x - MAP_CENTER[0], self.current_y - MAP_CENTER[1])

        if distance_to_center < RETURN_TO_CENTER_MIN_DISTANCE:
            self.get_logger().info("Successfully returned to map center. Resuming wandering.")
            self.auto_state = AUTO_STATE.WANDERING_TURN
            self.target_yaw = None
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.publish_twist(0.0, 0.0)
            return

        # Calculate vector from current position to center
        vec_x = MAP_CENTER[0] - self.current_x
        vec_y = MAP_CENTER[1] - self.current_y

        # Calculate angle to center using atan2(y, x) for correct quadrant.
        # This angle is relative to the positive X-axis (standard mathematical convention).
        target_yaw_to_center_rad = math.atan2(vec_y, vec_x)
        target_yaw_to_center_deg = math.degrees(target_yaw_to_center_rad)

        # Assuming current_yaw (from odom_cb) is also 0 along +X, increasing CCW
        # Convert target_yaw to 0-360 range
        target_yaw_to_center_norm = (target_yaw_to_center_deg + 360) % 360

        # Only update target_yaw if it's significantly different, to avoid jitter
        if self.target_yaw is None or abs((self.target_yaw - target_yaw_to_center_norm + 360) % 360) > ANGLE_TOLERANCE / 2:
            self.target_yaw = target_yaw_to_center_norm
            self.get_logger().info(
                f"Returning to center. Recalculating target yaw to {self.target_yaw:.1f}°.")
            # Brief stop to re-orient if target yaw significantly changes
            if self.transition_stop_end_time <= time.time():
                self.publish_twist(0.0, 0.0)
                self.transition_stop_end_time = time.time() + 0.1  # Very brief pause

        angle_diff = (self.target_yaw - self.current_yaw + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360 # Adjust to shortest turn (-180 to 180)

        abs_angle_diff = abs(angle_diff)

        # Logging for debugging
        self.get_logger().debug(f"Return to Center: Current Yaw: {self.current_yaw:.1f}°, Target Yaw: {self.target_yaw:.1f}°, Angle Diff: {angle_diff:.1f}°, Abs Diff: {abs_angle_diff:.1f}°")

        linear_speed = 0.0
        angular_speed = 0.0

        if abs_angle_diff > ANGLE_TOLERANCE:
            # If we are outside the angle tolerance, turn only (or mostly turn)
            angular_speed = math.copysign(math.radians(RETURN_TO_CENTER_TURN_SPEED_DEG), angle_diff)
            # Reduce linear speed significantly or stop completely while turning
            linear_speed = RETURN_TO_CENTER_SPEED * 0.1 # Move slowly while turning
            # Consider linear_speed = 0.0 for pure turning
        else:
            # If we are aligned, drive forward towards the center
            linear_speed = RETURN_TO_CENTER_SPEED
            angular_speed = 0.0 # No angular correction needed if within tolerance

        self.publish_twist(linear_speed, angular_speed)

    def near_boundary(self, buffer_distance):
        # General check for being within buffer_distance of any wall
        # This function implicitly assumes map coordinates start at (0,0)
        return (self.current_x < buffer_distance or
                self.current_x > MAP_SIZE - buffer_distance or
                self.current_y < buffer_distance or
                self.current_y > MAP_SIZE - buffer_distance)

    def is_within_map_bounds(self):
        """Checks if the robot is within the defined MAP_SIZE boundaries (0 to MAP_SIZE for X and Y)."""
        return (0 <= self.current_x <= MAP_SIZE and 0 <= self.current_y <= MAP_SIZE)


    def is_heading_towards_boundary(self, check_buffer):
        """
        Checks if the robot's current heading is roughly pointing towards a boundary
        within a given `check_buffer`.
        Returns True if a boundary is in the path, False otherwise.
        """
        yaw_rad = math.radians(self.current_yaw)
        # Assuming ROS convention: 0 deg is +X, 90 deg is +Y (CCW)
        dir_x = math.cos(yaw_rad) # Corrected for ROS X-forward convention
        dir_y = math.sin(yaw_rad) # Corrected for ROS Y-left convention

        # Check for proximity to each wall and if heading towards it
        # Min Y wall (bottom)
        if self.current_y < check_buffer:
            if dir_y < 0:  # Moving towards Y_MIN
                return True
        # Max Y wall (top)
        elif self.current_y > MAP_SIZE - check_buffer:
            if dir_y > 0:  # Moving towards Y_MAX
                return True

        # Min X wall (left)
        if self.current_x < check_buffer:
            if dir_x < 0:  # Moving towards X_MIN
                return True
        # Max X wall (right)
        elif self.current_x > MAP_SIZE - check_buffer:
            if dir_x > 0:  # Moving towards X_MAX
                return True

        return False

    def predictive_boundary_check(self):
        """Enhanced predictive check with directional awareness"""
        # Only activate predictive avoidance if we are within PREDICTIVE_BUFFER but NOT already at the hard boundary
        if self.near_boundary(PREDICTIVE_BUFFER) and not self.near_boundary(BOUNDARY_BUFFER):
            return self.is_heading_towards_boundary(PREDICTIVE_BUFFER)
        return False

    def _calculate_predictive_avoidance_yaw(self):
        """Calculates a target yaw that steers away from the closest wall segment."""
        # Get angle towards center
        angle_to_center = self._calculate_general_inward_direction()

        # Check current heading relative to walls
        yaw_rad = math.radians(self.current_yaw)
        dir_x = math.cos(yaw_rad) # Corrected
        dir_y = math.sin(yaw_rad) # Corrected

        # Decide best escape angle
        escape_angle = None

        # These escape angles are relative adjustments.
        # If current_yaw is 0 (facing +X), turning +90 would be +Y.
        # If current_yaw is 90 (facing +Y), turning +90 would be -X.

        # Near left wall (low X), heading left (negative X direction)
        if self.current_x < PREDICTIVE_BUFFER and dir_x < 0:
            escape_angle = (self.current_yaw + random.uniform(90, 180)) % 360 # Turn towards +Y or -Y

        # Near right wall (high X), heading right (positive X direction)
        elif self.current_x > MAP_SIZE - PREDICTIVE_BUFFER and dir_x > 0:
            escape_angle = (self.current_yaw - random.uniform(90, 180)) % 360 # Turn towards +Y or -Y

        # Near bottom wall (low Y), heading down (negative Y direction)
        elif self.current_y < PREDICTIVE_BUFFER and dir_y < 0:
            escape_angle = (self.current_yaw - random.uniform(90, 180)) % 360 # Turn towards +X or -X

        # Near top wall (high Y), heading up (positive Y direction)
        elif self.current_y > MAP_SIZE - PREDICTIVE_BUFFER and dir_y > 0:
            escape_angle = (self.current_yaw + random.uniform(90, 180)) % 360 # Turn towards +X or -X

        if escape_angle is not None:
            # Blend the escape angle with the angle towards the center
            angle_diff_to_center = (angle_to_center - escape_angle + 360) % 360
            if angle_diff_to_center > 180:
                angle_diff_to_center -= 360

            return (escape_angle + angle_diff_to_center * 0.2) % 360
        else:
            # Fallback if no specific boundary heading is detected
            return (self.current_yaw + random.uniform(-MAX_WANDERING_TURN_ANGLE, MAX_WANDERING_TURN_ANGLE)) % 360


    def _calculate_general_inward_direction(self):
        """Calculates the yaw angle (0-360) that points from the robot's current position
        directly towards the center of the map (MAP_CENTER)."""
        vec_x = MAP_CENTER[0] - self.current_x
        vec_y = MAP_CENTER[1] - self.current_y

        # atan2(y, x) returns angle in radians relative to positive X-axis (standard math convention).
        angle_rad = math.atan2(vec_y, vec_x)
        angle_deg = math.degrees(angle_rad)

        # ROS convention for yaw: 0 deg is +X, 90 deg is +Y (CCW).
        # So, the angle from atan2 directly corresponds to the target yaw.
        target_yaw_angle = (angle_deg + 360) % 360
        return target_yaw_angle

    def execute_smooth_turn(self, next_state, linear_speed):
        angle_diff = (self.target_yaw - self.current_yaw + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360

        abs_angle_diff = abs(angle_diff)

        if abs_angle_diff < ANGLE_TOLERANCE:
            self.publish_twist(0.0, 0.0)
            self.auto_state = next_state
            self.start_position = (self.current_x, self.current_y)
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.target_yaw = None
            self.get_logger().info(
                f"Turn complete. Transitioning to {next_state.name}.")
            return

        if abs_angle_diff > MAX_TURN_ANGLE_FOR_FULL_SPEED:
            turn_factor = 1.0
        else:
            normalized_diff = abs_angle_diff / MAX_TURN_ANGLE_FOR_FULL_SPEED
            turn_factor = max(MIN_TURN_SPEED_FACTOR, normalized_diff**1.5)

        angular_speed = math.copysign(TURN_SPEED_DEG * turn_factor, angle_diff)

        self.publish_twist(linear_speed, math.radians(angular_speed))

    def publish_twist(self, linear, angular):
        cmd = Twist()
        # Add minimum thresholds for real robots to overcome friction
        MIN_LINEAR_CMD = 0.05 # Minimum linear speed command
        MIN_ANGULAR_CMD = math.radians(5.0) # Minimum angular speed command (5 degrees/sec)

        if abs(linear) > 0 and abs(linear) < MIN_LINEAR_CMD:
            cmd.linear.x = math.copysign(MIN_LINEAR_CMD, linear)
        else:
            cmd.linear.x = float(linear)

        if abs(angular) > 0 and abs(angular) < MIN_ANGULAR_CMD:
            cmd.angular.z = math.copysign(MIN_ANGULAR_CMD, angular)
        else:
            cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()