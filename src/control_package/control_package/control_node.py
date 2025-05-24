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
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from tf_transformations import euler_from_quaternion

# Configuration
# Degrees for initial left turn (can remain fixed or be randomized)
INITIAL_TURN_ANGLE = 90
INITIAL_DRIVE_DISTANCE = 2.5  # meters (increased for longer drive)
BOUNDARY_BUFFER = 1.0  # meters from edge to trigger turn
# meters (Assuming a square map from (0,0) to (MAP_SIZE, MAP_SIZE))
MAP_SIZE = 15.0

# Speeds
# degrees/second (Increased for faster turns when avoiding)
TURN_SPEED_DEG = 50
FORWARD_SPEED = 1  # m/s
ANGLE_TOLERANCE = 5.0  # degrees - used for turn completion

# Transition delay
# seconds (Increased for more reliable stops between actions)
TRANSITION_STOP_DURATION = 0.5

# Turning parameters for smoother control
MIN_TURN_SPEED_FACTOR = 0.2
MAX_TURN_ANGLE_FOR_FULL_SPEED = 45.0

# Wandering behavior tuning
MIN_DRIVE_DISTANCE = 2.0  # increased for longer drive
MAX_DRIVE_DISTANCE = 5.0  # increased for longer drive
# Changed: Allow turns both left and right for general wandering
# Max absolute angle for a random wandering turn (increased for more variety)
MAX_WANDERING_TURN_ANGLE = 120.0

# Boundary Avoidance Specifics
# Drive distance after an avoidance turn to ensure robot clears the boundary area
BOUNDARY_ESCAPE_DRIVE_DISTANCE = BOUNDARY_BUFFER * \
    2.5  # Drive further away from boundary


class DRIVE_MODE(Enum):
    MANUAL = 1
    AUTO = 2


class AUTO_STATE(Enum):
    INITIAL_TURN = 0
    INITIAL_DRIVE = 1
    WANDERING_TURN = 2
    WANDERING_DRIVE = 3


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.mutex = threading.Lock()

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_team10', 10)
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.drive_mode = DRIVE_MODE.AUTO
        self.auto_state = AUTO_STATE.INITIAL_TURN
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_yaw = None
        self.start_position = (0.0, 0.0)
        self.drive_distance = 0.0
        # This variable is not strictly necessary but kept for context
        self.wandering_target_yaw = None

        self.turn_start_time = None
        self.turn_duration = 0.0

        self.transition_stop_end_time = 0.0

        self.last_linear = 0.0
        self.last_angular = 0.0

        # NEW: Flag for boundary escape drive
        self.boundary_avoidance_in_progress = False

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Starting exploration routine")

    def joy_cb(self, msg):
        with self.mutex:
            if msg.buttons[1]:  # B button (Xbox)
                self.drive_mode = DRIVE_MODE.MANUAL
                self.publish_twist(0.0, 0.0)
                self.get_logger().info("Manual mode activated")
            if msg.buttons[0]:  # A button (Xbox)
                self.drive_mode = DRIVE_MODE.AUTO
                self.auto_state = AUTO_STATE.INITIAL_TURN
                self.target_yaw = None
                # Reset boundary avoidance flag when entering auto mode
                self.boundary_avoidance_in_progress = False
                self.get_logger().info("Auto mode activated. Restarting initial turn.")

            if self.drive_mode == DRIVE_MODE.MANUAL:
                lin_input = msg.axes[1]
                ang_input = msg.axes[3]

                linear = -(lin_input ** 3) * 0.8
                angular = -(ang_input ** 3) * 1.5

                self.publish_twist(linear, angular)

    def odom_cb(self, msg):
        with self.mutex:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y

            orientation = msg.pose.pose.orientation
            _, _, yaw_rad = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            # Ensure yaw is always positive (0-360) for consistent calculations
            self.current_yaw = (math.degrees(yaw_rad) + 360) % 360

    def control_loop(self):
        with self.mutex:
            if self.drive_mode != DRIVE_MODE.AUTO:
                return

            # Prioritize stopping for transitions
            if time.time() < self.transition_stop_end_time:
                self.publish_twist(0.0, 0.0)
                return

            # Check for boundary near during driving states and trigger turn
            # IMPORTANT: Only trigger boundary avoidance if NOT currently in an escape drive
            if self.auto_state in [AUTO_STATE.INITIAL_DRIVE, AUTO_STATE.WANDERING_DRIVE] and \
               self.near_boundary() and not self.boundary_avoidance_in_progress:
                self.get_logger().warn("Boundary detected during drive, initiating avoidance turn.")
                self.auto_state = AUTO_STATE.WANDERING_TURN  # Force transition to turn state
                self.target_yaw = None  # Reset target yaw to calculate avoidance turn
                self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
                self.boundary_avoidance_in_progress = True  # Set flag for escape drive
                return  # Exit to let transition delay pass

            # State handling
            if self.auto_state == AUTO_STATE.INITIAL_TURN:
                self.handle_initial_turn()
            elif self.auto_state == AUTO_STATE.INITIAL_DRIVE:
                self.handle_initial_drive()
            elif self.auto_state == AUTO_STATE.WANDERING_TURN:
                self.handle_wandering_turn()
            elif self.auto_state == AUTO_STATE.WANDERING_DRIVE:
                self.handle_wandering_drive()

    def handle_initial_turn(self):
        if self.target_yaw is None:
            self.target_yaw = (self.current_yaw + INITIAL_TURN_ANGLE) % 360
            self.get_logger().info(
                f"Initial turn to {self.target_yaw:.1f}° from {self.current_yaw:.1f}°")
            self.publish_twist(0.0, 0.0)  # Ensure stop before turn
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            return  # Exit to let transition delay pass

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
        # Initialize linear_speed_for_turn with a default (e.g., 0.0 for spin-in-place)
        # It will be overridden if self.target_yaw is None
        linear_speed_for_turn = 0.0

        if self.target_yaw is None:
            # Determine if this turn is for boundary avoidance or general wandering
            if self.near_boundary():
                self.target_yaw = self._calculate_boundary_avoidance_yaw()
                self.get_logger().info(
                    f"Boundary avoidance turn to {self.target_yaw:.1f}° from {self.current_yaw:.1f}°.")
                # Set distance for the following drive
                self.drive_distance = BOUNDARY_ESCAPE_DRIVE_DISTANCE
                linear_speed_for_turn = 0.0  # Spin in place for precise boundary avoidance
                # Set the flag to indicate we're entering a boundary escape phase
                self.boundary_avoidance_in_progress = True
            else:
                random_angle = random.uniform(-MAX_WANDERING_TURN_ANGLE,
                                              MAX_WANDERING_TURN_ANGLE)
                self.target_yaw = (self.current_yaw + random_angle) % 360
                self.drive_distance = random.uniform(
                    MIN_DRIVE_DISTANCE, MAX_DRIVE_DISTANCE)
                self.get_logger().info(
                    f"Wandering turn to {self.target_yaw:.1f}° from {self.current_yaw:.1f}° (drive {self.drive_distance:.1f}m).")
                linear_speed_for_turn = FORWARD_SPEED  # Arc turn for general wandering
                # Ensure flag is false for general wandering turns
                self.boundary_avoidance_in_progress = False

            self.wandering_target_yaw = self.target_yaw
            # Ensure robot stops before initiating a turn
            self.publish_twist(0.0, 0.0)
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            return  # Exit to let transition delay pass

        # If we reach here, a turn target is already set.
        # Determine linear_speed_for_turn based on whether this turn was initiated for boundary avoidance.
        # We can infer this by checking the flag `self.boundary_avoidance_in_progress`.
        if self.boundary_avoidance_in_progress:
            linear_speed_for_turn = 0.0  # Still spinning in place for avoidance
        else:
            linear_speed_for_turn = FORWARD_SPEED  # Continue arc turn for general wandering

        self.execute_smooth_turn(
            AUTO_STATE.WANDERING_DRIVE, linear_speed_for_turn)

    def handle_wandering_drive(self):
        distance = math.hypot(self.current_x - self.start_position[0],
                              self.current_y - self.start_position[1])

        # Determine the target distance based on whether we are escaping a boundary
        target_drive_distance = self.drive_distance
        if self.boundary_avoidance_in_progress:
            target_drive_distance = BOUNDARY_ESCAPE_DRIVE_DISTANCE

        if distance >= target_drive_distance:
            self.auto_state = AUTO_STATE.WANDERING_TURN
            self.target_yaw = None
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION

            # Reset the boundary avoidance flag once the escape drive is complete
            if self.boundary_avoidance_in_progress:
                self.boundary_avoidance_in_progress = False
                self.get_logger().info("Boundary escape drive complete. Starting next wandering turn.")
            else:
                self.get_logger().info("Wandering drive distance met. Starting next wandering turn.")
        else:
            self.publish_twist(FORWARD_SPEED, 0.0)

    def near_boundary(self):
        # Assuming map from (0,0) to (MAP_SIZE, MAP_SIZE)
        return (self.current_x < BOUNDARY_BUFFER or
                self.current_x > MAP_SIZE - BOUNDARY_BUFFER or
                self.current_y < BOUNDARY_BUFFER or
                self.current_y > MAP_SIZE - BOUNDARY_BUFFER)

    def _calculate_boundary_avoidance_yaw(self):
        """
        Calculates the target yaw to turn away from the closest boundary.
        Assumes 0 degrees yaw is along positive Y-axis, 90 degrees along positive X-axis.
        """
        target_yaw_deg = self.current_yaw  # Default if no clear direction

        # Check proximity to each boundary
        near_x_min = self.current_x < BOUNDARY_BUFFER
        near_x_max = self.current_x > MAP_SIZE - BOUNDARY_BUFFER
        near_y_min = self.current_y < BOUNDARY_BUFFER
        near_y_max = self.current_y > MAP_SIZE - BOUNDARY_BUFFER

        # Prioritize corners for a diagonal escape
        # Bottom-left corner (turn towards +X, +Y)
        if near_x_min and near_y_min:
            target_yaw_deg = 45.0
        # Bottom-right corner (turn towards -X, +Y)
        elif near_x_max and near_y_min:
            target_yaw_deg = 135.0
        # Top-left corner (turn towards +X, -Y)
        elif near_x_min and near_y_max:
            target_yaw_deg = 315.0  # Or -45.0
        # Top-right corner (turn towards -X, -Y)
        elif near_x_max and near_y_max:
            target_yaw_deg = 225.0
        # Then handle edges
        elif near_x_min:  # Near left edge, turn right (+X)
            target_yaw_deg = 90.0
        elif near_x_max:  # Near right edge, turn left (-X)
            target_yaw_deg = 270.0
        elif near_y_min:  # Near bottom edge, turn up (+Y)
            target_yaw_deg = 0.0
        elif near_y_max:  # Near top edge, turn down (-Y)
            target_yaw_deg = 180.0

        return target_yaw_deg

    def execute_smooth_turn(self, next_state, linear_speed):
        """
        Executes a turn towards target_yaw with a specified linear_speed.
        Transitions to next_state when complete.
        """
        angle_diff = (self.target_yaw - self.current_yaw + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360  # Normalize to -180 to 180

        abs_angle_diff = abs(angle_diff)

        if abs_angle_diff < ANGLE_TOLERANCE:
            self.publish_twist(0.0, 0.0)  # Stop turning
            self.auto_state = next_state
            # Reset start position for next drive
            self.start_position = (self.current_x, self.current_y)
            self.transition_stop_end_time = time.time() + TRANSITION_STOP_DURATION
            self.target_yaw = None
            self.get_logger().info(
                f"Turn complete. Transitioning to {next_state.name}.")
            return  # Turn completed

        # Determine angular speed based on angle difference
        if abs_angle_diff > MAX_TURN_ANGLE_FOR_FULL_SPEED:
            turn_factor = 1.0
        else:
            normalized_diff = abs_angle_diff / MAX_TURN_ANGLE_FOR_FULL_SPEED
            turn_factor = max(MIN_TURN_SPEED_FACTOR, normalized_diff**1.5)

        # Determine turn direction
        angular_speed = math.copysign(TURN_SPEED_DEG * turn_factor, angle_diff)

        self.publish_twist(linear_speed, math.radians(angular_speed))

    def publish_twist(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
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
