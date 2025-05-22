#!/usr/bin/env python3
import math
import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan, NavSatFix
from std_msgs.msg import Float32, Int32
from tf_transformations import euler_from_quaternion

# --- Configuration ---
WAY_POINTS = [
    (-31.980327, 115.817317),
    (-31.980554, 115.817743),
    (-31.980368, 115.818476)
]
DEST_MARGIN = 1.0       # meters
ANGLE_MARGIN = 5.0      # degrees
LIDAR_STOP_DISTANCE = 1.0  # meters

# --- Enums ---


class DRIVE_MODE(Enum):
    NONE = 1000
    AUTO = 1001
    MANUAL = 1002
    PENDING = 1003


class FOLLOW_MODE(Enum):
    NONE = 3000
    FOLLOWING = 3001
    FINDING = 3002


class LidarScan:
    def __init__(self):
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.ranges = []

    def update(self, msg: LaserScan):
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.ranges = msg.ranges

    def min_distance(self):
        valid = [d for d in self.ranges if math.isfinite(d)]
        return min(valid) if valid else float('inf')


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.mutex = threading.Lock()

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_team10', 10)
        self.dist_pub = self.create_publisher(Float32, 'drive_distance', 10)
        self.angle_pub = self.create_publisher(Float32, 'turn_angle', 10)
        self.heartbeat_pub = self.create_publisher(
            Int32, 'heartbeat_team10', 10)

        # Subscribers
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        self.create_subscription(NavSatFix, 'fix', self.gps_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        # State
        with self.mutex:
            self.drive_mode = DRIVE_MODE.AUTO
            self.follow_mode = FOLLOW_MODE.FINDING
            self.current_wp = 0
            self.angle = 0.0
            self.current_lat = None
            self.current_lon = None
            self.angular_speed = 2.0
            self.time_factor = 1.15
            # Add angle tracking variables
            self.initial_angle = None
            self.target_rel_angle = 0.0

        self.lidar = LidarScan()
        self.trigger = False
        self.last_linear = 0.0
        self.last_angular = 0.0

        # Tuning
        self.turn_speed = 0.5
        self.forward_speed = 0.3
        self.distance_tolerance = DEST_MARGIN
        self.turn_tolerance = ANGLE_MARGIN

        # Timer
        self.create_timer(0.25, self.heartbeat_cb)

    def lidar_cb(self, msg: LaserScan):
        self.lidar.update(msg)
        if self.lidar.min_distance() < LIDAR_STOP_DISTANCE:
            self.publish_twist(0.0, 0.0)

    def joy_cb(self, msg: Joy):
        with self.mutex:
            # --- Mode switching ---
            if msg.buttons[0]:  # X (PS4)
                if self.drive_mode != DRIVE_MODE.AUTO:
                    self.get_logger().info("Switching to AUTO mode")
                    self.drive_mode = DRIVE_MODE.AUTO
                    self.current_wp = 0
                    self.follow_mode = FOLLOW_MODE.FINDING
                    self.publish_twist(0.0, 0.0)

            elif msg.buttons[1]:  # Circle (PS4)
                if self.drive_mode != DRIVE_MODE.MANUAL:
                    self.get_logger().info("Switching to MANUAL mode")
                    self.drive_mode = DRIVE_MODE.MANUAL
                    self.publish_twist(0.0, 0.0)

            # --- Deadman (R2/L2) ---
            # R2 trigger (fully pressed ~ -1.0)
            deadman_pressed = msg.axes[5] < - \
                0.9 if len(msg.axes) > 5 else False

            if not deadman_pressed:
                if self.trigger:
                    self.publish_twist(0.0, 0.0)
                    self.get_logger().warn("Deadman released! Stopping.")
                self.trigger = False
                return

            self.trigger = True

            # --- Manual control ---
            if self.drive_mode == DRIVE_MODE.MANUAL:
                lin_input = msg.axes[1]  # Left stick vertical
                ang_input = msg.axes[3]  # Right stick horizontal

                # Cubic scaling for finer control
                linear = -(lin_input ** 3) * 0.8
                angular = -(ang_input ** 3) * 1.5

                # Ramping
                if abs(linear - self.last_linear) > 0.1:
                    linear = self.last_linear + \
                        math.copysign(0.1, linear - self.last_linear)
                if abs(angular - self.last_angular) > 0.15:
                    angular = self.last_angular + \
                        math.copysign(0.15, angular - self.last_angular)

                self.last_linear = linear
                self.last_angular = angular

                self.publish_twist(linear, angular)

    def odom_cb(self, msg: Odometry):
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        yaw_deg = math.degrees(yaw)
        with self.mutex:
            current_angle = (90 - yaw_deg) % 360
            # Update angle tracking
            if self.drive_mode == DRIVE_MODE.AUTO and self.follow_mode == FOLLOW_MODE.FINDING:
                if self.initial_angle is not None:
                    # Calculate angle difference
                    delta = current_angle - self.initial_angle
                    # Normalize to [-180, 180]
                    delta = (delta + 180) % 360 - 180

                    # Check if we've turned enough
                    if abs(delta - self.target_rel_angle) <= self.turn_tolerance:
                        self.get_logger().info("Target angle reached. Switching to FOLLOWING.")
                        self.follow_mode = FOLLOW_MODE.FOLLOWING
                        self.initial_angle = None
                        self.publish_twist(0.0, 0.0)
            self.angle = current_angle

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        with self.mutex:
            if self.drive_mode != DRIVE_MODE.AUTO:
                return

            if self.current_wp >= len(WAY_POINTS):
                self.get_logger().info("All waypoints completed.")
                self.publish_twist(0.0, 0.0)
                return

            wp_lat, wp_lon = WAY_POINTS[self.current_wp]
            dist, bearing, rel_angle = self.get_relative(
                self.current_lat, self.current_lon, wp_lat, wp_lon, self.angle)

            self.get_logger().info(
                f"Current:lat.{self.current_lat},lon.{self.current_lon}, Target:lat.{wp_lat},lon.{wp_lon}")

            if self.follow_mode == FOLLOW_MODE.FINDING:
                if self.initial_angle is None:
                    # Initialize turn
                    self.initial_angle = self.angle
                    self.target_rel_angle = rel_angle
                    turn_dir = -self.turn_speed if rel_angle > 0 else self.turn_speed
                    self.publish_twist(0.0, turn_dir)
                    self.get_logger().info(
                        f"Starting turn: {rel_angle:.1f} degrees")

            elif self.follow_mode == FOLLOW_MODE.FOLLOWING:
                if dist < self.distance_tolerance:
                    self.get_logger().info("Waypoint reached.")
                    self.current_wp += 1
                    if self.current_wp < len(WAY_POINTS):
                        self.follow_mode = FOLLOW_MODE.FINDING
                        self.initial_angle = None  # Reset for next waypoint
                        self.get_logger().info("Next waypoint set. Turning to align.")
                    else:
                        self.get_logger().info("Final waypoint reached.")
                        self.publish_twist(0.0, 0.0)
                        self.drive_mode = DRIVE_MODE.NONE
                else:
                    self.publish_twist(self.forward_speed, 0.0)

    def get_relative(self, lat1, lon1, lat2, lon2, current_heading):
        R = 6371000.0
        r1, r2 = math.radians(lat1), math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(r2)
        x = math.cos(r1) * math.sin(r2) - math.sin(r1) * \
            math.cos(r2) * math.cos(dlon)
        bearing = math.degrees(math.atan2(y, x)) % 360
        rel = ((bearing - current_heading) + 180) % 360 - 180
        dlat = r2 - r1
        a = math.sin(dlat/2)**2 + math.cos(r1) * \
            math.cos(r2) * math.sin(dlon/2)**2
        dist = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return dist, bearing, rel

    def publish_twist(self, lin: float, ang: float):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

    def heartbeat_cb(self):
        hb = Int32(data=1 if self.trigger else 0)
        self.heartbeat_pub.publish(hb)


def main():
    rclpy.init()
    rclpy.spin(ControlNode())
    rclpy.shutdown()
