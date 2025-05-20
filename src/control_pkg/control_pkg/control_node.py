#!/usr/bin/env python3
import math
from enum import Enum
import threading

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
    NONE = 0
    AUTO = 1
    MANUAL = 2


class STATE(Enum):
    IDLE = 0
    TURNING = 1
    DRIVING = 2
    REACHED = 3


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

        # Protected state
        with self.mutex:
            self.drive_mode = DRIVE_MODE.AUTO
            self.state = STATE.IDLE
            self.current_wp = 0
            self.angle = 0.0
            self.current_lat = None
            self.current_lon = None
            self.target_heading = None
            self.angular_speed = 2.0
            self.time_factor = 1.15
            self.last_action_time = 0

        self.lidar = LidarScan()
        self.trigger = False
        self.last_linear = 0.0
        self.last_angular = 0.0

        # Timers
        self.create_timer(0.25, self.heartbeat_cb)
        self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg: Odometry):
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        yaw_deg = math.degrees(yaw)
        with self.mutex:
            self.angle = (90 - yaw_deg) % 360

    def lidar_cb(self, msg: LaserScan):
        self.lidar.update(msg)
        if self.lidar.min_distance() < LIDAR_STOP_DISTANCE:
            self.publish_twist(0.0, 0.0)

    def gps_cb(self, msg: NavSatFix):
        with self.mutex:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude

    def joy_cb(self, msg: Joy):
        with self.mutex:
            # --- Mode Switching ---
            # Button 'X' (msg.buttons[0]) enables AUTO mode
            if msg.buttons[0]:  # X button (PS4) / A button (Xbox)
                if self.drive_mode != DRIVE_MODE.AUTO:
                    self.get_logger().info('Switching to AUTO mode.')
                    self.drive_mode = DRIVE_MODE.AUTO
                    self.state = STATE.IDLE
                    self.current_wp = 0
                    self.publish_twist(0.0, 0.0)

            # Button 'O' (msg.buttons[1]) enables MANUAL mode
            elif msg.buttons[1]:  # O button (PS4) / B button (Xbox)
                if self.drive_mode != DRIVE_MODE.MANUAL:
                    self.get_logger().info('Switching to MANUAL mode.')
                    self.drive_mode = DRIVE_MODE.MANUAL
                    self.state = STATE.IDLE
                    self.publish_twist(0.0, 0.0)

            # --- Dead-Man Switch ---
            # Use L1 button (msg.buttons[4]) as deadman switch
            dead_man_engaged = msg.buttons[4]  # L1 button (PS4) / LB button (Xbox)

            # Emergency stop if deadman released
            if not dead_man_engaged:
                if self.trigger:  # Only publish stop once
                    self.publish_twist(0.0, 0.0)
                    self.get_logger().warn('Deadman released! Emergency stop.')
                self.trigger = False
                return

            self.trigger = True

            # --- Manual Control ---
            if self.drive_mode == DRIVE_MODE.MANUAL:
                # Standard teleop_twist_joy configuration:
                # Left stick vertical (axes[1]) for linear velocity
                # Right stick horizontal (axes[3]) for angular velocity

                # Apply cubic scaling for finer control
                linear_raw = msg.axes[1]
                angular_raw = msg.axes[3]

                # Invert axes if needed and apply scaling
                linear_vel = -(linear_raw ** 3) * 0.8  # Max 0.8 m/s
                angular_vel = -(angular_raw ** 3) * 1.5  # Max 1.5 rad/s

                # Apply smooth ramping
                if abs(linear_vel - self.last_linear) > 0.1:
                    linear_vel = self.last_linear + \
                        math.copysign(0.1, linear_vel - self.last_linear)
                if abs(angular_vel - self.last_angular) > 0.15:
                    angular_vel = self.last_angular + \
                        math.copysign(0.15, angular_vel - self.last_angular)

                self.last_linear = linear_vel
                self.last_angular = angular_vel

                self.publish_twist(linear_vel, angular_vel)

    def publish_twist(self, lin: float, ang: float):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

    def control_loop(self):
        with self.mutex:
            if self.drive_mode != DRIVE_MODE.AUTO:
                return

            if self.state == STATE.REACHED or self.current_wp >= len(WAY_POINTS):
                return

            if self.current_lat is None or self.current_lon is None:
                return

            current_state = self.state
            tgt_lat, tgt_lon = WAY_POINTS[self.current_wp]
            angle = self.angle
            current_lat = self.current_lat
            current_lon = self.current_lon

        dist, abs_bear, rel_bear = self.get_relative(
            current_lat, current_lon, tgt_lat, tgt_lon, angle
        )

        with self.mutex:
            if current_state == STATE.IDLE:
                if abs(rel_bear) > ANGLE_MARGIN:
                    self.state = STATE.TURNING
                    self.target_heading = abs_bear % 360
                    self.expected_turn_duration = abs(math.radians(
                        rel_bear)) / self.angular_speed * self.time_factor
                    self.last_action_time = self.get_clock().now()
                    rad = -math.radians(rel_bear)
                    self.angle_pub.publish(Float32(data=rad))
                    self.get_logger().info(
                        f'Turning {rel_bear:.1f}° (exp: {self.expected_turn_duration:.1f}s) to WP{self.current_wp}')
                else:
                    self.state = STATE.DRIVING
                    self.dist_pub.publish(Float32(data=dist))
                    self.get_logger().info(
                        f'Driving {dist:.1f}m to WP{self.current_wp}')
                    self.last_action_time = self.get_clock().now()

            elif current_state == STATE.TURNING:
                elapsed = (self.get_clock().now() -
                           self.last_action_time).nanoseconds / 1e9
                heading_err = (
                    (self.angle - self.target_heading + 180) % 360) - 180

                # Recalculate bearing during turn
                new_dist, new_abs_bear, new_rel_bear = self.get_relative(
                    self.current_lat, self.current_lon,
                    WAY_POINTS[self.current_wp][0],
                    WAY_POINTS[self.current_wp][1],
                    self.angle
                )

                # Update turn command if needed
                if abs(new_rel_bear) > ANGLE_MARGIN:
                    rad = -math.radians(new_rel_bear)
                    self.angle_pub.publish(Float32(data=rad))

                # Completion check (time + heading)
                if elapsed > self.expected_turn_duration and abs(heading_err) <= ANGLE_MARGIN:
                    self.state = STATE.DRIVING
                    self.dist_pub.publish(Float32(data=new_dist))
                    self.get_logger().info(
                        f'Completed turn to {self.target_heading:.1f}°')
                elif elapsed > self.expected_turn_duration * 1.5:  # Safety timeout
                    self.get_logger().warning(f'Turn timeout, proceeding anyway')
                    self.state = STATE.DRIVING
                    self.dist_pub.publish(Float32(data=new_dist))

            elif current_state == STATE.DRIVING:
                elapsed = (self.get_clock().now() -
                           self.last_action_time).nanoseconds / 1e9
                if elapsed > 1.0:  # Minimum drive time before rechecking
                    if dist <= DEST_MARGIN:
                        self.current_wp += 1
                        if self.current_wp >= len(WAY_POINTS):
                            self.state = STATE.REACHED
                            self.get_logger().info('All waypoints reached!')
                        else:
                            self.state = STATE.IDLE
                            self.get_logger().info(
                                f'Proceeding to WP{self.current_wp}')
                    elif abs(rel_bear) > ANGLE_MARGIN:
                        self.state = STATE.TURNING
                        self.target_heading = abs_bear % 360
                        rad = -math.radians(rel_bear)
                        self.angle_pub.publish(Float32(data=rad))
                        self.get_logger().info(
                            f'Recalculating bearing, turning {rel_bear:.1f}°')
                        self.last_action_time = self.get_clock().now()
                    else:
                        # Continue driving with updated distance
                        self.dist_pub.publish(Float32(data=dist))
                        self.last_action_time = self.get_clock().now()

    def get_relative(self, lat1, lon1, lat2, lon2, current_heading):
        # Same implementation as before
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

    def heartbeat_cb(self):
        hb = Int32(data=1 if self.trigger else 0)
        self.heartbeat_pub.publish(hb)


def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
