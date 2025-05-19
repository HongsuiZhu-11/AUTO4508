#!/usr/bin/env python3
import math
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
    NONE = 0
    AUTO = 1
    MANUAL = 2


class FOLLOW_MODE(Enum):
    FINDING = 0
    FOLLOWING = 1
    REACHED = 2

# --- Lidar Helper ---


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

# --- Control Node ---


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

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
        self.drive_mode = DRIVE_MODE.MANUAL
        self.follow_mode = FOLLOW_MODE.FINDING
        self.current_wp = 0
        self.angle = 0.0           # current heading (deg)
        self.is_turning = False
        self.target_heading = None
        self.turn_start = None
        self.turn_duration = 0.0
        self.lidar = LidarScan()
        self.trigger = False
        self.angular_speed = 2.0     # rad/s
        self.time_factor = 1.15

        # Timer
        self.create_timer(0.25, self.heartbeat_cb)
        self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg: Odometry):
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        yaw_deg = math.degrees(yaw)
        self.angle = (90 - yaw_deg) % 360

    def lidar_cb(self, msg: LaserScan):
        self.lidar.update(msg)
        if self.lidar.min_distance() < LIDAR_STOP_DISTANCE:
            self.publish_twist(0.0, 0.0)

    def gps_cb(self, msg: NavSatFix):
        self.drive_mode = DRIVE_MODE.AUTO
        self.is_turning = False
        if self.drive_mode != DRIVE_MODE.AUTO:
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        # compute waypoint info
        if self.current_wp < len(WAY_POINTS):
            tgt_lat, tgt_lon = WAY_POINTS[self.current_wp]
            dist, abs_bear, rel_bear = self.get_relative(
                self.current_lat, self.current_lon, tgt_lat, tgt_lon, self.angle)
        else:
            dist = rel_bear = None

        # mid-turn gating: wait until both time and heading met
        if self.is_turning:
            elapsed = (self.get_clock().now() -
                       self.turn_start).nanoseconds / 1e9
            raw_err = ((self.angle - self.target_heading + 180) % 360) - 180
            heading_err = abs(raw_err)
            if elapsed >= self.turn_duration and heading_err <= ANGLE_MARGIN:
                self.is_turning = False
                self.follow_mode = FOLLOW_MODE.FOLLOWING
                self.get_logger().info('Turn complete')
            else:
                return

        # waypoint handling
        if self.current_wp < len(WAY_POINTS):
            if self.follow_mode == FOLLOW_MODE.FINDING:
                if abs(rel_bear) > ANGLE_MARGIN and not self.is_turning:
                    self.send_turn(rel_bear, abs_bear)
                else:
                    self.follow_mode = FOLLOW_MODE.FOLLOWING
            elif self.follow_mode == FOLLOW_MODE.FOLLOWING:
                if dist > DEST_MARGIN:
                    self.dist_pub.publish(Float32(data=dist))
                else:
                    self.current_wp += 1
                    self.follow_mode = FOLLOW_MODE.FINDING
            return

    def joy_cb(self, msg: Joy):
        # B button: manual, Square: auto
        if msg.buttons[1]:
            self.drive_mode = DRIVE_MODE.MANUAL
        if msg.buttons[2]:
            self.drive_mode = DRIVE_MODE.AUTO
        # trigger axis
        self.trigger = msg.axes[5] < 0

    def heartbeat_cb(self):
        hb = Int32(data=1 if self.trigger else 0)
        self.heartbeat_pub.publish(hb)

    def send_turn(self, rel_deg: float, abs_deg: float):
        rad = math.radians(rel_deg)
        self.turn_duration = abs(rad) / self.angular_speed * self.time_factor
        #self.target_heading = (self.angle + rel_deg) % 360
        self.target_heading = abs_deg % 360
        self.turn_start = self.get_clock().now()
        self.is_turning = True
        self.angle_pub.publish(Float32(data=rad))

    def publish_twist(self, lin: float, ang: float):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

    @staticmethod
    def get_relative(lat1, lon1, lat2, lon2, current_heading):
        R = 6371000.0
        r1, r2 = math.radians(lat1), math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(r2)
        x = math.cos(r1) * math.sin(r2) - math.sin(r1) * \
            math.cos(r2) * math.cos(dlon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        rel = ((bearing - current_heading) + 180) % 360 - 180
        dlat = r2 - r1
        a = math.sin(dlat/2)**2 + math.cos(r1) * \
            math.cos(r2) * math.sin(dlon/2)**2
        dist = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return dist, bearing, rel

    def control_loop(self):
        # Only process in AUTO mode
        if self.drive_mode != DRIVE_MODE.AUTO:
            return

        # Skip if no waypoints or no GPS data yet
        if self.current_wp >= len(WAY_POINTS) or not hasattr(self, 'current_lat'):
            return

        # Retrieve target waypoint
        tgt_lat, tgt_lon = WAY_POINTS[self.current_wp]

        # Get current position and heading
        lat, lon = self.current_lat, self.current_lon
        dist, abs_bear, rel_bear = self.get_relative(lat, lon, tgt_lat, tgt_lon, self.angle)

        # Check turn completion
        if self.is_turning:
            elapsed = (self.get_clock().now() - self.turn_start).nanoseconds / 1e9
            heading_err = abs(((self.angle - self.target_heading + 180) % 360) - 180)
            if elapsed >= self.turn_duration and heading_err <= ANGLE_MARGIN:
                self.is_turning = False
                self.follow_mode = FOLLOW_MODE.FOLLOWING
                self.get_logger().info('Turn complete')
            else:
                return  # Still turning, wait

        # Waypoint handling logic
        if self.follow_mode == FOLLOW_MODE.FINDING:
            if abs(rel_bear) > ANGLE_MARGIN:
                self.send_turn(rel_bear)
            else:
                self.follow_mode = FOLLOW_MODE.FOLLOWING
        elif self.follow_mode == FOLLOW_MODE.FOLLOWING:
            if dist > DEST_MARGIN:
                self.dist_pub.publish(Float32(data=dist))
            else:
                self.current_wp += 1
                self.follow_mode = FOLLOW_MODE.FINDING

def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
