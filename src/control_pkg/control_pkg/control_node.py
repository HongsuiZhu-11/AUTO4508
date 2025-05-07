#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from gps_msgs.msg import GPSFix
# from interfaces.msg import Gpsx
# from gpsx.msg import Gpsx
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist

from enum import Enum
import math


WAY_POINTS = [(-31.980327, 115.817317), (-31.980554,
                                         115.817743), (-31.980368, 115.818476)]

DIST_MIN = 100000
DEST_MARGIN = 5
ANGLE_MARGIN = 5

# Lidar has some 0 reads and detects it's back plate, threshold in meters for minimum acceptable readings
MIN_LIDAR_MARGIN = 0.1
LIDAR_STOP_DISTANCE = 0.5


class DRIVE_MODE(Enum):
    NONE = 1000
    AUTO = 1001
    MANUAL = 1002
    PENDING = 1003


class FOWLLOW_MODE(Enum):
    NONE = 3000
    FOLLOWING = 3001
    FINDING = 3002


class LidarScan:
    def __init__(self):
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.ranges = []

    def scan_update(self, angle_min, angle_max, angle_increment, ranges):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.ranges = ranges

    def get_distance(self, angle_degrees):
        # Normalize angle to [-180, 180]
        angle_degrees = (angle_degrees + 180) % 360 - 180

        # Convert degrees to radians
        angle_radians = math.radians(angle_degrees)

        # Compute index
        if self.angle_increment == 0 or not self.ranges:
            return -1.0

        index = int((angle_radians - self.angle_min) / self.angle_increment)

        if 0 <= index < len(self.ranges) and math.isfinite(self.ranges[index]):
            return self.ranges[index]
        return -1.0

    def get_min_range(self):
        if not self.ranges:
            return -1.0
        return min((r for r in self.ranges if r > MIN_LIDAR_MARGIN), default=-1.0)


class ControlNode(Node):
    def __init__(self):
        super().__init__("Control_Node")

        # Subscribers
        self.create_subscription(NavSatFix, "fix", self.gps_callback, 10)
        self.create_subscription(Joy, "joy", self.joy_cb, 10)
        self.create_subscription(Twist, "cmd_vel", self.twist_cb, 10)
        self.create_subscription(LaserScan, "scan", self.lidar_cb, 10)

        # Publisher
        self.robot_pub = self.create_publisher(Twist, 'cmd_vel_team10', 10)
        self.heartbeat_pub = self.create_publisher(
            Int32, 'heartbeat_team10', 10)
        # self.report_pub = self.create_publisher(String, 'report_team10', 10)

        # Timer
        self.create_timer(0.04, self.timer_cb)
        self.create_timer(0.25, self.heartbeat_timer_cb)

        # Variables
        self.lat = 0
        self.long = 0
        self.angle = 0
        self.is_start = False

        self.current_point = 0

        self.drive_mode = DRIVE_MODE.MANUAL
        self.following_mode = FOWLLOW_MODE.NONE
        self.trigger = False

        self.linear = 0.0
        self.angualr = 0.0

        self.angle_counter = -1
        self.turning_angle = 0

        self.lidar = LidarScan()

    def gps_callback(self, msg):
        # print(msg)
        cur_lat = msg.latitude
        cur_long = msg.longitude

        if (type(cur_lat) == str or math.isnan(cur_lat) or int(cur_lat) != -31):
            return

        if self.drive_mode != DRIVE_MODE.AUTO:
            # print('not Driving', self.drive_mode)
            return

        if not self.trigger:
            # print('not trigger', self.trigger)
            return

        if self.angle_counter >= 0:
            # on Turning
            # print("on Turning", self_counter)
            return

        if self.lat == 0 or self.long == 0:
            print('set current gps')
            self.lat = cur_lat
            self.long = cur_long
            return

        target_lat = WAY_POINTS[self.current_point][0]
        target_long = WAY_POINTS[self.current_point][1]
        tart_dist, target_angle, rot = self.get_relative_dist(
            cur_lat, target_lat, cur_long, target_long)
        #print('dist', tart_dist, 'angle', target_angle,
        #      'cur_angle', self.angle, 'rot', rot)
        # check goal
        if tart_dist <= DEST_MARGIN:
            print('Arrive: ', self.current_point)

            control_msg = self.convert_msg(0.0, 0.0)
            self.robot_pub.publish(control_msg)
            self.is_start = False
            # TODO Find Optical
            # while Object not not in center of frame
            #   Turn toward Object
            #       read Lidar at 0 (forward)

            # self.following_mode = FOWLLOW_MODE.FINDING
            self.current_point += 1
            return

        if not self.is_start and self.following_mode != FOWLLOW_MODE.FINDING:
            self.is_start = True
            self.following_mode = FOWLLOW_MODE.FOLLOWING
            if abs(rot) <= ANGLE_MARGIN:
                return
            # Turning
            self.turn_robot(rot)
            return

        # calc angle
        dist, angle, _ = self.get_relative_dist(
            self.lat, cur_lat, self.long, cur_long)
        if dist >= DIST_MIN:
            #print('Adjust - dist', dist, 'adjusted angle', angle)
            self.angle = angle
            self.lat = cur_lat
            self.long = cur_long

            rot = int(target_angle - self.angle)
            if (rot > 180):
                rot = rot - 360
            elif (rot < -180):
                rot = rot + 360

            if abs(rot) <= ANGLE_MARGIN:
                return
            # Turning
            self.turn_robot(rot)
            return

        # 500mm/s
        control_msg = self.convert_msg(1.0, 0.0)
        self.robot_pub.publish(control_msg)

    def turn_robot(self, angle):
        self.turning_angle = angle
        self.angle_counter = 0
        if (angle > 0):
            w = 0.5  # 50 * 0.5 = 25 degree /s => 1 degree / 40ms
        else:
            w = -0.5

        control_msg = self.convert_msg(0.0, w)
        self.robot_pub.publish(control_msg)

    def get_relative_dist(self, lat1, lat2, long1, long2):
        R_KM = 6371.0

        x1 = R_KM * long1 * math.cos((lat1 + lat2) / 2)
        y1 = R_KM * lat1
        x2 = R_KM * long2 * math.cos((lat1 + lat2) / 2)
        y2 = R_KM * lat2

        dx = x2 - x1
        dy = y2 - y1

        test_angle = math.atan2(dy, dx) * 180.0 / math.pi
        test_dist = math.sqrt(dx * dx + dy * dy)
        act_angle = round(test_angle - self.angle)

        if (act_angle > 180):
            act_angle = act_angle - 360
        elif (act_angle < -180):
            act_angle = act_angle + 360

        y_difference = lat2 - lat1
        x_difference = long2 - long1
        y_distance = math.radians(y_difference) * R_KM * 1000
        x_distance = math.radians(x_difference) * R_KM * math.cos(lat1) * 1000

        angle = math.atan2(y_distance, x_distance)
        angle = math.degrees(angle)
        dist = math.sqrt(y_distance*y_distance + x_distance*x_distance)

        rot = int(angle - self.angle)
        if (rot > 180):
            rot = rot - 360
        elif (rot < -180):
            rot = rot + 360

        return test_dist, test_angle, rot
        # x_distance = x_difference * 111.320 * (math.cos(y_difference * math.pi / 180)) *1000

    def lidar_cb(self, msg):
        # print(msg)
        self.lidar.scan_update(msg.angle_min, msg.angle_max,
                               msg.angle_increment, msg.ranges)
        # print('lidar', self.lidar.get_distance(0))
        # print('lidar', self.lidar.get_min_range())

        if (self.drive_mode == DRIVE_MODE.MANUAL):
            return

        # Given the lidar data, check if there is an obstacle in front of the robot
        # and stop the robot if there is one
        if (self.drive_mode == DRIVE_MODE.AUTO and self.trigger):
            min_range = self.lidar.get_min_range()
            if (min_range < LIDAR_STOP_DISTANCE):
                print('Obstacle detected')
                control_msg = self.convert_msg(0.0, 0.0)
                self.robot_pub.publish(control_msg)
                return

    def joy_cb(self, msg):
        # print(msg)
        if (msg.buttons[1]):  # B Circle
            print('Button 1 - manual', 'trigger ', self.trigger)
            self.drive_mode = DRIVE_MODE.MANUAL
        elif (msg.buttons[2]):  # Square
            print('Button 2 - auto', 'trigger ', self.trigger)
            self.drive_mode = DRIVE_MODE.AUTO

        elif (msg.buttons[3]):
            print('Button 3')
            self.angle = 0
            self.angle_counter = -1
            self.long = 0
            self.lat = 0

        elif (msg.buttons[4]):
            print('Button 4')
        elif (msg.buttons[5]):
            print('Button 5')
        elif (msg.buttons[6]):
            print('Button 6')
        elif (msg.buttons[7]):
            print('Button 7')
        elif (msg.buttons[11]):
            #print('Button 11 - up - trigger', self.trigger)
            control_msg = self.convert_msg(1.0, 0.0)
            self.robot_pub.publish(control_msg)
        elif (msg.buttons[12]):
            #print('Button 12 - down - trigger', self.trigger)
            control_msg = self.convert_msg(-1.0, 0.0)
            self.robot_pub.publish(control_msg)
        elif (msg.buttons[13]):
            if (self.angle_counter >= 0):
                return
            #print('Button 13 - LEFT- trigger', self.trigger)
            self.turn_robot(10)
        elif (msg.buttons[14]):
            if (self.angle_counter >= 0):
                return
            print('Button 14 - RIGHT- trigger', self.trigger)
            self.turn_robot(-10)

        if (msg.axes[5] < 0):
            self.trigger = True
        else:
            self.trigger = False
            self.is_start = False
            if (self.linear != 0.0 or self.angualr != 0.0):
                control_msg = self.convert_msg(0.0, 0.0)
                self.robot_pub.publish(control_msg)
            self.angle_counter = -1

    def twist_cb(self, msg):
        if (self.drive_mode == DRIVE_MODE.MANUAL):
            self.robot_pub.publish(msg)

    def timer_cb(self):
        if (self.angle_counter < 0):
            return

        # update current angle
        if (self.turning_angle < 0):
            if (self.angle <= -179):
                self.angle = 180
            else:
                self.angle = self.angle - 1  # self.angle_counter
        else:
            if (self.angle >= 180):
                self.angle = -179
            else:
                self.angle = self.angle + 1  # self.angle_counter

        if (self.angle_counter >= abs(self.turning_angle)):
            # stop
            control_msg = self.convert_msg(0.0, 0.0)
            print("Angle counter: ", self.angle_counter)
            self.robot_pub.publish(control_msg)
            self.turning_angle = 0
            self.angle_counter = -1
            print('Turn Finish - current angle: ', self.angle)
            return

        self.angle_counter += 1

    def heartbeat_timer_cb(self):
        heartbeat_msg = Int32()
        if (self.trigger):
            heartbeat_msg.data = 1
        else:
            heartbeat_msg.data = 0
        self.heartbeat_pub.publish(heartbeat_msg)

    def convert_msg(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.linear = linear
        self.angualr = angular
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
