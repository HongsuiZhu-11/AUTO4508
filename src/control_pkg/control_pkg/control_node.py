#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from gps_msgs.msg import GPSFix
#from interfaces.msg import Gpsx
from gpsx.msg import Gpsx
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from enum import Enum
import math


WAY_POINTS = [(-31.980327, 115.817317), (-31.980554, 115.817743), (-31.980368, 115.818476)]

DIST_MIN = 1
DEST_MARGIN = 0.5
ANGLE_MARGIN = 5

class DRIVE_MODE(Enum):
    NONE = 1000
    AUTO = 1001
    MANUAL = 1002
    PENDING = 1003
    
class FOWLLOW_MODE(Enum):
    NONE = 3000
    FOLLOWING = 3001
    FINDING = 3002
 
class ControlNode(Node): 
    def __init__(self):
        super().__init__("Control_Node")
        
        # Subscribers 
        self.create_subscription(Gpsx, "gpsx", self.gps_callback, 10)
        self.create_subscription(Joy, "joy", self.joy_cb, 10)
        self.create_subscription(Twist, "cmd_vel", self.twist_cb, 10)
        
        
        # Publisher
        self.robot_pub = self.create_publisher(Twist, 'cmd_vel_team10', 10)
        self.heartbeat_pub = self.create_publisher(Int32, 'heartbeat_team10', 10)
        
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
        
                
    def gps_callback(self, msg):
        #print(msg)
        cur_lat = msg.latitude
        cur_long = msg.longitude
        
        if (type(cur_lat) == str or int(cur_lat) != -31):
            
            return
        if self.lat == 0 or self.long == 0:
            print('set current gps')
            self.lat = cur_lat
            self.long = cur_long
            return

        if self.drive_mode != DRIVE_MODE.AUTO:
            #print('not Driving', self.drive_mode)
            return
        if not self.trigger:
            #print('not trigger', self.trigger)
            return

        if self.angle_counter >=0:
            # on Turning
            #print("on Turning", self_counter)
            return
        
        target_lat = WAY_POINTS[self.current_point][0]
        target_long = WAY_POINTS[self.current_point][1]
        tart_dist, target_angle = self.get_relative_dist(cur_lat, target_lat, cur_long, target_long)
        print('tart_dist',tart_dist, 'target_angle',target_angle)
        # check goal
        if tart_dist <= DEST_MARGIN:
            print('Arrive: ', self.current_point)
            control_msg = self.convert_msg(0.0, 0.0)
            self.robot_pub.publish(control_msg)
            self.is_start = False
            # TODO Find Optical
            #self.following_mode = FOWLLOW_MODE.FINDING
            self.current_point += 1
            return
             

        # calc angle
        dist, angle = self.get_relative_dist(self.lat, cur_lat, self.long, cur_long)

        print('dist', dist, 'angle', angle)
        if not self.is_start and self.following_mode != FOWLLOW_MODE.FINDING:
            self.is_start = True
            self.following_mode = FOWLLOW_MODE.FOLLOWING
            if abs(target_angle - self.angle) < ANGLE_MARGIN:
                return
            # Turning
            self.turn_robot(target_angle)
            return
        
        if dist >= DIST_MIN:
            self.angle = angle
            self.lat = cur_lat
            self.long = cur_long
            
            rot = int(target_angle - self.angle)
            if (rot > 180):
                rot = rot - 360
            elif (rot < -180):
                rot = rot + 360
                    
            if abs(rot) < ANGLE_MARGIN:
                return
            # Turning
            self.turn_robot(rot)
            return
        
        # 250mm/s 
        control_msg = self.convert_msg(0.5, 0.0)
        self.robot_pub.publish(control_msg)

    def turn_robot(self, angle):
        self.turning_angle = angle
        self.angle_counter = 0
        if (angle > 0):
            w = 0.5 # 50 * 0.5 = 25 degree /s => 1 degree / 40ms
        else:
            w = -0.5
        
        control_msg = self.convert_msg(0.0, w)
        self.robot_pub.publish(control_msg)

    def get_relative_dist(self, lat1, lat2, long1, long2):
        R_KM = 6371.0
        y_difference = lat2 - lat1
        x_difference = long2 - long1
        y_distance = math.radians(y_difference) * R_KM * 1000
        x_distance = math.radians(x_difference) * R_KM * math.cos(self.lat) * 1000
        
        angle = math.atan2(y_distance, x_distance)
        angle = math.degrees(angle)
        dist = math.sqrt(y_distance*y_distance + x_distance*x_distance)
        return  dist, angle
        #x_distance = x_difference * 111.320 * (math.cos(y_difference * math.pi / 180)) *1000

    
    def joy_cb(self, msg):
        #print(msg)
        if (msg.buttons[1]): # B Circle
            print('Button 1')
            self.drive_mode = DRIVE_MODE.MANUAL
        elif (msg.buttons[2]): # Square
            print('Button 2 - auto', 'trigger ', self.trigger)
            self.drive_mode = DRIVE_MODE.AUTO
            
        elif (msg.buttons[3]):
            print('Button 3')
            self.angle = 0
            self.angle_counter = -1
            
            
        elif (msg.buttons[4]):
            print('Button 4')
        elif (msg.buttons[11]):
            print('Button 11 - up')
            control_msg = self.convert_msg(0.5, 0.0)
            self.robot_pub.publish(control_msg)
        elif (msg.buttons[13]):
            if (self.angle_counter >= 0):
                return
            print('Button 13 - LEFT')
            self.turn_robot(10)
        elif (msg.buttons[14]):
            if (self.angle_counter >= 0):
                return
            print('Button 14 - RIGHT')
            self.turn_robot(-10)
        
        
        if (self.drive_mode == DRIVE_MODE.MANUAL):
            return
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
            self.angle = self.angle - self.angle_counter
        else:
            self.angle = self.angle + self.angle_counter
        
        if (self.angle_counter >= abs(self.turning_angle)):
            # stop
            control_msg = self.convert_msg(0.0, 0.0)
            self.robot_pub.publish(control_msg)
            self.turning_angle = 0
            self.angle_counter = -1
            print('Turn Finish')
            return
        
        self.angle_counter += 1

    def heartbeat_timer_cb(self):
        heartbeat_msg = Int32()
        heartbeat_msg.data = 1
        self.heartbeat_pub.publish(heartbeat_msg)

    def convert_msg(self, linear:float, angular:float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.linear = linear
        self.angualr = angular
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
