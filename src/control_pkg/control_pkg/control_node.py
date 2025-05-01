#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from gps_msgs.msg import GPSFix
#from interfaces.msg import Gpsx
from gpsx.msg import Gpsx
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from enum import Enum
import math


WAY_POINTS = [(-31.980327, 115.817317), (-31.980554, 115.817743), (-31.980368, 115.818476)]

class DRIVE_MODE(Enum):
    NONE = 1000
    AUTO = 1001
    MANUAL = 1002
    PENDING = 1003
    
class FOWLLOW_MODE(Enum):
    NONE = 3000
    TURNING = 3001
    STRAIGHT = 3002
 
class ControlNode(Node): 
    def __init__(self):
        super().__init__("Control_Node")
        
        # Subscribers 
        self.create_subscription(Gpsx, "gpsx", self.gps_callback, 10)
        self.create_subscription(Joy, "joy", self.joy_cb, 10)
        self.create_subscription(Twist, "cmd_vel", self.twist_cb, 10)
        
        
        # Publisher
        self.robot_pub = self.create_publisher(Twist, 'cmd_vel_team10', 10)
        
        # Timer
        self.create_timer(100, self.timer_cb)
        
        # Variables
        self.lat = 0
        self.long = 0
        self.angle = 0
        
        self.current_point = 0
        
        self.drive_mode = DRIVE_MODE.MANUAL
        self.following_mode = FOWLLOW_MODE.NONE
        self.trigger = False
        
        self.linear = 0.0
        self.angualr = 0.0
        
                
    def gps_callback(self, msg):
        #print(msg)
        self.lat = msg.latitude
        self.long = msg.latitude
        
    
    def joy_cb(self, msg):
        #print(msg)
        if (msg.buttons[1]): # B Circle
            print('Button 1')
            self.drive_mode = DRIVE_MODE.MANUAL
        elif (msg.buttons[2]): # X Square
            print('Button 2')
            self.drive_mode = DRIVE_MODE.AUTO
        elif (msg.buttons[3]):
            print('Button 3')
        elif (msg.button[4]):
            print('Button 4')
            
        if (msg.axes[5] < 0):
            self.trigger = True
        else:
            self.trigger = False
            control_msg = self.convert_msg(0.0, 0.0)
            self.robot_pub.publish(control_msg)
            
    def twist_cb(self, msg):
        if (self.drive_mode == DRIVE_MODE.MANUAL):
            self.robot_pub.publish(msg)
        
            
    def timer_cb(self):
        if self.trigger:
            if (self.drive_mode == DRIVE_MODE.PENDING):
                self.drive_mode = DRIVE_MODE.AUTO
                # Resume driving
            if (self.drive_mode == DRIVE_MODE.AUTO):
                print('Keep Driving')
                
        else:
            if (self.drive_mode == DRIVE_MODE.AUTO):
                self.drive_mode = DRIVE_MODE.PENDING
                print('Stop Driving')
                # Stop Driving

    def convert_msg(self, linear:float, angular:float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        return msg
        
    def get_relative_dist(self):
        target_lat = WAY_POINTS[self.current_point][0]
        target_long = WAY_POINTS[self.current_point][1]
        
        y_difference = target_lat - self.lat
        x_difference = target_long - self.long
        y_distance = math.radians(y_difference) * R_KM * 1000
        x_distance = math.radians(x_difference) * R_KM * math.cos(self.lat) * 1000
        return x_distance, y_distance
        #x_distance = x_difference * 111.320 * (math.cos(y_difference * math.pi / 180)) *1000
        
R_KM = 6371.0
 
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
