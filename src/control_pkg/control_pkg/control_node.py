#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from gps_msgs.msg import GPSFix
from interfaces.msg import Gpsx
from sensor_msgs.msg import Joy
from enum import Enum

class DRIVE_MODE(Enum):
    NONE = 1000
    AUTO = 1001
    MANUAL = 1002

 
class ControlNode(Node): 
    def __init__(self):
        super().__init__("Control_Node")
        
        # Subscribers 
        self.create_subscription(GPSFix, "gpsx", self.gps_callback, 10)
        self.create_subscription(Joy, "joy", self.joy_cb, 10)
        
        # Variables
        self.lat = 0
        self.long = 0
        
        self.drive_mode = DRIVE_MODE.MANUAL
        self.trigger = False
        
                
    def gps_callback(self, msg):
        #print(msg)
        self.lat = msg.latitude
        self.long = msg.latitude
        
    
    def joy_cb(self, msg):
        print(msg)
        if (msg.buttons[1]): # B Circle
            self.drive_mode = DRIVE_MODE.MANUAL
        elif (msg.buttons[2]): # X Square
            self.drive_mode = DRIVE_MODE.AUTO
            
        if (msg.axis[5] < 0):
            self.trigger = True
        else:
            self.trigger = False
        
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
