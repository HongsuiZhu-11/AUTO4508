#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from gps_msgs.msg import GPSFix
from control_pkg.msg import Gpsx


 
 
class ControlNode(Node): 
    def __init__(self):
        super().__init__("Control_Node")
        
        # Subscribers 
        self.create_subscription(gpsx, "/sbg/gps_pos", self.gps_fix_callback, 10)
        
                
        
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
