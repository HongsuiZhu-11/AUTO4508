import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Float32MultiArray
#include <std_msgs/msg/float32_multi_array.hpp>

# Path to your virtual environment
venv_path = "/home/team10/ws/AUTO4508/venv"
#venv_path = "/home/shawn/ws/AUTO4508/venv"

# Manually set environment variables to activate the venv
os.environ["VIRTUAL_ENV"] = venv_path
os.environ["PATH"] = f"{venv_path}/bin:" + os.environ["PATH"]

# Add the virtual environment's site-packages to sys.path
site_packages = os.path.join(venv_path, "lib", "python3.12", "site-packages")  # Adjust Python version
sys.path.insert(0, site_packages)
# import files
from Phidget22.Devices.Spatial import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *

class ImuNode(Node): 
    def __init__(self):
        super().__init__("IMU_Node")
        self.spatial = Spatial()
        self.spatial.open()
        self.create_timer(0.25, self.timer_cb)
        self.spatial_counter = 0
        
        self.imu_pub = self.create_publisher(Float32MultiArray, 'imu_team10', 10)
        
    def timer_cb(self):
        if (self.spatial_counter < 0):
            return
        
        if (self.spatial_counter > 20): # wait 5s
            print('error open to spatial')
            self.spatial_counter = -1
            return
        
        if (not self.spatial.getIsOpen()):
            self.spatial_counter += 1
            return
        
        self.spatial_counter = 0
        
        angles = self.spatial.getEulerAngles()
        angles_msg = Float32MultiArray()
        angles_msg.data = [angles.pitch, angles.roll, angles.heading]
        self.imu_pub.publish(angles_msg)
        #print('pitch:', angles.pitch, 'roll:', angles.roll, 'heading:', angles.heading)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = ImuNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()