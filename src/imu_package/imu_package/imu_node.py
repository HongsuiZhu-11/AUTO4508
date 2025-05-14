import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32

# Path to your virtual environment
venv_path = "/home/shawn/ws/AUTO4508/venv"

# Manually set environment variables to activate the venv
os.environ["VIRTUAL_ENV"] = venv_path
os.environ["PATH"] = f"{venv_path}/bin:" + os.environ["PATH"]

# Add the virtual environment's site-packages to sys.path
site_packages = os.path.join(venv_path, "lib", "python3.11", "site-packages")  # Adjust Python version
sys.path.insert(0, site_packages)
# import files
from Phidget22.Devices.Spatial import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *

class ImuNode(Node): 
    def __init__(self):
        super().__init__("IMU_Node")
        self.spatial = Spatial()
        self.create_timer(0.25, self.timer_cb)
        
    def timer_cb(self):
        angles = self.spatial.getEulerAngles()
        print('pitch:', angles.pitch, 'roll:', angles.roll, 'heading:', angles.heading)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = ImuNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()