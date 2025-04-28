import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import cv2
#import depthai as dai

TOPIC = 'rgb/preview/image_raw'

class CameraNode(Node): 
    def __init__(self):
        super().__init__("Camera_Node")
        print('Create Camera Node')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()