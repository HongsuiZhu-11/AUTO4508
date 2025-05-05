import rclpy
from rclpy.node import Node
from vision_interface.srv import DetectTarget

class Client(Node):
    def __init__(self):
        super().__init__('detect_target_client')
        self.cli = self.create_client(DetectTarget, 'detect_target')
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Waiting for detect_target service...')
        self.req = DetectTarget.Request()

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = Client()
    response = client.send_request()
    if response.success:
        print(f"[OK] Target: {response.label}")
        print(f"Distance: {response.distance:.2f} mm")
        print(f"Coords: X={response.world_x:.2f}, Y={response.world_y:.2f}, Z={response.world_z:.2f}")
        print(f"Saved Image: {response.image_path}")
    else:
        print("[FAIL] No target detected.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
