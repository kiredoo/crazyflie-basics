import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Float64

class ConstantCoordinatePublisher(Node):
    def __init__(self):
        super().__init__('constant_coordinate_publisher')
        self.publisher_ = self.create_publisher(Float64, 'input', 10)
        self.timer = self.create_timer(0.1, self.publish_coordinates)  # 10 Hz

    def publish_coordinates(self):
        msg = Float64()
        msg.data = 0.1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing theta dot: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ConstantCoordinatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
