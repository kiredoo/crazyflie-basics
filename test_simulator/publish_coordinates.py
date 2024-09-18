import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ConstantCoordinatePublisher(Node):
    def __init__(self):
        super().__init__('constant_coordinate_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'coordinates', 10)
        self.timer = self.create_timer(1.0, self.publish_coordinates)  # 1 Hz

    def publish_coordinates(self):
        msg = Float64MultiArray()
        msg.data = [0.0 + 5*random.random(), 0.0 + 5*random.random()]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: x={}, y={}'.format(msg.data[0],msg.data[1]))

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
