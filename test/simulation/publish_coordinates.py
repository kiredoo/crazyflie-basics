import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Float64
global val
val = 0
class ConstantCoordinatePublisher(Node):
    def __init__(self):
        super().__init__('constant_coordinate_publisher')
        self.publisher_ = self.create_publisher(Float64, 'omega_control', 10)
        self.timer = self.create_timer(1.0, self.publish_coordinates)  # 1 Hz

    def publish_coordinates(self):
        msg = Float64()
        global val
        val +=0.2
        if val >= 1:
            val = -1.0
        msg.data = val
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: angular velocity ={}'.format(msg.data))

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
