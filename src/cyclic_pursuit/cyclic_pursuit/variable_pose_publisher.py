import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time

class GradualPosePublisher(Node):
    def __init__(self):
        super().__init__('gradualPosePublisher')
        self.publisher231_ = self.create_publisher(Pose, '/cf231/pose', 10)
        self.publisher123_ = self.create_publisher(Pose, '/cf123/pose', 10)
        self.timer231 = self.create_timer(0.01, self.publish_pose1)  # Publish at 100 Hz
        self.timer123 = self.create_timer(0.01, self.publish_pose2)
        self.pose231 = Pose()
        self.pose123 = Pose()
        
        # Initial position and movement speed per second
        self.pose231.position.x = 4.0
        self.pose231.position.y = 6.0
        self.pose231.position.z = 3.0
        self.pose231.orientation.x = 0.0
        self.pose231.orientation.y = 0.0
        self.pose231.orientation.z = 3.0
        self.pose231.orientation.w = 1.0

        self.pose123.position.x = 5.0
        self.pose123.position.y = 6.0
        self.pose123.position.z = 3.0
        self.pose123.orientation.x = 0.0
        self.pose123.orientation.y = 0.0
        self.pose123.orientation.z = 3.0
        self.pose123.orientation.w = 1.0
        self.start_time = time.time()

    def publish_pose1(self):
        # Calculate time elapsed since the node started
        elapsed_time = time.time() - self.start_time
        
        # Update position gradually over time
        self.pose231.position.x = elapsed_time * 0.1  # Moves 0.1 units per second along x
        self.pose231.position.y = elapsed_time * 0.05  # Moves 0.05 units per second along y
        self.pose231.position.z = 0.0  # Keep z constant, or adjust as needed
        self.pose231.orientation.z = 0.0
        # Publish the updated pose
        self.publisher231_.publish(self.pose231)  
    
    def publish_pose2(self):
        # Calculate time elapsed since the node started
        elapsed_time = time.time() - self.start_time
        
        # Update position gradually over time
        self.pose123.position.x = elapsed_time * 0.1  # Moves 0.1 units per second along x
        self.pose123.position.y = elapsed_time * 0.05  # Moves 0.05 units per second along y
        self.pose123.position.z = 0.0  # Keep z constant, or adjust as needed
        self.pose123.orientation.z = 0.0
        # Publish the updated pose
        self.publisher123_.publish(self.pose123)  


def main(args=None):
    rclpy.init(args=args)
    gradual_pose_publisher = GradualPosePublisher()
    rclpy.spin(gradual_pose_publisher)

    # Shutdown when done
    gradual_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()