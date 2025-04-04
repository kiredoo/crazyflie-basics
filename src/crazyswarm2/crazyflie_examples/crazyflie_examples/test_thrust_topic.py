"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm
import numpy as 

# import rclpy
# from geometry_msgs.msg import Twist

# TAKEOFF_DURATION = 2.5
# HOVER_DURATION = 5.0
# NAME = 'cf231'


# def main():
#     swarm = Crazyswarm()
#     timeHelper = swarm.timeHelper
#     cf = swarm.allcfs.crazyflies[0]
#     pub = rclpy.Publisher(f'/{NAME}/cmd_vel_legacy',Twist,queue_size=1)
    

#     for _ in range(300):
#         roll=0.0
#         pitch = 0.0
#         yaw = 0.0
#         thrust = 0.0
#         msg = Twist()
#         msg.linear.x = pitch
#         msg.linear.y = roll
#         msg.linear.z = thrust
#         msg.angular.z = yaw
#         pub.publish(msg)
#         timeHelper.sleep(1/100)




#     # cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
#     # timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
#     # cf.land(targetHeight=0.04, duration=2.5)
#     # timeHelper.sleep(TAKEOFF_DURATION)


# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep

MSG_BROADCAST_DURATION = 1
FREQ = 10


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        # Create a publisher that publishes to the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cf231/cmd_vel_legacy', 10)
        # Create a Twist message to hold the velocity commands
        self.msg = Twist()

        swarm = Crazyswarm()
        # timeHelper = swarm.timeHelper
        self.cf = swarm.allcfs.crazyflies[0]
        
    def move(self):
        # Set linear and angular velocities
        self.msg.linear.x = 0.5  # Move forward at 0.5 m/s
        self.msg.angular.z = 0.2  # Rotate at 0.2 rad/s
        roll=0.0
        pitch = 0.0
        yaw = 0.0
        thrust = 0.0
        msg = Twist()
        self.msg.linear.x = pitch
        self.msg.linear.y = roll
        self.msg.linear.z = thrust
        self.msg.angular.z = yaw
        self.publisher_.publish(self.msg)


        for _ in range(MSG_BROADCAST_DURATION*FREQ):
            # self.get_logger().info('Publishing velocity command...')
            self.publisher_.publish(self.msg)

            # Publish for 1 second
            sleep(1/FREQ)

        # After 1 second, stop the robot by setting velocities to 0
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

        self.get_logger().info('Stopping robot...')
        self.publisher_.publish(self.msg)

    def test_thrust(self):
        # Create an array that goes from 0 to 100 with 100 points
        thrust_cmd = np.concatenate((np.linspace(0, 100, 100),np.linspace(100, 0, 100)))
        


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    
    try:
        # Move the robot
        node.move()
    finally:
        # Shut down the node after execution
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

