import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np

v = 1
dt = 0.1

def unicycle(state,omega):
    return np.array([v*np.cos(state[2]),v*np.sin(state[2]),omega])
def rk4_step(state,omega):
    k1 = unicycle(state,omega)
    k2 = unicycle(state + (0.5*k1*dt),omega)
    k3 = unicycle(state + (0.5*k2*dt),omega)
    k4 = unicycle(state + (k3*dt),omega)

    return dt*((k1 + (2*k2) + (2*k3) + k4)/6)
class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot_node')
        self.subscription = self.create_subscription(
            Float64,
            'omega_control',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize the plot
        self.omega = 0.0  # Start with angular velocity = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.fig, self.ax = plt.subplots()
        self.sc = self.ax.scatter(self.x, self.y)
        self.ax.set_xlim(-10, 100)
        self.ax.set_ylim(-10, 100)
        plt.ion()
        plt.show()

        # Timer to update the plot at a higher rate
        self.timer_period = 0.1  # Update plot every 0.1 seconds
        self.timer = self.create_timer(self.timer_period, self.update_plot)

    def listener_callback(self, msg):
        # Update the angular velocity when a new message is received
        self.omega = msg.data
        self.get_logger().info(f'Received angular velocity in rad/s : {self.omega}')

    def update_plot(self):
        # Update the coordinates based on the angular velocity
        ds = rk4_step([self.x,self.y,self.theta],self.omega)
        self.x = self.x + ds[0]  # Simulate movement based on angular velocity
        self.y = self.y + ds[1]
        self.theta = self.theta + ds[2]
        self.sc.set_offsets(np.c_[[self.x], [self.y]])
        plt.draw()
        plt.pause(0.001)  # Pause to allow the plot to update

def main(args=None):
    rclpy.init(args=args)
    live_plot_node = LivePlotNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(live_plot_node, timeout_sec=0.01)
            # The event loop spins and the timer callback updates the plot regularly
    except KeyboardInterrupt:
        pass
    finally:
        live_plot_node.destroy_node()
        rclpy.shutdown()
        plt.ioff()  # Turn off interactive mode
        plt.show()  # Show plot one last time

if __name__ == '__main__':
    main()
