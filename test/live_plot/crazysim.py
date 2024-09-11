import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np

class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot_node')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'omega_control',
            self.listener_callback,
            20
        )
        self.subscription  # prevent unused variable warning

        # Initialize the plot
        self.x = 0.0
        self.y = 0.0
        self.fig, self.ax = plt.subplots()
        self.sc = self.ax.scatter(self.x, self.y)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        # Update the coordinates
        self.x, self.y = msg.data
        self.get_logger().info(f'Received coordinates: x={self.x}, y={self.y}')
        self.update_plot()

    def update_plot(self):
        # Update the scatter plot with new coordinates
        self.sc.set_offsets(np.c_[[self.x], [self.y]])
        plt.draw()
        plt.pause(0.1)  # Pause to allow the plot to update

def main(args=None):
    rclpy.init(args=args)
    live_plot_node = LivePlotNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(live_plot_node)
            # This will give control back to the event loop, allowing for plot updates
    except KeyboardInterrupt:
        pass
    finally:
        live_plot_node.destroy_node()
        rclpy.shutdown()
        plt.ioff()  # Turn off interactive mode
        plt.show()  # Show plot one last time

if __name__ == '__main__':
    main()
