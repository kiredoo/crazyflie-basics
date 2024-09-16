import rclpy
from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
import numpy as np

class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot_node')
        self.subscription = self.create_subscription(
            TFMessage,
            'tf',
            self.listener_callback,
            20
        )
        self.subscription  # prevent unused variable warning
        self.x = [0]
        self.y = [0]
        self.theta = 0
        self.arrow_length = 1.0  # Length of the arrow
        self.fig, self.ax = plt.subplots()
        self.arrow = self.ax.quiver(self.x[-1], self.y[-1], np.cos(self.theta), np.sin(self.theta), angles='xy', scale_units='xy', scale=1)
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.train_line, = self.ax.plot([], [], 'r-', lw=2,alpha = 0.8)
        plt.ion()
        plt.show()
    def listener_callback(self, msg):
        # Update the coordinates
        # print(msg.transforms[1].transform.translation.x)
        self.x.append(msg.transforms[0].transform.translation.x)
        self.y.append(msg.transforms[0].transform.translation.y)
        self.theta = 2*np.arccos(msg.transforms[0].transform.rotation.w)
        self.update_plot()
    def update_plot(self):
        # Update the scatter plot with new coordinates
        
        self.arrow.set_offsets([self.x[-1], self.y[-1]])
        self.arrow.set_UVC(np.cos(self.theta), np.sin(self.theta)) 
        # print(np.rad2deg(self.theta))
        self.train_line.set_data(self.x, self.y)  # Update the train line
        plt.draw()
        plt.pause(0.00001)  # Pause to allow the plot to update

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
