import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np


vo = 1
dt = 0.01


def f_unicycle(t, xs, u):
    theta_dot_t = u
    return np.array([vo * np.cos(xs[2]), vo * np.sin(xs[2]), theta_dot_t])

def rk4(f,x,t,dt,u):
    if not isinstance(x, np.ndarray):
        raise TypeError(f"x should be a NumPy array, but got {type(x)}")
    k1 = f(t,x,u)
    #print(k1)
    k2 = f(t + (0.5*dt),x + (0.5*k1*dt),u)
    #print(k2)
    k3 = f(t + (0.5*dt),x + (0.5*k2*dt),u)
    #print(k3)
    k4 = f(t + dt,x + (k3*dt),u)
    #print(k4)

    return dt*((k1 + (2*k2) + (2*k3) + k4)/6)

class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot_node')
        self.subscription = self.create_subscription(
            Float64,
            'input',self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize the plot
        self.x = [0.0]
        self.y = [0.0]
        self.fig, self.ax = plt.subplots()
        self.sc = self.ax.scatter(self.x, self.y)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        #plt.plot(self.x,self.y)
        self.xs = np.array([0.0,0.0,0.0])
        self.t = 0.0
        plt.ion()
        plt.show()
        # self.timer = self.create_timer(0.02, self.listener_callback)
        # don't change
    def listener_callback(self, msg):
        # Update the coordinates
        print('simulation running')
        self.u = float(msg.data)
        self.get_logger().info(f'Received input, u = {self.u}')
        print(f"Before rk4: self.xs = {self.xs}, type = {type(self.xs)}")
        dx = rk4(f_unicycle,self.xs,self.t,dt,self.u)
        self.xs += dx
        self.t += dt
        print(f"After rk4: self.xs = {self.xs}, type = {type(self.xs)}")
        self.update_plot()
        # publisher publishes theta dot
        # needs to be updated
    def update_plot(self):
        
        #Update the scatter plot with new coordinates
        #print(f'self.xs is {self.xs}')
        self.x.append(self.xs[0])
        self.y.append(self.xs[1])
        self.sc.set_offsets(np.c_[self.x, self.y])
        # print(f"self.x is {self.x}")
        # print(f"self.y is {self.y}")
        plt.draw()
        #plt.show()
        plt.pause(0.05)  # Pause to allow the plot to update
        """
        i/o - self, msg
        msg.u = theta dot

        """
    
    

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


