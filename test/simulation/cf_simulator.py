import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np


#velocity of crazyflie
v = 1

#discrete time step of the simulator
dt = 0.1

#Dynamics of the crazyflie - unicycle
def unicycle(state,omega):
    return np.array([v*np.cos(state[2]),v*np.sin(state[2]),omega])

#RK4 solver of the crazyflie
def rk4_step(state,omega):
    k1 = unicycle(state,omega)
    k2 = unicycle(state + (0.5*k1*dt),omega)
    k3 = unicycle(state + (0.5*k2*dt),omega)
    k4 = unicycle(state + (k3*dt),omega)
    return dt*((k1 + (2*k2) + (2*k3) + k4)/6)


class cf_sim(Node):
    def __init__(self):
        super().__init__('cf_sim')
        self.subscription = self.create_subscription(
            Float64,
            'omega_control',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        #LIVE PLOT
        # Store the history of positions to form the train
        self.position_history = []  # To store (x, y) positions
        self.train_length = 50  # Number of points in the train

        # Initialize the plot
        self.omega = 0.0  # Start with angular velocity = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.arrow_length = 1.0  # Length of the arrow
        self.fig, self.ax = plt.subplots()
        self.arrow = self.ax.quiver(self.x, self.y, np.cos(self.theta), np.sin(self.theta), angles='xy', scale_units='xy', scale=1)
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.ax.grid(True)
        self.t = 0
        # Add a text object to display the x value, initially set to 0
        self.x_text = self.ax.text(0.5, -0.1, f't: {self.t:.2f}s', transform=self.ax.transAxes, fontsize=12, color='green', verticalalignment='top')

        self.train_line, = self.ax.plot([], [], 'r-', lw=2,alpha = 0.8)  # Initialize the train as a red line
        plt.ion()
        plt.show()

        # FASTER PLOT UPDATE RATE IN LINE WITH THE TIME STEP dt
        # Timer to update the plot at a higher rate
        self.timer_period = dt  # Update plot every dt seconds
        self.timer = self.create_timer(self.timer_period, self.update_plot)

    def listener_callback(self, msg):
        # Update the angular velocity when a new message is received
        self.omega = msg.data
        # self.get_logger().info(f'Received angular velocity in rad/s : {self.omega}')

    def update_plot(self):
        self.t += dt
        # Update the coordinates based on the angular velocity
        ds = rk4_step([self.x,self.y,self.theta],self.omega)
        self.x = self.x + ds[0]  # Simulate movement based on angular velocity
        self.y = self.y + ds[1]
        self.theta = self.theta + ds[2]
        # print(self.theta)
        # Update the arrow direction and position
        self.arrow.set_offsets([self.x, self.y])
        self.arrow.set_UVC(np.cos(self.theta), np.sin(self.theta))  # Set direction of arrow
        # Add the current position to the history for the train
        self.position_history.append((self.x, self.y))
        # Limit the length of the train by keeping only the last `train_length` positions
        if len(self.position_history) > self.train_length:
            self.position_history.pop(0)
        # Update the displayed value of x on the side of the plot
        self.x_text.set_text(f't: {self.t:.2f}s')  # Update text to show current x value

        # Update the train plot (as a line connecting the previous positions)
        history_x, history_y = zip(*self.position_history)  # Separate x and y coordinates
        self.train_line.set_data(history_x, history_y)  # Update the train line
        self.ax.set_xlim(self.x - 20, self.x + 20)  # Center x-axis around the point
        self.ax.set_ylim(self.y - 20, self.y + 20)
        plt.draw()
        plt.pause(0.001)  # Pause to allow the plot to update

def main(args=None):
    rclpy.init(args=args)
    cf_sim_node = cf_sim()

    try:
        while rclpy.ok():
            rclpy.spin_once(cf_sim_node, timeout_sec=0.01)
            # The event loop spins and the timer callback updates the plot regularly
    except KeyboardInterrupt:
        pass
    finally:
        cf_sim_node.destroy_node()
        rclpy.shutdown()
        plt.ioff()  # Turn off interactive mode
        plt.show()  # Show plot one last time

if __name__ == '__main__':
    main()
