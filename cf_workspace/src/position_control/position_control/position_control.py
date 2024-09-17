import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
import numpy as np
# from pyquaternion import Quaternion
import threading
from threading import Event
import sys
import time
import logging

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

GOAL = [0,0]

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')
DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()
global position_estimate
global initial_position
global start_time
global run_time
run_time = 20 #s
start_time = 0
initial_position = [0,0,0]
position_estimate = "start"

logging.basicConfig(level=logging.ERROR)
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
        # self.arrow_length = 1.0  # Length of the arrow
        # self.fig, self.ax = plt.subplots()
        # self.arrow = self.ax.quiver(self.x[-1], self.y[-1], np.cos(self.theta), np.sin(self.theta), angles='xy', scale_units='xy', scale=1)
        # self.ax.set_xlim(-20, 20)
        # self.ax.set_ylim(-20, 20)
        # self.train_line, = self.ax.plot([], [], 'r-', lw=2,alpha = 0.8)
        # plt.ion()
        # plt.show()
    def listener_callback(self, msg):
        global position_estimate
        # Update the coordinates
        # print(msg.transforms[1].transform.translation.x)
        self.x.append(msg.transforms[0].transform.translation.x)
        self.y.append(msg.transforms[0].transform.translation.y)
        # q_theta = Quaternion(np.array([msg.transforms[0].transform.rotation.w, msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z]))
        self.theta = 2*np.arccos(msg.transforms[0].transform.rotation.w)
        
        position_estimate = [self.x,self.y,self.theta]
        print("Position of the crazyflie ground frame is : {}".format(position_estimate))
        if position_estimate == "start":
            initial_position = position_estimate
        
        # self.update_plot()
    # def update_plot(self):
    #     # Update the scatter plot with new coordinates
        
    #     self.arrow.set_offsets([self.x[-1], self.y[-1]])
    #     self.arrow.set_UVC(np.cos(self.theta), np.sin(self.theta)) 
    #     # print(np.rad2deg(self.theta))
    #     self.train_line.set_data(self.x, self.y)  # Update the train line
    #     plt.draw()
    #     plt.pause(0.00001)  # Pause to allow the plot to update


def optitrack(args=None):
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
        # plt.ioff()  # Turn off interactive mode
        # plt.show()  # Show plot one last tirun_timeme

def cf_control():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        position_feedback_control(scf)

def position_feedback_control(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Connected to crazyflie : {}".format(URI))
        mc.up(0.5)
        time.sleep(4)
        while(time.time() - start_time < run_time):
            global position_estimate
            time.sleep(3)
            if position_estimate == "start":
                continue
            v = 0.5
            delta_g = np.array(GOAL)  - np.array(position_estimate)[0:2] 
            theta = position_estimate[2]
            R_theta = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]).T
            delta = np.matmul(R_theta,delta_g.T)
            delta = delta/np.linalg.norm(delta)
            mc.start_linear_motion(0.1*delta[0],0.1*delta[1],0)
            print("[At time {} s] : Moving drone towards : {} in drone frame and {} in optitrack frame".format(round(time.time() - start_time,1),delta,delta_g))
        mc.stop()

def param_deck_flow(_, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')



def main(args=None):
    global start_time
    start_time = time.time()
    # Creating threads
    thread1 = threading.Thread(target=optitrack)
    thread2 = threading.Thread(target=cf_control)

    # Starting threads
    thread1.start()
    thread2.start()

    # Joining threads to the main thread
    thread1.join()
    thread2.join()
if __name__ == '__main__':
    main()
