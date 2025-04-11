import rclpy
from . import genericJoystick
from .crazyflie import CrazyflieServer, TimeHelper
from rclpy.signals import SignalHandlerOptions

class Crazyswarm:

    def __init__(self):
        # rclpy.init(args=None)
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)

        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)

        self.input = genericJoystick.Joystick(self.timeHelper)
