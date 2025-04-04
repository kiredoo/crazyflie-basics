import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import rclpy.subscription
from std_msgs.msg import Float64
import numpy as np
from cyclic_pursuit.functions.utils import *
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import ParameterValue, ParameterType

qos_profile = QoSProfile(depth=20)
qos_profile.reliability = ReliabilityPolicy.RELIABLE
qos_profile.durability = DurabilityPolicy.VOLATILE




global v
global dt
global psi
global c
global v
global n
global k
global radius
global rho_o
global control_signal
global control_hold
global control_rate
global theta_dot_save
global gamma_save
global alpha_v
global dl
global ds 
global av
global bd_save
global pij_vec_3d
global pij_save
global vis_zone_vec2
global utj_vec
global gamma_ij_glob

########### PARAMS FOR N = 1 ############
# v = 0.3
# av = np.pi/4
# ds = 0.5
# dl = 1
# psi = np.pi * (2)
# d_0 = 1.285
# c = 2
# cj = 2
# k = 0.075
# kj = 0.15
# rho_o = 0.02
# n = 2
#########################################

########### PARAMS FOR N = 2 ############
# v = 0.3
# av = np.pi/4
# ds = 0.5
# dl = 1
# psi = np.pi * (2)
# d_0 = 1.285
# c = 2
# cj = 2
# k = 0.075
# kj = 0.15
# rho_o = 0.02
# n = 2
#########################################

#define params

# v = 0.3
# av = np.pi/4
# ds = 1
# dl = 4
# psi = np.pi * (7/4)
# d_0 = 2.83
# c = 2
# cj = 2
# k = 0.17
# kj = 0.31
# rho_o = 0.8
# n = 3

# turn these global variables into an xml file
# to run parameter servers

def yaw_from_quaternion(quaternion):
    return 2*np.arctan2(quaternion.z, quaternion.w)


class Crazyflie(Node):

    def __init__(self,name,id,cfs_names,v,av,ds,dl,psi,d_0,c,cj,k,kj,rho_o,n,height):
        super().__init__(name)
        
        self.v = v
        self.av = av
        self.ds = ds
        self.dl = dl
        self.psi = psi
        self.d_0 = d_0
        self.c = c
        self.cj = cj
        self.k = k
        self.kj = kj
        self.rho_o = rho_o
        self.n = n
        self.height = height
        self.name = name
        self.id = id
        self.pose = [0.0,0.0,0.0]
        self.other_cfs_names = cfs_names
        self.other_odom = {cf_name: [0.0,0.0,0.0] for cf_name in self.other_cfs_names if cf_name != name}
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            f'/{self.name}/pose',
            self.pose_sub,
            10
        )
        # self.pose_subscrition()
        self.create_subscription(PoseStamped,'/all_crazyflies/odom',self.odom_callback,qos_profile = qos_profile)
        self.odom_publish = self.create_publisher(PoseStamped, '/all_crazyflies/odom',10)
        self.cmd_pub = self.create_publisher(Twist,f'/{self.name}/cmd_vel_hover',10)
        self.cmd_msg = Twist()

        self.cmd_msg.linear.x = self.v
        self.cmd_msg.linear.z = self.height

        timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(timer_period, self.controller)
        # self.odom_ready = False
        self.get_logger().info(
                                f'params for cf: {self.id}: v: {self.v}, av: {self.av}, ds: {self.ds}, '
                                f'dl: {self.dl}, psi: {self.psi}, d_0: {self.d_0}, '
                                f'c: {self.c}, cj: {self.cj}, k: {self.k}, kj: {self.kj}, '
                                f'rho_o: {self.rho_o}, n: {self.n}, height: {self.height}'
)



    def pose_sub(self,msg):
        
        yaw = yaw_from_quaternion(msg.pose.orientation)

        self.pose = [msg.pose.position.x,msg.pose.position.y,yaw]
        # self.get_logger().info(f'Pose_sub for cf: {self.id}: {str(self.pose)}')
        self.publish_odom()
    
    def publish_odom(self):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.pose[0]
        pose_msg.pose.position.y = self.pose[1]
        pose_msg.pose.orientation.z = self.pose[2]
        pose_msg.header.frame_id = self.name
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_publish.publish(pose_msg)

    def odom_callback(self,msg):
        sender_name = msg.header.frame_id
        # self.get_logger().info(f'Msg: {msg.header.frame_id} and other_odom.keys() {self.other_odom.keys()}')
        if sender_name in self.other_odom.keys():
            self.other_odom[sender_name] = [msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.z]
            # self.get_logger().info(f'odom callback {self.other_odom}')
            self.odom_ready = True
    

    def controller(self):
        # if not self.odom_ready:
        #     self.get_logger().warn("Waiting for odom data.")
        #     return  # Skip this iteration if data is not ready

    # Reset flag after using the data
        # self.odom_ready = False
        # self.get_logger().info("Controlling")
        self.get_logger().info(f"Current other_odom data: {self.other_odom}")

        cfs_odom = self.pose
        msg = []
        for cfs_name in self.other_odom:
            msg.extend(self.other_odom[cfs_name])
        xb = np.array([0 , 0])
        visible = None
        pij_vec = None
        utj = None
        gamma_ij_vec = None
        theta_dot_vec = []
        Bd_vec = None
        # self.get_logger().info(f'global message is {str(msg)}')
        gamma_ij_vec = (gamma_ij(msg,self.n,cfs_odom))
        # gamma_ij_save.append(gamma_ij_vec)
        Bd_vec = Bd(gamma_ij_vec)
        pij_vec = pij(msg,self.n,cfs_odom)
        # pij_vec_2.append(pij_vec)
        visible = visibility_zone_detector(self.dl,self.ds,self.av,Bd_vec,pij_vec)
        # vis_zone_vec.append(visible)
        # utj = 0
        utj = u_t_j(pij_vec,Bd_vec,self.cj,self.d_0,self.kj,visible)
        # utj_save.append(utj)
        theta_dot = u_t(cfs_odom[0:2],xb,cfs_odom[2]) + utj
        self.get_logger().info(str(theta_dot*180/np.pi))    
        self.cmd_msg.angular.z = -theta_dot*180/np.pi # Need theta dot in deg
        # self.cmd_msg.linear.x = 0.3
        # self.cmd_msg.linear.z = 0.5        # print(f"Controller: publish{theta_dot}")
        self.cmd_pub.publish(self.cmd_msg)
     
        

class pose_subscriber(Node):

    def __init__(self):
        super().__init__('params')
        # self.get_logger().info("Node active")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('cfs_names', ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY)),
                ('cfs_id', ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY)),
                ('v', 0.0),
                ('av', 0.0),
                ('ds', 0.0),
                ('dl', 0.0),
                ('psi', 0.0),
                ('d_0', 0.0),
                ('c', 0.0),
                ('cj', 0.0),
                ('k', 0.0),
                ('kj', 0.0),
                ('rho_o', 0.0),
                ('n', 0),
                ('height',0.0)
            ]
)   
        self.cfs_names = self.get_parameter('cfs_names').get_parameter_value().string_array_value
        self.cfs_id = self.get_parameter('cfs_id').get_parameter_value().string_array_value
        self.v = self.get_parameter('v').get_parameter_value().double_value
        self.av = self.get_parameter('av').get_parameter_value().double_value
        self.ds = self.get_parameter('ds').get_parameter_value().double_value
        self.dl = self.get_parameter('dl').get_parameter_value().double_value
        self.psi = self.get_parameter('psi').get_parameter_value().double_value
        self.d_0 = self.get_parameter('d_0').get_parameter_value().double_value
        self.c = self.get_parameter('c').get_parameter_value().double_value
        self.cj = self.get_parameter('cj').get_parameter_value().double_value
        self.k = self.get_parameter('k').get_parameter_value().double_value
        self.kj = self.get_parameter('kj').get_parameter_value().double_value
        self.rho_o = self.get_parameter('rho_o').get_parameter_value().double_value
        self.n = self.get_parameter('n').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().double_value
        # self.cfs_names = ['cf231','cf123','cf232']
        # self.cfs_id = ['231','123','232']

        self.cfs = {}
        for cf_name,id in zip(self.cfs_names,self.cfs_id):
            self.cfs[cf_name] = Crazyflie(cf_name,id,self.cfs_names,self.v,self.av,self.ds,
                                          self.dl,self.psi,self.d_0,self.c,self.cj,self.k,
                                          self.kj,self.rho_o,self.n,self.height)
            # rclpy.spin(self.cfs[cf_name])

    
    def getnodes(self):
        return list(self.cfs.values())
    


def main(args=None):
    rclpy.init(args=args)

    posesubscriber = pose_subscriber()

    # Using MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    # Add each Crazyflie node to the executor
    for node in posesubscriber.getnodes():
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in posesubscriber.getnodes():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
