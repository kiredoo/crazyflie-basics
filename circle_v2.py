import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
import numpy as np


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E704')

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)
# global position_estimate
position_estimate = [0, 0, 0]

global cf_data_log
global run_time
global dt
global psi
global c
global v
global n
global k
global radius
global rho_o
global control_signal
global start_angle
control_signal = []
start_angle = 90 #deg
dt = 0.01
c = 1.1
psi = np.pi*(3.01/2)

v = 0.2
n = 1
k = 0.1
radius = 0.6
lambda_v = np.exp((2*v)/(k*radius*np.pi))
rho_o = ((c-1)*radius)/((lambda_v*c) - 1)
run_time = 30
cf_data_log = []



def log_pos_callback(timestamp, data, logconf):
    # print(data)
    cf_data_log.append([data['stateEstimate.x'],data['stateEstimate.y'],data['stateEstimate.yaw']])
    global position_estimate
    position_estimate[0] = - data['stateEstimate.y']
    position_estimate[1] = data['stateEstimate.x']
    position_estimate[2] = data['stateEstimate.yaw'] + 180

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        start = time.time()
        while (time.time() - start) < run_time:
            time.sleep(0.2)
            xb = [-0.7,0.7]
            global position_estimate
            omega = u_t(position_estimate[0:-1],xb,position_estimate[2] + start_angle)
            radius_t = np.abs(v/omega)
            if omega > 0:
                mc.start_circle_left(radius_t,v)
                control_signal.append([time.time() - start,radius_t,v])
            else:
                mc.start_circle_right(radius_t,v)
                control_signal.append([time.time() - start,-radius_t,v])
            # position = cf_data_log[-1][0:-1]
            # xb = [-0.7,0.7]
            # global position_estimate
            # omega = u_t(position,xb,position_estimate[0] + start_angle)
            # radius_t = np.abs(v/omega)
            # if omega > 0:
            #     mc.circle_left(radius_t,v,10)
            # else:
            #     mc.circle_right(radius_t,v,10)

        time.sleep(3)
        mc.stop()


def g_function(xb,xv,c,rho_o):
    rho_v = np.array(xb) - np.array(xv)
    rho = np.linalg.norm(rho_v)
    g = np.log((((c-1)*rho) + rho_o)/(c*rho_o))
    if (rho<0.000000001):
        g = 0
    return g
def alpha_d(gamma,psi):
    if ((gamma>=0)and (gamma <=psi)):
        return gamma
    elif ((gamma>psi)and (gamma <2*np.pi)):
        return gamma - 2*np.pi
    else :
        print("Error in gamma value" + gamma)

def u_t(xv,xb,heading):
    gamma = get_angle(xb[0] - xv[0],xb[1] - xv[1]) - heading
    while (gamma>2*np.pi):
        gamma = gamma -2*np.pi
    while (gamma<0):
        gamma = gamma + 2*np.pi
    #print(np.rad2deg(gamma))
    alpha = alpha_d(gamma,psi)

    g = g_function(xb,xv,c,rho_o)
    
    u = k*g*alpha
    return u

def velocity(t,n):
    return np.ones(n)*v
def adjust_angle(x,n):
    for i in range (n):
        while (x[(3*i)+2]>2*np.pi):
            x[(3*i)+2] = x[(3*i)+2] -2*np.pi
        while (x[(3*i)+2]<0):
            x[(3*i)+2] = x[(3*i)+2] + 2*np.pi


    return x

def f_unicycle(t,x,n):
    xb = np.array([0 , 0])
    v_vec = velocity(t,n)
    theta_dot_vec = u_t(x[0:2],xb,x[2])
    rhs = []
    for i in range(n):
        v = v_vec[i]
        theta_dot = theta_dot_vec[i]
        theta = x[(3*i) + 2]
        rhs.append([(v*np.cos(theta)),(v*np.sin(theta)),(theta_dot)])

    rhs = np.array(rhs)
    rhs = rhs.flatten()
    return rhs
def rk4_step(f,x,t,dt,n):
    k1 = f(t,x,n)
    k2 = f(t + (0.5*dt),x + (0.5*k1*dt),n)
    k3 = f(t + (0.5*dt),x + (0.5*k2*dt),n)
    k4 = f(t + dt,x + (k3*dt),n)

    return dt*((k1 + (2*k2) + (2*k3) + k4)/6)


def get_angle(dx,dy):
    theta = np.arctan2(dy,dx)
    if(theta<0):
        theta = theta + 2*np.pi
    return theta


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf.start()

        move_linear_simple(scf)
        logconf.stop()
        np.save("cf_circle_v3.npy",np.array(cf_data_log))
        print(control_signal)
        np.save("control_signals_v3.npy",np.array(control_signal))


