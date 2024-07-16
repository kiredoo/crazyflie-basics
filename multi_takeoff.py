import threading
import time
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

URI1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
URI2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E711')

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)
position_estimate = [0, 0, 0]

def log_pos_callback(timestamp, data, logconf):
    # print(data)
    global position_estimate
    position_estimate[0] = - data['stateEstimate.y']
    position_estimate[1] = data['stateEstimate.x']
    position_estimate[2] = data['stateEstimate.yaw'] + 180
def take_off_1(scf):
    global position_estimate
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        print(position_estimate)
        mc.stop()

def take_off_2(scf):
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




# Shared variable
shared_variable = 1

# Frequency control
frequency1 = 1  # Frequency for the first loop (in Hz)
frequency2 = 2  # Frequency for the second loop (in Hz)


def cf1():
    global shared_variable
    # cflib.crtp.init_drivers()

    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',cb=param_deck_flow)
        time.sleep(1)
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        take_off_1(scf)

def cf2():
    global shared_variable
    
    with SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf:
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',cb=param_deck_flow)
        time.sleep(1)
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)
        logconf.start()
        take_off_2(scf)
        logconf.stop()
cflib.crtp.init_drivers()

# Creating threads
thread1 = threading.Thread(target=cf1)
thread2 = threading.Thread(target=cf2)

# Starting threads
thread1.start()
thread2.start()

# Joining threads to the main thread
thread1.join()
thread2.join()