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

position_estimate = [0, 0]

global cf_data_log
cf_data_log = []


def log_pos_callback(timestamp, data, logconf):
    print(data)
    cf_data_log.append([data['stateEstimate.x'],data['stateEstimate.y']])
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def zig_zag(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.circle_left(0.5,0.4,45)
        mc.forward(0.3)
        mc.circle_left(0.5,0.2,90)
        mc.forward(0.3)
        mc.circle_right(0.5,0.4,90)
        mc.forward(0.3)
        mc.circle_left(0.5,0.2,90)
        mc.forward(0.3)
        time.sleep(3)
        mc.stop()

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf.start()

        zig_zag(scf)
        logconf.stop()
        cf_data_log = np.array(cf_data_log)
        np.save("cf_data_log.npy",cf_data_log)

