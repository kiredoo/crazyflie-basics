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

URI1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E710')
URI2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E711')

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

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




# Shared variable
shared_variable = 1

# Frequency control
frequency1 = 1  # Frequency for the first loop (in Hz)
frequency2 = 2  # Frequency for the second loop (in Hz)


def loop3():
    global shared_variable
    # cflib.crtp.init_drivers()

    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        take_off_simple(scf)

def loop4():
    global shared_variable
    
    with SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        take_off_simple(scf)
cflib.crtp.init_drivers()

# Creating threads
thread1 = threading.Thread(target=loop4)
thread2 = threading.Thread(target=loop3)

# Starting threads
thread1.start()
thread2.start()

# Joining threads to the main thread
thread1.join()
thread2.join()