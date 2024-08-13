import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kiredoo/crazyflie-basics/isaac_ws/install/crazysim_v1'
