import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/turtlebot/Desktop/cycli_pursuit_ws2/install/cyclic_pursuit'
