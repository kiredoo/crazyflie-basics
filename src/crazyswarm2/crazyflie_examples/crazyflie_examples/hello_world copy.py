"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 3.0
FREQ = 10

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)


    pos = np.array([01.0,01.0,01.0])
    vel = np.array([0.0,0.0,0.0])
    acc = np.array([0.0,0.0,0.0])
    orientation = np.array([01.0,0.0,0.0,0.0])
    roll_rate = 0.0
    pitch_rate = 0.0
    yaw_rate = 0.0
 


    for _ in range(100):
        cf.cmdFullState(pos, vel, acc, 0.0,np.array([0.0,0.0,0.0]))
        timeHelper.sleep(1/FREQ)
    # cf.sendSetpoint(roll, pitch, yawrate, thrust)
    # pos = np.array([0.0, 0.0, 0.7])
    # cf.goTo(pos, 0, 2.0)
    # timeHelper.sleep(2.0)
    # pos = np.array([0.0, 0.0, 0.7])
    # cf.goTo(pos, 1, 2.0)
    # timeHelper.sleep(2.0)
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == '__main__':
    main()
