"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm


TAKEOFF_DURATION = 2.5
# HOVER_DURATION = 5.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    print('press button to land...')
    swarm.input.waitUntilButtonPressed()
  
    allcfs.land(targetHeight=0.04, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == '__main__':
    main()
