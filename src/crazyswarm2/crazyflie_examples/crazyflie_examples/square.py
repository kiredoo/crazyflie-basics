"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm
import numpy as np
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path
import sys


TAKEOFF_DURATION = 2.5
LAND_DURATION = 2.0
POINT_TO_POINT_DURATION= 1.5
HOVER_DURATION = 1

L = 0.6
height =0.5
delta = 0.25





left_traj = np.array([
    [-0,-0,height+delta+0.2],
    [L,L,height],
])

right_traj = np.array([
    [0,0,height-delta],
    [L,-L,height],
])

URIS = [
    'radio://0/80/2M/E7E7E7E708',
    'radio://0/80/2M/E7E7E7E715',
]

def get_cf(swarm,uris):
    cfs = []
    for uri in uris:
        for cf in swarm.allcfs.crazyflies:
            if cf.uri == uri:
                cfs.append(cf)
                break
    
    return cfs

def all_takeoff(cfs,height,takeoff_duration,sleep_time):
    for cf in cfs:
        cf.takeoff(height,duration=takeoff_duration)
    # timeHel


##
def square_simple():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    points = np.array([
        [-L,-L,height],
        [L,-L,height],
        [L,L,height],
        [-L,L,height],
    ])

    left_traj = np.array([
        [-L,-L,height],
        [L,-L,height],
        [L,L,height],
        [-L,L,height],
    ])

    right_traj = np.array([
        [L,-L,height],
        [L,L,height],
        [-L,L,height],
        [-L,-L,height],
    ])


    ## Take off
    swarm.allcfs.takeoff(targetHeight=0.5,duration=1.5)
    timeHelper.sleep(3)

    cf1,cf2 = get_cf(swarm=swarm,uris=URIS)

    for i in range(4):
        cf1.goTo(left_traj[i],0,duration=POINT_TO_POINT_DURATION)
        cf2.goTo(right_traj[i],0,duration=POINT_TO_POINT_DURATION)
        timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION)


    swarm.allcfs.land(targetHeight=0.0,duration=1.5)
    timeHelper.sleep(3)


def triangle():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    L = 0.6
    height = 0.5
    num_changes= 4

    points = np.array([
        [-L,L,height],
        [-L,-L,height],
        [L,0,height],
    ])

    URIS = [
        'radio://0/80/2M/E7E7E7E708',
        'radio://0/80/2M/E7E7E7E715',
        'radio://0/80/2M/E7E7E7E701',
    ]

    cf1,cf2,cf3= get_cf(swarm=swarm,uris=URIS)

    for i in range(num_changes):
        cf1.goTo(points[i%3],0,duration=POINT_TO_POINT_DURATION)
        cf2.goTo(points[(i+1)%3],0,duration=POINT_TO_POINT_DURATION)
        cf3.goTo(points[(i+2)%3],0,duration=POINT_TO_POINT_DURATION)
        timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION)

    swarm.allcfs.land(targetHeight=0.0,duration=1.5)
    timeHelper.sleep(3)




def circle():

    TAKEOFF_DURATION = 2.5
    LAND_DURATION = 2.0
    POINT_TO_POINT_DURATION= 1.5
    HOVER_DURATION = 0.0

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    radius = 0.5          # Radius of each circle
    num_points = 10    # Number of points per circle
    center = (0, 0)     # Center of each circle (x, y)
    offset_angles = [0, 2*np.pi / 3, -2 * np.pi / 3]  # 0, 60, and 120 degrees in radians

    theta = np.linspace(0, 2 * np.pi, num_points)

    def get_pos(theta,offset,height=0.5):
        x = center[0] + radius * np.cos(theta + offset)  
        y = center[1] + radius * np.sin(theta + offset)
        return np.array([x,y,height])
    

    URIS = [
        'radio://0/80/2M/E7E7E7E701',
        'radio://0/80/2M/E7E7E7E708',
        'radio://0/80/2M/E7E7E7E715',
    ]

    cf1,cf2,cf3= get_cf(swarm=swarm,uris=URIS)

    ## Take off
    swarm.allcfs.takeoff(targetHeight=0.5,duration=TAKEOFF_DURATION)
    timeHelper.sleep(3)
    
    cf1.goTo(get_pos(0,offset_angles[0]),0,duration=POINT_TO_POINT_DURATION)
    cf2.goTo(get_pos(0,offset_angles[1]),0,duration=POINT_TO_POINT_DURATION)
    cf3.goTo(get_pos(0,offset_angles[2]),0,duration=POINT_TO_POINT_DURATION)
    timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION+2)


    # ## Trajectory
    for t in theta:
        cf1.goTo(get_pos(t,offset_angles[0]),0,duration=POINT_TO_POINT_DURATION)
        cf2.goTo(get_pos(t,offset_angles[1]),0,duration=POINT_TO_POINT_DURATION)
        cf3.goTo(get_pos(t,offset_angles[2]),0,duration=POINT_TO_POINT_DURATION)
        timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION)
    

    ## Land
    swarm.allcfs.land(targetHeight=0.0,duration=1.5)
    timeHelper.sleep(3)

def fnacy_circle():

    TAKEOFF_DURATION = 2.5
    LAND_DURATION = 2.0
    POINT_TO_POINT_DURATION= 1.5
    HOVER_DURATION = 0.0

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    radius = 0.5          # Radius of each circle
    num_points = 10    # Number of points per circle
    center = (0, 0)     # Center of each circle (x, y)
    offset_angles = [0, 2*np.pi / 3, -2 * np.pi / 3]  # 0, 60, and 120 degrees in radians

    theta = np.linspace(0, 2 * np.pi, num_points)

    def get_pos(theta,offset,height=0.5):
        x = center[0] + radius * np.cos(theta + offset)  
        y = center[1] + radius * np.sin(theta + offset)
        return np.array([x,y,height])
    

    URIS = [
        'radio://0/80/2M/E7E7E7E701',
        'radio://0/80/2M/E7E7E7E708',
        'radio://0/80/2M/E7E7E7E715',
    ]

    cf1,cf2,cf3= get_cf(swarm=swarm,uris=URIS)

    ## Take off
    swarm.allcfs.takeoff(targetHeight=0.5,duration=TAKEOFF_DURATION)
    timeHelper.sleep(3)
    
    cf1.goTo(get_pos(0,offset_angles[0]),0,duration=POINT_TO_POINT_DURATION)
    cf2.goTo(get_pos(0,offset_angles[1]),0,duration=POINT_TO_POINT_DURATION)
    cf3.goTo(get_pos(0,offset_angles[2]),0,duration=POINT_TO_POINT_DURATION)
    timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION+2)


    # ## Trajectory
    for t in theta:
        cf1.goTo(get_pos(t,offset_angles[0]),0,duration=POINT_TO_POINT_DURATION)
        cf2.goTo(get_pos(t,offset_angles[1]),0,duration=POINT_TO_POINT_DURATION)
        cf3.goTo(get_pos(t,offset_angles[2]),0,duration=POINT_TO_POINT_DURATION)
        timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION)
    

    ## Land
    swarm.allcfs.land(targetHeight=0.0,duration=1.5)
    timeHelper.sleep(3)



def figure8():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')

    # enable logging
    allcfs.setParam('usd.logging', 1)

    URIS = [
            # 'radio://0/80/2M/E7E7E7E708',
            'radio://0/80/2M/E7E7E7E715',
            # 'radio://0/80/2M/E7E7E7E701',
        ]

    cf1= get_cf(swarm=swarm,uris=URIS)[0]

    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        cf1.uploadTrajectory(0,0,traj1)
        # cf2.uploadTrajectory(0,0,traj1)
        # cf3.uploadTrajectory(0,0,traj1)

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in [cf1]:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)

    # disable logging
    allcfs.setParam('usd.logging', 0)




def LR():

    point_to_point_duration = 1
    hover_duration = 1.5
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=height, duration=1+hover_duration)
    timeHelper.sleep(1 + hover_duration)

    for _ in range(3):
        cf.goTo(np.array([0, 0.5, height]), 0, point_to_point_duration)
        timeHelper.sleep(point_to_point_duration + hover_duration)
        cf.goTo(np.array([0, -0.5, height]), 0, point_to_point_duration)
        timeHelper.sleep(point_to_point_duration + hover_duration)

    cf.goTo(np.array([0, 0, height]), 0, point_to_point_duration)
    cf.land(targetHeight=0.04, duration=2)
    timeHelper.sleep(2)
    return


def main():

    # LR()
    # return
    
    # triangle()

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper


    cf = swarm.allcfs.crazyflies[0]


    TAKEOFF_DURATION = 2.0
    HOVER_DURATION = 2.0
    POINT_TO_POINT_DURATION = 1.0
    LAND_DURATION = 2.0

    n = 15
    points = np.column_stack((
        np.zeros(n),
        np.random.uniform(-1, 1, n),
        np.random.uniform(0.1, 0.5, n)
    ))

    cf.takeoff(targetHeight=height, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    for point in points:
        cf.goTo(np.array(point), 0, POINT_TO_POINT_DURATION)
        print(f'Goint to {point}')
        timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION)
    cf.land(targetHeight=0.04, duration=LAND_DURATION)
    timeHelper.sleep(LAND_DURATION)
    return



# def main():
#     swarm = Crazyswarm()
#     timeHelper = swarm.timeHelper
#     cf = swarm.allcfs.crazyflies[0]

#     cf.takeoff(targetHeight=height, duration=TAKEOFF_DURATION)
#     timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
#     for _ in range(4):
#         cf.goTo(np.array([0,1,0.5]), 0, POINT_TO_POINT_DURATION)
#         timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION)
#         cf.goTo(np.array([0,-1,0.5]), 0, POINT_TO_POINT_DURATION)
#         timeHelper.sleep(POINT_TO_POINT_DURATION+HOVER_DURATION)
#     cf.land(targetHeight=0.04, duration=LAND_DURATION)
#     timeHelper.sleep(LAND_DURATION)


if __name__ == '__main__':
    main()
