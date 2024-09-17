# Crazyflie 2.1 [ROS packages] Documentation for MURO LAB

> Author : Sudhanshu Shankar

## Cyclic pursuit paper

The paper can be found here : [Collective circular motion of multi-vehicle systems](https://www.sciencedirect.com/science/article/pii/S0005109808002951)

### Summary : 

The paper presents a distributed controller for unicycle point sized vehicles to come around a circle of a stationary or moving Beacon.

**Key advantages of the controller :**

1. Does not depend on the number of agents
2. Can follow moving beacons
3. Avoids agent collisions who are within the control law


### Crazyflie fundamentals

Every time you power on a crazyflie, it initializes it's state as origin and 0 rad orientation. The world frame is also initialized at that point with x-axis pointing front and z-axis up. 