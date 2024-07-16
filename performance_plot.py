import numpy as np
import matplotlib.pyplot as plt
from sys import exit


#Data sets
experimental_data_set = r"/home/turtlebot/Sudhanshu_shankar/cf_code/cf_circle_v4.npy"
sim_data_set = r"/home/turtlebot/Sudhanshu_shankar/cf_code/gt.npy"

#Load data sets
try:
  experimental_trajectory_data = np.load(experimental_data_set)
except:
  print("Experimental data not found in the specified address")
  exit(0)


try:
  sim_trajectory = np.load(sim_data_set)
except:
  print("Simulation data not found in the specified address")
  exit(0)




experimental_trajectory_data = experimental_trajectory_data.T
plt.plot(-experimental_trajectory_data[1,:],experimental_trajectory_data[0,:])
plt.plot(np.array(sim_trajectory)[:,0] - 0.7 ,np.array(sim_trajectory)[:,1]+0.7,':',alpha = 0.5)
plt.axis('equal')
plt.grid(True)
plt.show()