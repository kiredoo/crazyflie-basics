import numpy as np

global dt
global psi
global c
global v
global n
global k
global radius
global rho_o
dt = 0.01
c = 1.1
psi = np.pi*(3.01/2)

v = 0.2
n = 1
k = 0.5
radius = 0.6
lambda_v = np.exp((2*v)/(k*radius*np.pi))
rho_o = ((c-1)*radius)/((lambda_v*c) - 1)
