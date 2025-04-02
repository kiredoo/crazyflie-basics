import numpy as np
global psi 
global rho_o
global k 
global v
global c

psi = np.pi * 2
rho_o = 0.02
k = 0.075
v = 0.3
c = 2

def dist(a,b):
    d = np.array(a) - np.array(b)
    d = np.linalg.norm(d)
    return d

def get_angle(dx,dy):
    theta = np.arctan2(dy,dx)
    if(theta<0):
        theta = theta + 2*np.pi
    return theta

def get_angle_j(agent_vec,pij_vec):
    det = agent_vec[0] * pij_vec[1] - agent_vec[1] * pij_vec[0]
    dot = agent_vec[0] * pij_vec[0] + agent_vec[1] * pij_vec[1]
    theta = np.arctan2(det,dot)
    # tolerance = 1e-10
    # if abs(theta) < tolerance:
    #     theta = 0.0
    
    if theta < 0:
        theta += 2 * np.pi
    return theta

def g_function_j(pij, cv, do):
    g = np.log((((cv-1)*pij) + do)/(cv*do))
    if (pij<0.000000001):
        g = 0
    return g

# Controller calculations for collision prevention between agents

def Bd(gamma_ij):
    bd = []
    for i in range(len(gamma_ij)):
        gamma_ij_target = gamma_ij[i]
        if ((gamma_ij_target>=0) and (gamma_ij_target <= np.pi)):
            bd.append(gamma_ij_target)
        elif ((gamma_ij_target > np.pi)and (gamma_ij_target < 2*np.pi)):
            bd.append(gamma_ij_target - 2*np.pi)
        else :
            print("Error in gamma_ij value" + str(gamma_ij_target))
    return bd

def pij(x, n, agent_coord):
    pij = []
    # pij_debug = []
    
    for i in range(n-1):
        # if i == agent_index: 
        #     continue
        # print(f"Agent {i}: Comparing {agent_coord[0:2]} with {x[counter:counter+2]}")  # Debug print
        pij_magnitude = dist(agent_coord[0:2], x[3*i:3*i+2])  # Check distances
        # print(f"Distance for agent {i}: {pij_magnitude}")  # Debug the calculated distance
        
        # pij_debug.append(pij_magnitude)
        pij.append(pij_magnitude)
        # Skip over x, y, theta
    
    # pij_save.append(pij_debug)
    return pij


def gamma_ij(x,n,agent_coord):
    
    gamma_ij_vec = []
    theta_agent = agent_coord[2]
    agent_vec = [np.cos(theta_agent),np.sin(theta_agent)]
    x = np.array(x)
    agent_coord = np.array(agent_coord)
    for i in range(n-1):
        # if i == agent_index:
        #     continue
        pij_vector = x[3*i:3*i+2] - agent_coord[0:2]
        # pij_theta = get_angle(pij_vector[1],pij_vector[0])
        gamma_ij = get_angle_j(agent_vec,pij_vector)
        
        # print(f"pij theta = {pij_theta}")
        # print(f"agent coord = {theta_agent}")
        # gamma_ij = pij_theta - theta_agent
        gamma_ij_vec.append(gamma_ij)
    return gamma_ij_vec

def visibility_zone_detector(dl,ds,alpha_v,Bd_vec,pij):  # call in u_t_j
    visible = []
    #v_vector = [v*np.cos(theta), v*np.sin(theta)]
    for i in range(len(pij)):
        if pij[i] < ds:
            visible.append(True)
            continue
        elif ((pij[i] <= dl) and (abs(Bd_vec[i]) <= alpha_v)):
            visible.append(True)
        else:
            visible.append(False)
    return visible

def u_t_j(pij,b_d,cv,do,kv,visible): # return 0 when none within range_finder
    u_ij = []
    for i in range(len(visible)):
        if visible[i]:
            bij = b_d[i]
            gij = g_function_j(pij[i],cv,do)
            u_ij.append(kv*gij*bij)
    u = sum(u_ij)
    return u

#controller calculations relative to the beacon

def g_function(xb,xv,c,rho_o):
    rho_v = np.array(xb) - np.array(xv)
    rho = np.linalg.norm(rho_v)
    g = np.log((((c-1)*rho) + rho_o)/(c*rho_o))
    if (rho<0.000000001):
        g = 0
    return g

def alpha_d(gamma,psi):
    if ((gamma>=0)and (gamma <=psi)):
        return gamma
    elif ((gamma>psi)and (gamma <2*np.pi)):
        return gamma - 2*np.pi
    else :
        print("Error in gamma value" + gamma)


def u_t(xv,xb,heading):
    
    gamma = get_angle(xb[0] - xv[0],xb[1] - xv[1]) - heading
    while (gamma>2*np.pi):
        gamma = gamma -2*np.pi
    while (gamma<0):
        gamma = gamma + 2*np.pi
        #print(np.rad2deg(gamma))
    alpha = alpha_d(gamma,psi)

    g = g_function(xb,xv,c,rho_o)
        
    u = k*g*alpha
    if dist(xv,xb) < 0.1:
        u = 0
    # control_signal.append(u)
    return u

# def u_t_j(xv,xb,heading): # old utj function
#     gamma = get_angle(xb[0] - xv[0],xb[1] - xv[1]) - heading
#     while (gamma>2*np.pi):
#         gamma = gamma -2*np.pi
#     while (gamma<0):
#         gamma = gamma + 2*np.pi
#     #print(np.rad2deg(gamma))
#     alphaj = alpha_d(gamma,np.pi)

#     gj = g_function(xb,xv,cj,d_0)
    
#     u = kj*gj*alphaj
#     if dist(xv,xb) > 1.5:
#         u = 0
#     control_signal.append(u)
#     return u

def gamma_angle(xv,xb,heading):
    gamma = get_angle(xb[0] - xv[0],xb[1] - xv[1]) - heading
    while (gamma>2*np.pi):
        gamma = gamma -2*np.pi
    while (gamma<0):
        gamma = gamma + 2*np.pi
    return gamma

def velocity(t,n):
    return np.ones(n)*v

def adjust_angle(x,n):
    for i in range (n):
        while (x[(3*i)+2]>2*np.pi):
            x[(3*i)+2] = x[(3*i)+2] -2*np.pi
        while (x[(3*i)+2]<0):
            x[(3*i)+2] = x[(3*i)+2] + 2*np.pi
    return x


def get_angle(dx,dy):
    theta = np.arctan2(dy,dx)
    if(theta<0):
        theta = theta + 2*np.pi
    return theta



               
    
