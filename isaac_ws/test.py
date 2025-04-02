import numpy as np

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

agent_vec = [1, 1]
pij_vec = [1, -1]
print(get_angle_j(agent_vec,pij_vec))