import math


# simulation settings
t = 0
dt = 1
T_sim = 100
rate_var = 0.4   # X Hz (X messages per second)
dsbmpc_pred_hor = 3000
dsbmpc_t_step = 10

# ships initial information 
ship_ids = ['ship_1', 'ship_2']

# states = [t, id, x, y, psi, u, v, r, r18, wps]
ship1_trajectory = [[0.0, 0.0, 8.0], [0.0, 3000.0, 8.0]]
ship1_init_states = [0, 'ship_1', ship1_trajectory[0][0], ship1_trajectory[0][1], math.radians(90.0), ship1_trajectory[0][2], 0.0, 0.0, "PDV", ship1_trajectory]

ship2_trajectory = [[2000.0, 2000.0, 8.0], [-2000.0, 2000.0, 8.0]]
ship2_init_states = [0, 'ship_2', ship2_trajectory[0][0], ship2_trajectory[0][1], math.radians(180.0), ship2_trajectory[0][2], 0.0, 0.0, "PDV", ship2_trajectory]

all_states = {'ship_1': ship1_init_states, 'ship_2': ship2_init_states}
