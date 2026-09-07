import math


# simulation settings
t = 0            # current sim time (updated at runtime)
dt = 1           # integration time step [s]
T_sim = 100      # total simulated duration [s]
rate_var = 0.4   # ROS loop rate [Hz] (messages broadcast per second)
dsbmpc_pred_hor = 3000   # Distributed SB-MPC prediction horizon [s]
dsbmpc_t_step = 10       # Distributed SB-MPC prediction time step [s]

# ships initial information
ship_ids = ['ship_1', 'ship_2']

# Each ship state follows the wire convention:
# states = [t, id, x, y, psi, u, v, r, r18, wps]
#   t=time, id=ship id, x/y=position [m], psi=heading [rad],
#   u=surge speed, v=sway speed, r=yaw rate, r18=vessel type flag,
#   wps=waypoint list. A trajectory entry is [x, y, speed].

# Scenario: ship_1 heads north, ship_2 heads west -> crossing encounter.
ship1_trajectory = [[0.0, 0.0, 8.0], [0.0, 3000.0, 8.0]]
ship1_init_states = [0, 'ship_1', ship1_trajectory[0][0], ship1_trajectory[0][1], math.radians(90.0), ship1_trajectory[0][2], 0.0, 0.0, "PDV", ship1_trajectory]

ship2_trajectory = [[2000.0, 2000.0, 8.0], [-2000.0, 2000.0, 8.0]]
ship2_init_states = [0, 'ship_2', ship2_trajectory[0][0], ship2_trajectory[0][1], math.radians(180.0), ship2_trajectory[0][2], 0.0, 0.0, "PDV", ship2_trajectory]

# Lookup of every ship's initial state, keyed by id.
all_states = {'ship_1': ship1_init_states, 'ship_2': ship2_init_states}
