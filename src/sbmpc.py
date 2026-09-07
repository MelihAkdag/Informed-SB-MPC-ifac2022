from utility import *
from ship_model import *
from shapely.geometry import Point
from map_polygons import *
import time


class SBMPC:
    def __init__(self, pred_hor=600, time_step=5):
        # Initial parameters for SB-MPC algorithm
        self.T = pred_hor
        self.dt = time_step
        self.sample_size = int(self.T / self.dt)
        self.params = {'d_init': 1800, 'd_close': 1800, 'd_safe': 500,
                       'k_p': 10.0, 'k_delta_p': 5.0, 'Kappa': 10, 'rho': 0.0,
                       'p': 1.0, 'q': 4.0, 'k_coll': 0.05,
                       'k_chi_stb': 5.0, 'k_chi_port': 5.2, 'k_dchi_stb': 4.0, 'k_dchi_port': 4.2,
                       'p_g': 1.0, 'q_g': 2.5, 'grounding': 5.0}

        # Ship data arrays for prediction horizon
        self.x_pred = np.zeros(self.sample_size)
        self.y_pred = np.zeros(self.sample_size)
        self.psi_pred = np.zeros(self.sample_size)
        self.u_pred = np.zeros(self.sample_size)
        self.v_pred = np.zeros(self.sample_size)
        self.r_pred = np.zeros(self.sample_size)

        # Control behaviors for scenario combinations
        self.chi_ca = np.deg2rad(
            np.array([-90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]))
        self.p_ca = np.array([0.0, 0.5, 1.0])
        self.chi_ca_last = 0
        self.p_ca_last = 1

    def ship_trajectory(self, ship, chi_ca, p_ca):
        # Creating constant velocity and course ship trajectory within the prediction horizon
        self.x_pred[0] = ship.x
        self.y_pred[0] = ship.y
        self.psi_pred[0] = normalize_angle(ship.chi_d + chi_ca)
        self.u_pred[0] = ship.u * p_ca

        # states = [0, 'ship_1', x, y, psi, u, v, r, "PDV", ship1_trajectory]
        os_temp_states = [0, ship.id, self.x_pred[0], self.y_pred[0], self.psi_pred[0], 
                          self.u_pred[0], 0.0, 0.0, ship.r18, ship.wp]
        os_temp = DynamicModel(os_temp_states)

        for i in range(1, self.sample_size):
            # Ownship trajectory prediction dynamic model
            e = os_temp.move(1)
            self.x_pred[i] = os_temp.x
            self.y_pred[i] = os_temp.y
            self.psi_pred[i] = os_temp.psi
            self.u_pred[i] = os_temp.u
            # Ownship trajectory prediction kinematic model
            #self.x_pred[i] = self.x_pred[i-1] + self.u_pred[i-1] * np.cos(self.psi_pred[i-1]) * self.dt
            #self.y_pred[i] = self.y_pred[i-1] + self.u_pred[i-1] * np.sin(self.psi_pred[i-1]) * self.dt
            #self.psi_pred[i] = self.psi_pred[i-1]
            #self.u_pred[i] = self.u_pred[i-1]

    def cost_function(self, targetship, chi_ca, p_ca, params, ground_flag):
        # Initial parameters
        t0, t, H0, H1, H2, cost = 0, 0, 0, 0, 0, 0

        for i in range(self.sample_size):
            R, C, G, R18, mu, zeta = 0, 0, 0, 0, 0, 0
            t += self.dt

            rotation_os = np.array([[np.cos(self.psi_pred[i]), -np.sin(self.psi_pred[i])],
                                    [np.sin(self.psi_pred[i]), np.cos(self.psi_pred[i])]])
            vel_os = rotation_os @ np.array([self.u_pred[i], self.v_pred[i]])

            rotation_ts = np.array([[np.cos(targetship.psi_pred[i]), -np.sin(targetship.psi_pred[i])],
                                    [np.sin(targetship.psi_pred[i]), np.cos(targetship.psi_pred[i])]])
            vel_ts = rotation_ts @ np.array([targetship.u_pred[i], targetship.v_pred[i]])

            euc_dist = distance(self.x_pred[i], self.y_pred[i], targetship.x_pred[i], targetship.y_pred[i])
            rule = colregs_rule(self.x_pred[i], self.y_pred[i], self.psi_pred[i], self.u_pred[i], 
                                targetship.x_pred[i], targetship.y_pred[i], targetship.psi_pred[i], targetship.u_pred[i])
            rel_bear = int(relative_bearing(self.x_pred[i], self.y_pred[i], self.psi_pred[i], targetship.x_pred[i], targetship.y_pred[i]))
            rel_bear_os = int(relative_bearing(targetship.x_pred[i], targetship.y_pred[i], targetship.psi_pred[i], self.x_pred[i], self.y_pred[i]))

            # Grounding risk factor
            if ground_flag:
                static_dist = Point(self.x_pred[i], self.y_pred[i]).distance(poly_full)
                if static_dist <= params['d_safe']:
                    G = (1 / (abs(t - t0) ** params['p_g'])) * ((params['d_safe'] / (static_dist + 1)) ** params['q_g'])

            # Collision risk with target ship
            if euc_dist < params['d_close']:
                # Collision risk (R) and Collision cost (C)
                if euc_dist <= params['d_safe']:
                    R = (1 / (abs(t - t0) ** params['p'])) * ((params['d_safe'] / (euc_dist+1)) ** params['q'])
                    C = params['k_coll'] * np.linalg.norm(vel_os - vel_ts) ** 2

                # Violation of COLREGs
                if (rule == "HO-GW" and rel_bear <= math.radians(13)) or \
                        (((rule == "CR-GW" or rule == "CR-SO") and rel_bear <= 0) and rule != "ON-SO") or \
                        (rule == 'OG' and abs(rel_bear) <= math.radians(22.5)):
                    mu = 1

                if targetship.r18 != "PDV":
                    if abs(rel_bear) <= math.radians(22.5) and abs(rel_bear_os) <= math.radians(22.5):
                        zeta = 1.1
                    elif math.radians(22.5) < abs(rel_bear_os) <= math.radians(90):
                        zeta = 1.0

            H0 = (R * C) + (params['Kappa'] * mu) + (params['rho'] * zeta) + (params['grounding'] * G)
            if H0 > H1:
                H1 = H0

        # Penalty function
        H2 = params['k_p'] * (1 - p_ca) + self.k_ci(chi_ca, params) + params['k_delta_p'] * \
             abs(self.p_ca_last - p_ca) + self.delta_chi(chi_ca, params)

        # Cost function: Hazard value for the scenario (H)
        cost = H1 + H2
        return cost

    def k_ci(self, chi_ca, params):
        d_ci = chi_ca - self.chi_ca_last
        if chi_ca < 0:
            return params['k_chi_stb'] * (chi_ca**2)
        elif chi_ca >= 0:
            return params['k_chi_port'] * (chi_ca**2)

    def delta_chi(self, chi_ca, params):
        d_chi = chi_ca - self.chi_ca_last
        if d_chi <= 0:
            return params['k_dchi_stb'] * (d_chi**2)
        elif d_chi > 0:
            return params['k_dchi_port'] * (d_chi**2)

    def get_optimal_ctrl_offset(self, ownship, ts_list):
        sync_t = 0.6
        start_time = time.time()
        sbmpc_active = False
        ground_flag = False
        cost = np.inf

        # Checking distance to closest land polygon
        ship_point = Point(ownship.x, ownship.y)
        static_dist = ship_point.distance(poly_full)

        # If there is no target ship and land is away don't change speed or course
        if len(ts_list) == 0 and static_dist > self.params['d_init']:
            chi_os_best = 0
            u_os_best = 1
            self.chi_ca_last = 0
            self.p_ca_last = 1
            # Sync for time to prevent lag between ships
            end_time = time.time()
            delta_time = end_time - start_time
            if delta_time < sync_t:
                wait_time = sync_t - delta_time
                time.sleep(wait_time)
            return u_os_best, chi_os_best

        elif len(ts_list) == 0 and static_dist <= self.params['d_init']:
            ground_flag = True
            sbmpc_active = True

        elif len(ts_list) > 0:
            # Create target ship trajectory predictions
            for targetship in ts_list:
                # Check each target's distance to decide for initiating SB-MPC
                euc_dist = distance(ownship.x, ownship.y, targetship.x, targetship.y)
                if euc_dist <= self.params['d_init'] and static_dist <= self.params['d_init']:
                    ground_flag = True
                    sbmpc_active = True
                elif euc_dist <= self.params['d_init'] and static_dist > self.params['d_init']:
                    ground_flag = False
                    sbmpc_active = True
                elif euc_dist > self.params['d_init'] and static_dist <= self.params['d_init']:
                    ground_flag = True
                    sbmpc_active = True
                elif euc_dist > self.params['d_init'] and static_dist > self.params['d_init']:
                    ground_flag = False
                    sbmpc_active = False

        if not sbmpc_active:
            chi_os_best = 0
            u_os_best = 1
            self.chi_ca_last = 0
            self.p_ca_last = 1
            # Sync for time to prevent lag between ships
            end_time = time.time()
            delta_time = end_time - start_time
            if delta_time < sync_t:
                wait_time = sync_t - delta_time
                time.sleep(wait_time)
            return u_os_best, chi_os_best

        if sbmpc_active:
            # Create ownship trajectories for each course and speed offset combination
            for i in range(len(self.chi_ca)):
                for j in range(len(self.p_ca)):
                    self.ship_trajectory(ownship, self.chi_ca[i], self.p_ca[j])
                    # Calculate cost function for each target ship
                    cost_i = -1
                    for targetship in ts_list:
                        rel_bear = round(relative_bearing(ownship.x, ownship.y, ownship.psi, targetship.x_pred[0], targetship.y_pred[0]), 2)

                        # Parameter sets for Rule 18
                        if targetship.r18 != "PDV":
                            self.params = {'d_init': 1800, 'd_close': 1800, 'd_safe': (2100 / (abs(rel_bear)**2 + 3.0)),
                                           'k_p': 15.0, 'k_delta_p': 5.0, 'Kappa': 5.0, 'rho': 15.0,
                                           'p': 1.0, 'q': 4.0, 'k_coll': 0.05,
                                           'k_chi_stb': 5.0, 'k_chi_port': 5.2, 'k_dchi_stb': 4.0, 'k_dchi_port': 4.2,
                                           'p_g': 1.0, 'q_g': 2.5, 'grounding': 5.0}
                        # Parameter sets for normal scenarios
                        else:
                            self.params = {'d_init': 1800, 'd_close': 1800, 'd_safe': (1500 / (abs(rel_bear) ** 2 + 3.0)),
                                           'k_p': 10.0, 'k_delta_p': 5.0, 'Kappa': 10, 'rho': 0.0,
                                           'p': 1.0, 'q': 4.0, 'k_coll': 0.05,
                                           'k_chi_stb': 5.0, 'k_chi_port': 5.2, 'k_dchi_stb': 4.0, 'k_dchi_port': 4.2,
                                           'p_g': 1.0, 'q_g': 2.5, 'grounding': 5.0}

                        # Calculate cost function
                        cost_k = self.cost_function(targetship, self.chi_ca[i], self.p_ca[j], self.params, ground_flag)
                        if cost_k > cost_i:
                            cost_i = cost_k

                    if cost_i < cost:
                        cost = cost_i
                        u_os_best = self.p_ca[j]
                        chi_os_best = self.chi_ca[i]

        # Wear and tear logging for the scenario evaluation
        if u_os_best != self.p_ca_last:
            ownship.u_os_best_log += 1

        # d_safe logging for animation
        #ownship.d_safe_log.append(self.params['d_safe'])

        # Wear and tear logging for the scenario evaluation
        ownship.chi_os_best_log = ownship.chi_os_best_log + abs(chi_os_best)

        self.chi_ca_last = chi_os_best
        self.p_ca_last = u_os_best

        # Sync for time to prevent lag between ships
        end_time = time.time()
        delta_time = end_time - start_time
        if delta_time < sync_t:
            wait_time = sync_t - delta_time
            time.sleep(wait_time)
        return u_os_best, chi_os_best
