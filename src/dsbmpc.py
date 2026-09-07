from utility import *
from ship_model import *
from shapely.geometry import Point
from map_polygons import *
import time
from random import randint
from config import *


class DSBMPC:
    def __init__(self, ownship, pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step):
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
        self.chi_ca = np.deg2rad(np.array([-90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]))
        self.p_ca = np.array([0.0, 0.5, 1.0])
        self.chi_ca_last = 0
        self.p_ca_last = 1

        # Last proposed control actions
        self.ppsd_chi = ownship.psi
        self.ppsd_u = ownship.u
        self.cost_last = np.inf
        self.msg_last = ''
        self.ppsd_psi = ownship.psi
        self.ppsd_speed = ownship.u

    def ship_trajectory(self, ship, chi_ca, p_ca):
        # Creating constant velocity and course ship trajectory within the prediction horizon
        self.x_pred[0] = ship.x
        self.y_pred[0] = ship.y
        self.psi_pred[0] = normalize_angle(ship.chi_d + chi_ca)
        self.u_pred[0] = ship.u * p_ca
        
        # states = [0, 'ship_1', x, y, psi, u, v, r, "PDV", ship1_trajectory]
        os_temp_states = [0, ship.id, self.x_pred[0], self.y_pred[0], self.psi_pred[0], 
                          self.u_pred[0], self.v_pred[0], self.r_pred[0], ship.r18, ship.wp]
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
            
            H0 = (R * C) + (params['Kappa'] * mu) + (params['grounding'] * G)

            if H0 > H1:
                H1 = H0

        # Penalty function
        #H2 = params['k_p'] * (1 - p_ca) + self.k_ci(chi_ca, params) + params['k_delta_p'] * \
        #     abs(self.p_ca_last - p_ca) + self.delta_chi(chi_ca, params)

        # Cost function: Hazard value for the scenario (H)
        cost = H1 #+ H2
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


    def cost_new(self, targetship, chi_ca, p_ca, params, ground_flag):
        # Initial parameters
        t, H1, cost = 0, 0, 0
        min_dist = np.inf

        for i in range(self.sample_size):
            mu = 0
            t += self.dt

            euc_dist = distance(self.x_pred[i], self.y_pred[i], targetship.x_pred[i], targetship.y_pred[i])
            rule = colregs_rule(self.x_pred[i], self.y_pred[i], self.psi_pred[i], self.u_pred[i], 
                                targetship.x_pred[i], targetship.y_pred[i], targetship.psi_pred[i], targetship.u_pred[i])
            rel_bear = int(relative_bearing(self.x_pred[i], self.y_pred[i], self.psi_pred[i], targetship.x_pred[i], targetship.y_pred[i]))
            
            if euc_dist < min_dist:
                min_dist = euc_dist
            
            # Violation of COLREGs
            if (rule == "HO-GW" and rel_bear <= math.radians(13)) or \
                ((rule == "CR-GW" or rule == "CR-SO") and rel_bear <= 0) and \
                (rule != "ON-SO" or (rule == 'OG' and abs(rel_bear) <= math.radians(22.5))):
                mu = 1
            
            if (rule == "CR-SO" or rule == "ON-SO") and chi_ca != self.chi_ca_last:
                omega = 1
            else:
                omega = 0
            
            H0 = 1.0 * (500 / (min_dist + 1)) + (5.0 * mu) + (5.0 * omega)

            if H0 > H1:
                H1 = H0

        # Penalty function
        H2 = params['k_p'] * (1 - p_ca) + self.k_ci(chi_ca, params) + params['k_delta_p'] * \
                    abs(self.p_ca_last - p_ca) + self.delta_chi(chi_ca, params)

        # Cost function: Hazard value for the scenario (H)
        cost = H1 + 0.01 * H2

        return cost



    def get_optimal_ctrl_offset(self, ownship, targetship, ts_list, selfish_behave=False):
        sync_t = 0.6
        start_time = time.time()
        cost = np.inf
        ground_flag = False

        # Checking distance to closest land polygon
        ship_point = Point(ownship.x, ownship.y)
        static_dist = ship_point.distance(poly_full)
        if static_dist <= self.params['d_init']:
            ground_flag = True

        if selfish_behave:
            self.chi_ca = np.array([0.0])
            self.p_ca = np.array([1.0])
        
        elif not selfish_behave:
            self.chi_ca = np.deg2rad(np.array([-90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]))
            self.p_ca = np.array([0.0, 0.5, 1.0])

            if targetship.msg_count == 0:
                # If this is the initial negotiation step propose a selfish maneuver
                self.chi_ca = np.deg2rad(np.array([-30.0, -15.0, 0.0, 15.0, 30.0]))
                self.p_ca = np.array([1.0])
            elif targetship.msg_count == 1:
                # If this is the second negotiation step implement relaxation on the proposal
                self.chi_ca = np.deg2rad(np.array([-60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0]))
                self.p_ca = np.array([0.5, 1.0])
            elif targetship.msg_count > 1:
                # Propose a control action from whole control action sets
                self.chi_ca = np.deg2rad(np.array([-90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]))
                self.p_ca = np.array([0.0, 0.5, 1.0])

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
                        self.params = {'d_init': 2500, 'd_close': 2500, 'd_safe': (1500 / (abs(rel_bear) ** 2 + 3.0)),
                                       'k_p': 10.0, 'k_delta_p': 5.0, 'Kappa': 10, 'rho': 0.0,
                                       'p': 1.0, 'q': 4.0, 'k_coll': 0.05,
                                       'k_chi_stb': 5.0, 'k_chi_port': 5.2, 'k_dchi_stb': 4.0, 'k_dchi_port': 4.2,
                                       'p_g': 1.0, 'q_g': 2.5, 'grounding': 5.0}
                    
                    # Calculate cost function
                    #cost_k = self.cost_function(targetship, chi_ca[i], p_ca[j], self.params, ground_flag)
                    cost_k = self.cost_new(targetship, self.chi_ca[i], self.p_ca[j], self.params, ground_flag)
                    if cost_k > cost_i:
                        cost_i = cost_k

                if cost_i < cost:
                    cost = cost_i
                    u_os_best = self.p_ca[j]
                    chi_os_best = self.chi_ca[i]

        # Calculating optimal control actions
        self.chi_ca_last = chi_os_best
        self.p_ca_last = u_os_best

        # Sync for time to prevent lag between ships
        end_time = time.time()
        delta_time = end_time - start_time
        if delta_time < sync_t:
            wait_time = sync_t - delta_time
            time.sleep(wait_time)
        return cost, u_os_best, chi_os_best
    

    def direct_msg_request(self, ship_id, header, sender, receiver, sender_psi_u, receiver_psi_u, msg_id):
        """
        Direct messaging (ROS service client)
        """
        rospy.wait_for_service(f'direct_msg_{ship_id}')
        try:
            dir_msg = rospy.ServiceProxy(f'direct_msg_{ship_id}', DirectMessage)
            resp = dir_msg(header, sender, receiver, sender_psi_u, receiver_psi_u, msg_id)
            print("\n", resp, "\n")
            return resp
        except rospy.ServiceException as e:
            print(sender, "Service call failed: %s"%e)

    
    def answer_request(self, req, ownship, targetship, ts_list):
        """
            Evaluate the received direct message and prepare answer to exchange.
        """
        # If received message type is acknowledgement:
        if req.header == "ACK":
            # Apply acknowledged control actions
            ownship.set_opt_ctrl(self.ppsd_chi, self.ppsd_u)
            # Erase message counter of the targetship to start a new negotiation later
            targetship.msg_count = 0
            targetship.msg_last = "ACK"
            self.msg_last = "ACK"
            return "ACK", req.receiver_psi_u[0], req.receiver_psi_u[1], req.sender_psi_u[0], req.sender_psi_u[1], req.msg_id

        # If received message type is intention query:
        if req.header == "INT":
            # Calculate optimal action and propose
            cost, u_os_best, chi_os_best = self.get_optimal_ctrl_offset(ownship, targetship, ts_list)
            os_u = ownship.u_d * u_os_best
            os_psi = wrap_to_pi(chi_os_best + ownship.chi_d)
            targetship.msg_count += 1
            targetship.msg_last = "PPS"
            self.ppsd_chi = chi_os_best
            self.ppsd_u = u_os_best
            self.cost_last = cost
            self.msg_last = "PPS"
            return "PPS", os_psi, os_u, req.sender_psi_u[0], req.sender_psi_u[1], req.msg_id

        # If received message type is proposal:
        elif req.header == "PPS":
            # Calculate cost1 with ownship's current psi,u and proposed optimal actions from targetship
            targetship.trajectory_prediction(psi=req.sender_psi_u[0], U=req.sender_psi_u[1], pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step)
            cost1, u_os_best1, chi_os_best1 = self.get_optimal_ctrl_offset(ownship, targetship, ts_list, selfish_behave=True)

            # Calculate cost2 with ownship's optimal psi,u and proposed optimal actions from targetship
            targetship.trajectory_prediction(psi=req.sender_psi_u[0], U=req.sender_psi_u[1], pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step)
            cost2, u_os_best2, chi_os_best2 = self.get_optimal_ctrl_offset(ownship, targetship, ts_list, selfish_behave=False)
            
            # Calculate cost3 with ownship's optimal psi,u and current actions of targetship
            targetship.trajectory_prediction(psi=targetship.psi, U=targetship.u, pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step)
            cost3, u_os_best3, chi_os_best3 = self.get_optimal_ctrl_offset(ownship, targetship, ts_list, selfish_behave=False)
            
            print("\n", ownship.id, "COST VALUES: ", cost1, cost2, cost3)
            print(ownship.id, "CHI: ", chi_os_best1, chi_os_best2, chi_os_best3)
            print(ownship.id, "U: ", u_os_best1, u_os_best2, u_os_best3, "\n")

            # Compare three costs
            if (cost1 <= cost2 and cost1 <= cost3):
                os_u = ownship.u * u_os_best1
                os_psi = wrap_to_pi(chi_os_best1 + ownship.chi_d)
                targetship.msg_count = 0
                targetship.msg_last = "ACK"
                self.ppsd_chi = 0
                self.ppsd_u = 1
                self.cost_last = cost1
                self.msg_last = "ACK"
                ownship.set_opt_ctrl(self.ppsd_chi, self.ppsd_u)
                return "ACK", os_psi, os_u, req.sender_psi_u[0], req.sender_psi_u[1], req.msg_id

            elif cost2 < self.cost_last and cost2 < cost1 and cost2 <= cost3:
                os_u = ownship.u_d * u_os_best2
                os_psi = wrap_to_pi(chi_os_best2 + ownship.chi_d)
                targetship.msg_count += 1
                targetship.msg_last = "PPS"
                self.ppsd_chi = chi_os_best2
                self.ppsd_u = u_os_best2
                self.cost_last = cost2
                self.msg_last = "PPS"
                return "PPS", os_psi, os_u, req.sender_psi_u[0], req.sender_psi_u[1], req.msg_id

            elif cost3 < self.cost_last and cost3 < cost1 and cost3 < cost2:
                os_u = ownship.u_d * u_os_best3
                os_psi = wrap_to_pi(chi_os_best3 + ownship.chi_d)
                targetship.msg_count += 1
                targetship.msg_last = "PPS"
                self.ppsd_chi = chi_os_best3
                self.ppsd_u = u_os_best3
                self.cost_last = cost3
                self.msg_last = "PPS"
                return "PPS", os_psi, os_u, targetship.psi, targetship.u, req.msg_id
