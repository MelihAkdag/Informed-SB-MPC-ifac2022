import math
import numpy as np
from utility import *
from config import *


class DynamicModel:
    def __init__(self, states):
        # states = [t, id, x, y, psi, u, v, r, r18, wps]
        self.id = states[1]
        self.x = states[2]
        self.y = states[3]
        self.psi = states[4]
        self.u = states[5]
        self.v = states[6]
        self.r = states[7]
        self.r18 = states[8]
        self.wp = states[9]
        self.next_wp_id = 1

        # LOS guidance parameters
        self.chi_d = 0  # Desired course
        self.R_a = 250  # Radius of acceptance for path following
        self.delta = 1000  # Look ahead distance

        self.u_opt = 1
        self.chi_opt = 0
        self.u_d = states[5]
        self.u_c = self.u_d * self.u_opt
        self.chi_c = wrap_to_pi(self.chi_d + self.chi_opt)

        # Logging for animation and analysis
        self.x_log = [states[2]]
        self.y_log = [states[3]]
        self.psi_log = [states[4]]
        self.u_log = [states[5]]
        self.wp_log = [self.wp]
        self.u_os_best_log = 0.0
        self.chi_os_best_log = 0.0
        self.d_safe_log = []
        self.safe_dist_flag = 0.0
        self.goal_flag = 0.0

        self.length = 116
        self.width = 25
        self.M = 15524000
        self.I_z = 1.0437 * 10**10
        self.x_g = -3.7

        # added mass terms
        self.X_udot = -979290.0
        self.Y_vdot = -10727527.0
        self.Y_rdot = -11357800.0
        self.N_rdot = -6.2422 * 10**9
        self.N_vdot = 0.0  # it is not supported.

        # linear damping terms
        self.X_u = -1650.0
        self.Y_v = -10500.6
        self.Y_r = 0.0
        self.N_v = 0.0
        self.N_r = -19622349  #-2452793600.0

        # inertia matrix = rigid body mass matrix + added mass matrix
        self.Mtot = np.array([[self.M - self.X_udot, 0, 0],
                              [0, self.M - self.Y_vdot, (self.M * self.x_g)-self.Y_rdot],
                              [0, (self.M * self.x_g)-self.Y_rdot, self.I_z - self.N_rdot]])
        self.Minv = np.linalg.inv(self.Mtot)

        # coriolis and centripetal matrix
        self.Cvv = np.zeros((3, 1))
        # Damping matrix
        self.Dvv = np.zeros((3, 1))
        # Force matrix
        self.tau = np.zeros((3, 1))

        # Controller gains
        self.Kp = 0.1
        self.Kp_psi = 0.05

    def move(self, dt):
        # Updating reference course and speed
        self.u_c = self.u_d * self.u_opt
        self.chi_c = wrap_to_pi(self.chi_opt + self.chi_d)

        self.psi = normalize_angle(self.psi)
        # rotation matrix elements
        r11 = math.cos(self.psi)
        r12 = -math.sin(self.psi)
        r21 = math.sin(self.psi)
        r22 = math.cos(self.psi)

        self.Cvv[0] = (-self.M * (self.x_g * self.r + self.v) + self.Y_vdot * self.v + self.Y_rdot * self.r) * self.r
        self.Cvv[1] = (self.M * self.u - self.X_udot * self.u) * self.r
        self.Cvv[2] = (self.M * (self.x_g * self.r + self.v) - self.Y_vdot * self.v - self.Y_rdot * self.r) * self.u + (-self.M * self.u + self.X_udot * self.u) * self.v

        self.Dvv[0] = -self.X_u * self.u
        self.Dvv[1] = -self.Y_v * self.v - self.Y_r * self.r
        self.Dvv[2] = -self.N_v * self.v - self.N_r * self.r

        # Controller
        delta_psi = wrap_to_2pi(self.chi_c) - wrap_to_2pi(self.psi)
        r_d = self.Kp_psi * wrap_to_pi(delta_psi)

        Fx = self.Cvv[0] + self.Dvv[0] - self.Kp * (self.M - self.X_udot) * (self.u - self.u_c)
        Fy = self.Cvv[1] + self.Dvv[1] - self.Kp * (self.M - self.Y_vdot) * self.v - self.Kp * (self.M * self.x_g - self.Y_rdot) * (self.r - r_d)
        Fz = self.Cvv[2] + self.Dvv[2] - self.Kp * (self.M * self.x_g - self.Y_rdot) * self.v - self.Kp * (self.I_z - self.N_rdot) * (self.r - r_d)

        self.tau[0] = Fx
        self.tau[1] = Fy
        self.tau[2] = Fz

        # Eta
        self.x = self.x + dt * (r11 * self.u + r12 * self.v)
        self.y = self.y + dt * (r21 * self.u + r22 * self.v)
        self.psi = self.psi + dt * self.r
        self.psi = wrap_to_pi(self.psi)

        # Mu
        mu_dot = self.Minv @ (self.tau - self.Cvv - self.Dvv)
        mu_dot = mu_dot.flatten()
        self.u = self.u + dt * mu_dot[0]
        self.v = self.v + dt * mu_dot[1]
        self.r = self.r + dt * mu_dot[2]

        # Waypoint manager
        self.update_active_wp()
        e = self.update_desired_course()  # Cross track error will be used in target trajectory prediction

        # Check if ship reached its last waypoint
        if (self.wp[-1][0] - self.x) ** 2 + (self.wp[-1][1] - self.y) ** 2 <= self.R_a ** 2:
            self.u = 0
            self.v = 0
            self.goal_flag = 1.0
        return e

    def set_opt_ctrl(self, chi_opt, u_opt):
        self.chi_opt = chi_opt
        self.u_opt = u_opt

    def update_active_wp(self):
        if self.next_wp_id < len(self.wp) - 1:
            dist_next_wp = np.array([self.wp[self.next_wp_id][0] - self.x, self.wp[self.next_wp_id][1] - self.y])
            if np.linalg.norm(dist_next_wp) <= self.R_a:
                # OS is inside the radius of acceptance value of the waypoint
                self.u_d = self.wp[self.next_wp_id][2]
                self.next_wp_id += 1
                return
            last_wp_segment = np.array([self.wp[self.next_wp_id][0] - self.wp[self.next_wp_id - 1][0],
                                        self.wp[self.next_wp_id][1] - self.wp[self.next_wp_id - 1][1]])
            segment_passed = np.dot(normalize_vec(last_wp_segment), normalize_vec(dist_next_wp)) < np.cos(np.pi / 2)
            if segment_passed:
                self.next_wp_id += 1

    def update_ca_wp(self, t):
        # Updates next waypoint with collision avoidance intention
        #if self.next_wp_id < len(self.wp) - 1:
        x = round(self.x + (self.u * math.cos(self.psi) - self.v * math.sin(self.psi)) * 120, 2)
        y = round(self.y + (self.u * math.sin(self.psi) + self.v * math.cos(self.psi)) * 120, 2)
        self.wp[self.next_wp_id] = (x, y, round(math.sqrt(self.u**2+self.v**2), 2))

    def update_desired_course(self):
        # Based on proportional LOS guidancce from Fossen
        pi_p = math.atan2(self.wp[self.next_wp_id][1] - self.wp[self.next_wp_id - 1][1],
                          self.wp[self.next_wp_id][0] - self.wp[self.next_wp_id - 1][0])
        e = -np.sin(pi_p) * (self.x - self.wp[self.next_wp_id - 1][0]) + np.cos(pi_p) * (
                    self.y - self.wp[self.next_wp_id - 1][1])
        self.chi_d = wrap_to_pi(pi_p + math.atan(-e / self.delta))
        return e


class KinematicModel:
    def __init__(self, states):
        # states = [t, id, x, y, psi, u, v, r, r18, wps]
        self.id = states[1]
        self.x = states[2]
        self.y = states[3]
        self.psi = states[4]
        self.u = states[5]
        self.v = states[6]
        self.r = states[7]
        self.r18 = states[8]
        self.wp = states[9]
        self.next_wp_id = 1

        # LOS guidance parameters
        self.chi_d = 0  # Desired course
        self.R_a = 50  # Radius of acceptance for path following
        self.delta = 500  # Look ahead distance

        self.u_opt = 1
        self.chi_opt = 0
        self.u_d = states[5]
        self.u_c = self.u_d * self.u_opt
        self.chi_c = wrap_to_2pi(self.chi_d + self.chi_opt)

        # Logging for animation and analysis
        self.x_log = [states[2]]
        self.y_log = [states[3]]
        self.psi_log = [states[4]]
        self.u_log = [states[5]]
        self.wp_log = [self.wp]
        self.u_os_best_log = 0.0
        self.chi_os_best_log = 0.0
        self.d_safe_log = []
        self.safe_dist_flag = 0.0
        self.goal_flag = 0.0

    def move(self, dt):
        # Kinematic Model:
        self.u_c = self.u_d * self.u_opt
        self.chi_c = wrap_to_2pi(self.chi_opt + self.chi_d)

        self.x += (self.u * math.cos(self.psi) - self.v * math.sin(self.psi)) * dt
        self.y += (self.u * math.sin(self.psi) + self.v * math.cos(self.psi)) * dt
        self.psi += self.r * dt
        self.psi = wrap_to_2pi(self.psi)

        a = np.sign(self.u_c - self.u) * min(abs(self.u_c - self.u), 0.2)
        self.u += a * dt
        self.u = np.sign(self.u) * min(abs(self.u), 10)

        delta_psi = math.atan2(np.sin(self.chi_c - self.psi), np.cos(self.chi_c - self.psi))
        self.r = np.sign(delta_psi) * min(abs(delta_psi), np.deg2rad(5))

        self.update_active_wp()
        e = self.update_desired_course()  # Cross track error will be used in target trajectory prediction

        # Check if ship reached its last waypoint
        if (self.wp[-1][0] - self.x) ** 2 + (self.wp[-1][1] - self.y) ** 2 <= self.R_a ** 2:
            self.u = 0
            self.v = 0
            self.goal_flag = 1.0
        return e

    def set_opt_ctrl(self, chi_opt, u_opt):
        self.chi_opt = chi_opt
        self.u_opt = u_opt

    def update_active_wp(self):
        if self.next_wp_id < len(self.wp) - 1:
            dist_next_wp = np.array([self.wp[self.next_wp_id][0] - self.x, self.wp[self.next_wp_id][1] - self.y])
            if np.linalg.norm(dist_next_wp) <= self.R_a:
                # OS is inside the radius of acceptance value of the waypoint
                self.u_d = self.wp[self.next_wp_id][2]
                self.next_wp_id += 1
                return
            last_wp_segment = np.array([self.wp[self.next_wp_id][0] - self.wp[self.next_wp_id - 1][0],
                                        self.wp[self.next_wp_id][1] - self.wp[self.next_wp_id - 1][1]])
            segment_passed = np.dot(normalize_vec(last_wp_segment), normalize_vec(dist_next_wp)) < np.cos(np.pi / 2)
            if segment_passed:
                self.next_wp_id += 1

    def update_desired_course(self):
        # Based on proportional LOS guidance from Fossen
        pi_p = math.atan2(self.wp[self.next_wp_id][1] - self.wp[self.next_wp_id - 1][1],
                          self.wp[self.next_wp_id][0] - self.wp[self.next_wp_id - 1][0])
        # cross-track error
        e = -np.sin(pi_p) * (self.x - self.wp[self.next_wp_id - 1][0]) + np.cos(pi_p) * (
                    self.y - self.wp[self.next_wp_id - 1][1])
        self.chi_d = wrap_to_2pi(pi_p + math.atan(-e / self.delta))
        return e


class TargetShip:
    """
        Creating the targetship object for ownship's collision avoidance algorithm.
    """
    def __init__(self, rx_data, id):
        # rx_data : [time, ship_id, x, y, psi, speed, r18, wps]
        self.id = id
        self.x = rx_data[2]
        self.y = rx_data[3]
        self.psi = rx_data[4]
        self.u = rx_data[5]
        self.v = 0.0
        self.r18 = rx_data[6]
        self.wp = rx_data[7]
        self.msg_last = None
        self.msg_count = 0
        self.distance = 0.0
        self.dcpa = 0.0
        self.tcpa = 0.0
        self.cri = 0.0
        self.colregs_role = None
        self.msg_id = 0
        # Logging for animation and analysis
        self.x_log = [rx_data[2]]
        self.y_log = [rx_data[3]]
        self.psi_log = [rx_data[4]]
        self.u_log = [rx_data[5]]
        self.wp_log = [self.wp]
        # Targetship's proposed optimal control actions (psi and u) for negotiation
        self.ppsd_psi = self.psi
        self.ppsd_u = self.u 
    
    def trajectory_prediction(self, psi=0.0, U=0.0, pred_hor=600, time_step=5):
        # TS data arrays for prediction horizon
        self.sample_size = int(pred_hor / time_step)
        self.x_pred = np.zeros(self.sample_size)
        self.y_pred = np.zeros(self.sample_size)
        self.psi_pred = np.zeros(self.sample_size)
        self.u_pred = np.zeros(self.sample_size)
        self.v_pred = np.zeros(self.sample_size)
        # Setting the initial states
        self.x_pred[0] = self.x
        self.y_pred[0] = self.y
        self.psi_pred[0] = psi
        self.u_pred[0] = U 
        # states = [t, id, x, y, psi, u, v, r, r18, wps]
        ts_temp_states = [0, self.id, self.x_pred[0], self.y_pred[0], self.psi_pred[0], self.u_pred[0], 0.0, 0.0, self.r18, self.wp]
        ts_temp = KinematicModel(ts_temp_states)

        # Targetship trajectory prediction
        for i in range(1, self.sample_size):
            e = ts_temp.move(1)
            # If cross track error is smaller than threshold value or XTD value from route exchange message,
            # prediction is based on route exchange

            if abs(e) <= 100:  # Uncomment to enable route exchange
            #if abs(e) < 0:      # Uncomment to disable route exchange
                self.x_pred[i] = ts_temp.x
                self.y_pred[i] = ts_temp.y
                self.psi_pred[i] = ts_temp.psi
                self.u_pred[i] = ts_temp.u
            # If cross track error bigger than threshold value or XTD value,
            # prediction is based on constant velocity model
            else:
                self.x_pred[i] = self.x_pred[i-1] + self.u_pred[i-1] * math.cos(self.psi_pred[i-1]) * time_step
                self.y_pred[i] = self.y_pred[i-1] + self.u_pred[i-1] * math.sin(self.psi_pred[i-1]) * time_step
                self.psi_pred[i] = self.psi_pred[i-1]
                self.u_pred[i] = self.u_pred[i-1]

    def evaluate_pose(self, ownship):
        # Evaluating targetships distance, CRI, and COLREGs role of the ownship
        self.distance = distance(ownship.x, ownship.y, self.x, self.y)
        dcpa, tcpa = cpa(ownship, self)
        self.dcpa = dcpa
        self.tcpa = tcpa
        self.cri = cri(self.dcpa, self.tcpa)
        self.colregs_role = colregs_rule(ownship.x, ownship.y, ownship.psi, ownship.u, self.x, self.y, self.psi, self.u)
    


def create_ts_data(ts_id_list, ts_list, rx_data, ownship, dt):
    """
        Creating and updating the list of targetship objects for the ownship.
    """
    try:
        rx_data_keys = list(rx_data.keys())

        for id in rx_data_keys:
            if id == ownship.id:
                ownship.x_log.append(rx_data[id][2])
                ownship.y_log.append(rx_data[id][3])
                ownship.psi_log.append(rx_data[id][4])
                ownship.u_log.append(rx_data[id][5])
                ownship.wp_log.append(rx_data[id][7])

            elif (id not in ts_id_list) and (id != ownship.id):
                ts_id_list.append(id)
                # Creating targetship object
                ts = TargetShip(rx_data[id], id)
                # Predicting its future trajectory
                ts.trajectory_prediction(psi=ts.psi, U=ts.u, pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step)
                ts.evaluate_pose(ownship)
                ts_list.append(ts)

            elif id in ts_id_list:
                for ts in ts_list:
                    if id == ts.id:
                        ts.id = rx_data[id][1]
                        ts.x = rx_data[id][2]
                        ts.y = rx_data[id][3]
                        ts.psi = rx_data[id][4]
                        ts.u = rx_data[id][5]
                        ts.r18 = rx_data[id][6]
                        ts.wp = rx_data[id][7]
                        # Updating future trajectory prediction
                        ts.trajectory_prediction(psi=ts.psi, U=ts.u, pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step)
                        ts.evaluate_pose(ownship)
                        # Logging data
                        ts.x_log.append(rx_data[id][2])
                        ts.y_log.append(rx_data[id][3])
                        ts.psi_log.append(rx_data[id][4])
                        ts.u_log.append(rx_data[id][5])
                        ts.wp_log.append(rx_data[id][7])
        return ts_id_list, ts_list
    except:
        print(ownship.id, ": Error at creating 'create_ts_data'!")
        return ts_id_list, ts_list

