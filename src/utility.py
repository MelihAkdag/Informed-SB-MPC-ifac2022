import numpy as np
import pandas as pd
import math
import json
import rospy
from ros_mas_test.srv import DirectMessage
from ros_mas_test.srv import DirectMessageRequest
from ros_mas_test.srv import DirectMessageResponse



def normalize_vec(v: np.ndarray):
    # Return normalized vector
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    else:
        return v / norm


def wrap_to_pi(angle):
    #wraps the angle to [-pi,pi)
    res = math.fmod(angle + 2 * math.pi, 2 * math.pi)
    if res > math.pi:
        res -= 2*math.pi
    return res


def wrap_to_2pi(angle):
    # wraps the angle to [0,2*pi)
    res = math.fmod(angle + 2 * math.pi, 2 * math.pi)
    return res


def normalize_angle(angle):
    # wraps the angle to [-pi, pi)
    while angle <= -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle


def distance(ship1_x, ship1_y, ship2_x, ship2_y):
    euc_dist = math.sqrt((ship1_x - ship2_x)**2 + (ship1_y - ship2_y)**2)
    return euc_dist


def true_bearing(ship1_x, ship1_y, ship2_x, ship2_y):
    # result in radians between -pi and pi
    true_bearing = math.atan2((ship2_y - ship1_y), (ship2_x - ship1_x))
    # result in radians between 0 and 2*pi
    true_bearing = wrap_to_2pi(true_bearing)
    return true_bearing


def relative_bearing(ship1_x, ship1_y, ship1_psi, ship2_x, ship2_y):
    rel_bearing = true_bearing(ship1_x, ship1_y, ship2_x, ship2_y) - ship1_psi
    # Relative bearing is between -pi, pi
    rel_bearing = wrap_to_pi(rel_bearing)
    return rel_bearing


def cpa(ship1, ship2, dt=1):
    x1, y1, u1, psi1 = ship1.x, ship1.y, ship1.u, ship1.psi
    x2, y2, u2, psi2 = ship2.x, ship2.y, ship2.u, ship2.psi
    dcpa_values = np.zeros(600)
    for i in range(600):
        dcpa_values[i] = np.sqrt((x1-x2)**2 + (y1-y2)**2)

        x1 += u1 * math.cos(psi1) * dt
        x2 += u2 * math.cos(psi2) * dt

        y1 += u1 * math.sin(psi1) * dt
        y2 += u2 * math.sin(psi2) * dt
    dcpa_idx = np.argmin(dcpa_values)
    dcpa = dcpa_values[dcpa_idx]
    tcpa = dcpa_idx
    return dcpa, tcpa


def cpa_new(ship1_x, ship1_y, ship1_psi, ship1_u, ship2_x, ship2_y, ship2_psi, ship2_u, dt=1):
    x1, y1, u1, psi1 = ship1_x, ship1_y, ship1_u, ship1_psi
    x2, y2, u2, psi2 = ship2_x, ship2_y, ship2_u, ship2_psi
    dcpa_values = np.zeros(600)
    for i in range(600):
        dcpa_values[i] = np.sqrt((x1-x2)**2 + (y1-y2)**2)

        x1 += u1 * math.cos(psi1) * dt
        x2 += u2 * math.cos(psi2) * dt

        y1 += u1 * math.sin(psi1) * dt
        y2 += u2 * math.sin(psi2) * dt
    dcpa_idx = np.argmin(dcpa_values)
    dcpa = dcpa_values[dcpa_idx]
    tcpa = dcpa_idx
    return dcpa, tcpa

"""
def cpa(ownship, targetship):
        '''
            DCPA and TCPA with targetship
        '''
        #targetship_u = targetship.u * math.cos(targetship.psi)
        #targetship_v = targetship.u * math.sin(targetship.psi)

        # Distance between ownship and targetship
        D_r = math.sqrt((targetship.x - ownship.x)**2 + (targetship.y - ownship.y)**2)
    
        # Relative speed between ownship and targetship
        U_r = math.sqrt((targetship.u - ownship.u)**2 + (targetship.v - ownship.v)**2)

        # alpha_r (True bearing of the targetship)
        if (targetship.y - ownship.y >= 0) and (targetship.x - ownship.x >= 0):
            delta_alpha = 0
        elif (targetship.y - ownship.y >= 0) and (targetship.x - ownship.x < 0):
            delta_alpha = 0
        elif (targetship.y - ownship.y < 0) and (targetship.x - ownship.x < 0):
            delta_alpha = 2 * math.pi
        elif (targetship.y - ownship.y < 0) and (targetship.x - ownship.x >= 0):
            delta_alpha = 2 * math.pi
        alpha_r = math.atan2((targetship.y - ownship.y), (targetship.x - ownship.x)) + delta_alpha
        
        # chi_r (Relative course of TS (from 0 to U_r))
        if (targetship.v - ownship.v >= 0) and (targetship.u - ownship.u >= 0):
            delta_chi = 0
        elif (targetship.v - ownship.v >= 0) and (targetship.u - ownship.u < 0):
            delta_chi = 0
        elif (targetship.v - ownship.v < 0) and (targetship.u - ownship.u < 0):
            delta_chi = 2 * math.pi
        elif (targetship.v - ownship.v < 0) and (targetship.u - ownship.u >= 0):
            delta_chi = 2 * math.pi
        chi_r = math.atan2((targetship.v - ownship.v), (targetship.u - ownship.u)) + delta_chi
        
        # beta
        beta = chi_r - alpha_r - math.pi 
        
        # DCPA and TCPA
        dcpa = abs(round(D_r * math.sin(beta), 2))
        tcpa = round((D_r * math.cos(beta)) / (abs(U_r)+1), 2)    
        return dcpa, tcpa
"""


def cpa_cost(os_x, os_y, os_u, os_v, ts_x, ts_y, ts_u, ts_v):
        '''
            DCPA and TCPA with targetship
        '''
        # Distance between ownship and targetship
        D_r = math.sqrt((ts_x - os_x)**2 + (ts_y - os_y)**2)
    
        # Relative speed between ownship and targetship
        U_r = math.sqrt((ts_u - os_u)**2 + (ts_v - os_v)**2)

        # alpha_r (True bearing of the targetship)
        if (ts_y - os_y >= 0) and (ts_x - os_x >= 0):
            delta_alpha = 0
        elif (ts_y - os_y >= 0) and (ts_x - os_x < 0):
            delta_alpha = 0
        elif (ts_y- os_y < 0) and (ts_x - os_x < 0):
            delta_alpha = 2 * math.pi
        elif (ts_y - os_y < 0) and (ts_x - os_x >= 0):
            delta_alpha = 2 * math.pi
        alpha_r = math.atan2((ts_y - os_y), (ts_x - os_x)) + delta_alpha
        
        # chi_r (Relative course of TS (from 0 to U_r))
        if (ts_v - os_v >= 0) and (ts_u - os_u >= 0):
            delta_chi = 0
        elif (ts_v - os_v >= 0) and (ts_u - os_u < 0):
            delta_chi = 0
        elif (ts_v - os_v < 0) and (ts_u - os_u < 0):
            delta_chi = 2 * math.pi
        elif (ts_v - os_v < 0) and (ts_u - os_u >= 0):
            delta_chi = 2 * math.pi
        chi_r = math.atan2((ts_v - os_v), (ts_u - os_u)) + delta_chi
        
        # beta
        beta = chi_r - alpha_r - math.pi 
        
        # DCPA and TCPA
        dcpa = abs(round(D_r * math.sin(beta), 2))
        tcpa = round((D_r * math.cos(beta)) / (abs(U_r)+1), 2)
        #print("DCPA:", dcpa, " TCPA:", tcpa)
                
        return dcpa, tcpa


def colregs_rule(ship1_x, ship1_y, ship1_psi, ship1_u, ship2_x, ship2_y, ship2_psi, ship2_u):
    # RB_os_ts: Relative bearing of TS from OS
    RB_os_ts = relative_bearing(ship1_x, ship1_y, ship1_psi, ship2_x, ship2_y)
    # RB_ts_os: Relative bearing of OS from TS
    RB_ts_os = relative_bearing(ship2_x, ship2_y, ship2_psi, ship1_x, ship1_y)
    # Head on, give way
    if abs(RB_os_ts) < math.radians(13) and abs(RB_ts_os) < math.radians(13):
        rule = 'HO-GW'
    # Overtaken, stand on
    elif abs(RB_os_ts) > math.radians(112.5) and abs(RB_ts_os) < math.radians(45) and (ship2_u > (ship1_u * 1.1)):
        rule = 'ON-SO'
    # Overtaking
    elif abs(RB_ts_os) > math.radians(112.5) and abs(RB_os_ts) < math.radians(45) and (ship1_u > (ship2_u * 1.1)):
        rule = 'OG'
    # Crossing, stand on
    elif RB_os_ts > 0 and RB_os_ts < math.radians(112.5) and RB_ts_os < math.radians(10) and RB_ts_os > math.radians(-112.5):
        rule = 'CR-SO'
    # Crossing give way
    elif RB_os_ts < math.radians(10) and RB_os_ts > math.radians(-112.5) and RB_ts_os > 0 and RB_ts_os < math.radians(112.5):
        rule = 'CR-GW'
    else:
        rule = 'None'
    return rule


def cri(dcpa, tcpa):
    d_min = 750
    t_min = 600
    a = 0.02
    b = 0.01
    if dcpa <= d_min and tcpa <= t_min:
        dyn_risk = (a * dcpa) ** 2 + (b * tcpa) ** 2
    else:
        dyn_risk = 0.0
    return dyn_risk


def publish_states(t, ship, msg, publisher):
    """
        Publishing current ship states to the server.
        t : time step
        ship: Ship class
        msg: Ship state message object (ship_states.msg)
        publisher: Publisher object (rospy.Publisher)
    """
    msg.data = json.dumps([t, ship.id, round(ship.x, 2), round(ship.y, 2), round(ship.psi, 2), round(ship.u, 2), ship.r18, ship.wp])
    publisher.publish(msg)


def create_animation_data(ownship, ts_list):
    anim_length = min(len(ownship.x_log), min([len(each.x_log) for each in ts_list]))
    df_anim = pd.DataFrame()
    df_anim[f"{ownship.id}_x"] = ownship.x_log[1:anim_length]
    df_anim[f"{ownship.id}_y"] = ownship.y_log[1:anim_length]
    df_anim[f"{ownship.id}_u"] = ownship.u_log[1:anim_length]
    df_anim[f"{ownship.id}_psi"] = ownship.psi_log[1:anim_length]
    df_anim[f"{ownship.id}_wp"] = ownship.wp_log[1:anim_length]
    for ts in ts_list:
        df_anim[f'{ts.id}_x'] = ts.x_log[1:anim_length]
        df_anim[f'{ts.id}_y'] = ts.y_log[1:anim_length]
        df_anim[f'{ts.id}_u'] = ts.u_log[1:anim_length]
        df_anim[f'{ts.id}_psi'] = ts.psi_log[1:anim_length]
        df_anim[f'{ts.id}_wp'] = ts.wp_log[1:anim_length]
    df_anim["time"] = df_anim.index
    anim_length = len(df_anim)
    return df_anim, anim_length

