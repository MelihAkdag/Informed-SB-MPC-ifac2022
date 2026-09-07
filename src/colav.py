from ship_model import *
from utility import *
from map_polygons import *
from config import *
from sbmpc import *
from dsbmpc import *
from random import randint


def colav(t, ownship, ts_list, sbmpc, dsbmpc, initial_reaction=False):
    """
        Decision making for the collision avoidance algorithm. 
        The function combines Informed SB-MPC, Distributed SB-MPC, and forms the negotiation protocol.
    """
    range_thr_1 = 4000
    range_thr_2 = 1800
    dcpa_thr = 1000
    tcpa_thr = 100

    distances = np.zeros(len(ts_list))
    dcpa_list = np.zeros(len(ts_list))
    tcpa_list = np.zeros(len(ts_list))
    msg_list = []
    for i in range(len(ts_list)):
        distances[i] = ts_list[i].distance
        dcpa_list[i] = ts_list[i].dcpa
        tcpa_list[i] = ts_list[i].tcpa
        msg_list.append(ts_list[i].msg_last)
    # Check if there is any risk:
    if t > 1 and (all(dist >= range_thr_1 for dist in distances) or all(dcpa >= dcpa_thr for dcpa in dcpa_list)):
        print(ownship.id, "NO COLAV")
        ownship.chi_opt = 0.0
        ownship.u_opt = 1.0
        return

    # If there exists a risk:
    elif t > 1 and (any(dist < range_thr_1 for dist in distances) or any(dcpa < dcpa_thr for dcpa in dcpa_list)):
        # There is not enough time for collaboration:
        if any(dist < range_thr_2 for dist in distances): #or any(tcpa < tcpa_thr for tcpa in tcpa_list):
            # Informed SBMPC finds best offset course and speed values
            u_os_best, chi_os_best = sbmpc.get_optimal_ctrl_offset(ownship, ts_list)
            ownship.set_opt_ctrl(chi_os_best, u_os_best)
            print(ownship.id, "INFORMED SBMPC IS ACTIVE")

        # There is time for collaboration:
        elif t > 1 and (all(dist >= range_thr_2 for dist in distances) or all(tcpa >= tcpa_thr for tcpa in tcpa_list)):
            # Previously no message received
            if initial_reaction and not any(each for each in msg_list):
                for ts in ts_list:
                    # If ownship is in stand on role than ask intention
                    if ts.colregs_role == "CR-SO" or ts.colregs_role == "ON-SO" or ts.colregs_role == "HO-GW":
                        msg_id = randint(0, 1000)
                        dsbmpc.direct_msg_request(ts.id, "INT", ownship.id, ts.id, [ownship.psi, ownship.u], [ts.psi, ts.u], msg_id)
                        dsbmpc.msg_last = "INT"
                        ts.msg_last = "PPS"
                    # If ownship is in give-way role calculate optimal action and propose
                    elif ts.colregs_role != "CR-SO" or ts.colregs_role != "ON-SO":
                        cost, u_os_best, chi_os_best = dsbmpc.get_optimal_ctrl_offset(ownship, ts, ts_list)
                        os_u = ownship.u_d * u_os_best
                        os_psi = wrap_to_2pi(chi_os_best + ownship.chi_d)
                        dsbmpc.ppsd_chi = chi_os_best
                        dsbmpc.ppsd_u = u_os_best
                        dsbmpc.cost_last = cost
                        dsbmpc.msg_last = "PPS"
                        ts.msg_last = "PPS"
                        msg_id = randint(0,1000)
                        resp = dsbmpc.direct_msg_request(ts.id, "PPS", ownship.id, ts.id, [os_psi, os_u], [ts.psi, ts.u], msg_id)
                        # If response message is an acknowledgement then apply new control actions
                        try:
                            if resp.header == "ACK":
                                # Apply acknowledged control actions
                                ownship.set_opt_ctrl(dsbmpc.ppsd_chi, dsbmpc.ppsd_u)
                                # Erase message counter of the targetship to start a new negotiation later
                                ts.msg_count = 0
                                ts.msg_last = "ACK"
                        except: 
                            print("Couldn't read response from ", ts.id)
                            
            # Previously at least one message received
            elif initial_reaction and any(each for each in msg_list):
                for ts in ts_list:
                    if ts.msg_last == "PPS":
                        msg_id = randint(0, 1000) 
                        
                        # Calculate cost1 with ownship's current psi,u and proposed optimal actions from targetship
                        ts.trajectory_prediction(psi=ts.ppsd_psi, U=ts.ppsd_u, pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step)
                        cost1, u_os_best1, chi_os_best1 = dsbmpc.get_optimal_ctrl_offset(ownship, ts, ts_list, selfish_behave=True)
                        
                        # Calculate cost2 with ownship's optimal psi,u and proposed optimal actions from targetship
                        ts.trajectory_prediction(psi=ts.ppsd_psi, U=ts.ppsd_u, pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step)
                        cost2, u_os_best2, chi_os_best2 = dsbmpc.get_optimal_ctrl_offset(ownship, ts, ts_list, selfish_behave=False)
                        
                        # Calculate cost3 with ownship's optimal psi,u and current actions of targetship
                        ts.trajectory_prediction(psi=ts.psi, U=ts.u, pred_hor=dsbmpc_pred_hor, time_step=dsbmpc_t_step)
                        cost3, u_os_best3, chi_os_best3 = dsbmpc.get_optimal_ctrl_offset(ownship, ts, ts_list, selfish_behave=False)

                        print("\n", ownship.id, "COST VALUES: ", cost1, cost2, cost3)
                        print(ownship.id, "CHI: ", chi_os_best1, chi_os_best2, chi_os_best3)
                        print(ownship.id, "U: ", u_os_best1, u_os_best2, u_os_best3, "\n")
                        
                        # Compare three costs
                        if (cost1 <= cost2 and cost1 <= cost3):
                            os_u = ownship.u
                            os_psi = ownship.psi
                            dsbmpc.ppsd_chi = 0.0
                            dsbmpc.ppsd_u = 1.0
                            dsbmpc.cost_last = cost1
                            dsbmpc.msg_last = "ACK"
                            ts.msg_count = 0
                            ts.msg_last = "ACK"
                            resp = dsbmpc.direct_msg_request(ts.id, "ACK", ownship.id, ts.id, [os_psi, os_u], [ts.ppsd_psi, ts.ppsd_u], msg_id)
                            # If response message is an acknowledgement then apply new control actions
                            if resp.header == "ACK":
                                # Apply acknowledged control actions
                                ownship.set_opt_ctrl(dsbmpc.ppsd_chi, dsbmpc.ppsd_u)
                                # Erase message counter of the targetship to start a new negotiation later
                                ts.msg_count = 0
                                ts.msg_last = "ACK"

                        elif cost2 < cost1 and cost2 <= cost3:
                            os_u = ownship.u_d * u_os_best2
                            os_psi = wrap_to_pi(chi_os_best2 + ownship.chi_d)
                            # Only propose if new cost is smaller than the previous cost or control actions are different
                            if cost2 < dsbmpc.cost_last or (os_u != ownship.u and os_psi != ownship.psi):
                                resp = dsbmpc.direct_msg_request(ts.id, "PPS", ownship.id, ts.id, [os_psi, os_u], [ts.ppsd_psi, ts.ppsd_u], msg_id)
                                dsbmpc.ppsd_chi = chi_os_best2
                                dsbmpc.ppsd_u = u_os_best2
                                dsbmpc.cost_last = cost2
                                dsbmpc.msg_last = "PPS"
                                ts.msg_count += 1
                                ts.msg_last = "PPS"
                                # If response message is an acknowledgement then apply new control actions
                                if resp.header == "ACK":
                                    # Apply acknowledged control actions
                                    ownship.set_opt_ctrl(dsbmpc.ppsd_chi, dsbmpc.ppsd_u)
                                    # Erase message counter of the targetship to start a new negotiation later
                                    ts.msg_count = 0
                                    ts.msg_last = "ACK"

                        elif cost3 < cost1 and cost3 < cost2:
                            os_u = ownship.u_d * u_os_best3
                            os_psi = wrap_to_pi(chi_os_best3 + ownship.chi_d)
                            # Only propose if new cost is smaller than the previous cost or control actions are different
                            if cost3 < dsbmpc.cost_last or (os_u != ownship.u and os_psi != ownship.psi):
                                resp = dsbmpc.direct_msg_request(ts.id, "PPS", ownship.id, ts.id, [os_psi, os_u], [ts.psi, ts.u], msg_id)
                                dsbmpc.ppsd_chi = chi_os_best3
                                dsbmpc.ppsd_u = u_os_best3
                                dsbmpc.cost_last = cost3
                                dsbmpc.msg_last = "PPS"
                                ts.msg_count += 1
                                ts.msg_last = "PPS"
                                # If response message is an acknowledgement then apply new control actions
                                if resp.header == "ACK":
                                    # Apply acknowledged control actions
                                    ownship.set_opt_ctrl(dsbmpc.ppsd_chi, dsbmpc.ppsd_u)
                                    # Erase message counter of the targetship to start a new negotiation later
                                    ts.msg_count = 0
                                    ts.msg_last = "ACK"

                    elif ts.msg_last == "ACK":                        
                        print("LAST MESSAGE ACKNOWLEDGE.")
            

