#!/usr/bin/env python

import rospy
import numpy as np
import json
from ros_mas_test.msg import ship_states, bcast_sitaw
from rospy.numpy_msg import numpy_msg
from ship_model import *
from utility import *
from config import *
from sbmpc import *
from dsbmpc import *
from colav import *


def sitaw_callback(rx_data):
    global all_states
    all_states = json.loads(rx_data.data)


def response_direct_msg(req):
    """
        Direct messaging (ROS service server)
    """
    # Filter the sender targetship
    ts = [ts for ts in ts_list if ts.id == req.sender][0]
    # Update last proposed control actions of targetship
    ts.ppsd_psi = req.sender_psi_u[0]
    ts.ppsd_u = req.sender_psi_u[1]
    ts.msg_last = req.header

    # If received message type is proposal or intention query:
    if req.header != "ACK": 
        ts.msg_count += 1
        # Prepare an answer for the request than print and send it
        header, os_psi, os_u, ts_psi, ts_u = dsbmpc.answer_request(req, ownship, ts, ts_list)
        print("\nheader: ", header, "\nsender: ", ownship.id, "\nreceiver: ", req.sender, "\nsender_psi_u: ", [os_psi, os_u], "\nreceiver_psi_u: ", [ts_psi, ts_u])
        return DirectMessageResponse(header, ownship.id, req.sender, [os_psi, os_u], [ts_psi, ts_u])

    # If received message type is acknowledgement:
    elif req.header == "ACK":
        # Apply acknowledged control actions
        ownship.set_opt_ctrl(req.receiver_psi_u[0], req.receiver_psi_u[1])
        # Erase message counter of the targetship to start a new negotiation later
        ts.msg_count = 0 
        return


def direct_msg_response(ownship):
    """
        Direct messaging (ROS service server). Initialize direct messaging
    """
    s = rospy.Service(f'direct_msg_{ownship.id}', DirectMessage, response_direct_msg)
    return s


# Defining initial states and creating the ownship class
ownship = DynamicModel(ship3_init_states)
sbmpc = SBMPC()
dsbmpc = DSBMPC()
ts_id_list = []
all_states = {}
ts_list = []

# create the publisher
pub_states = rospy.Publisher('ship_state_topic', ship_states, queue_size=10)
state_msg = ship_states()

# initialiaze the node
rospy.init_node('ship_node', anonymous=True)
rate = rospy.Rate(rate_var)

# starting direct message service server
s = direct_msg_response(ownship)

#while not rospy.is_shutdown():
while t <= T_sim:
    # publish own states [t, id, x, y, psi, U, colregs18, [trajectory]]
    publish_states(t, ownship, state_msg, pub_states)

    # receiving sitaw data
    rospy.Subscriber('bcast_states_topic', bcast_sitaw, sitaw_callback)
    print(all_states)
    
    # Creating target ships from received data
    ts_id_list, ts_list = create_ts_data(ts_id_list, ts_list, all_states, ownship, dt)

    # move ship
    ownship.move(dt)

    # Find best offset course and speed values
    colav(t, ownship, ts_list, sbmpc, dsbmpc)
    #ownship.set_opt_ctrl(0, 1)

    rate.sleep()
    t = t+1

# Wait for manual termination/shutdown
s.spin()
