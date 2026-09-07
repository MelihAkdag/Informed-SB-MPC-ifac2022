#!/usr/bin/env python

import rospy
import json
from informed_sbmpc.msg import ship_states, bcast_sitaw
from rospy.numpy_msg import numpy_msg
import numpy as np
from config import *


def ship_state_callback(ship_state_msg):
    global all_states
    # ship_info => [t, id, x, y, psi, U, colregs18, [trajectory]]
    ship_info = json.loads(ship_state_msg.data)
    all_states[ship_info[1]] = ship_info

# initializing the server node
rospy.init_node('server_node', anonymous=True)

# publisher for broadcasting total states
pub = rospy.Publisher('bcast_states_topic', numpy_msg(bcast_sitaw), queue_size=10)

# setting loop rate
rate = rospy.Rate(rate_var)

while not rospy.is_shutdown():
    # receiving individual ship states and combining them
    rospy.Subscriber('ship_state_topic', ship_states, ship_state_callback)

    # broadcasting total situational awareness data
    #temp_list = [all_states[key][0] for key in all_states.keys()]
    #if all(x==temp_list[0] for x in temp_list):

    print(all_states)
    bcast = bcast_sitaw()
    bcast.data = json.dumps(all_states)
    pub.publish(bcast)
    rate.sleep()
