#!/usr/bin/env python

from ftplib import all_errors
import json
from queue import Empty
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from ros_mas_test.msg import ship_states, bcast_sitaw
from rospy.numpy_msg import numpy_msg
from ship_model import *
from utility import *
from map_polygons import *
from config import *
from sbmpc import *
from dsbmpc import *
from colav import *
from random import randint


def sitaw_callback(rx_data):
    global all_states
    all_states = json.loads(rx_data.data)


def response_direct_msg(req):
    """
        Direct messaging (ROS service server)
    """
    print("\n", req, "\n")
    # Filter the sender targetship
    ts = [ts for ts in ts_list if ts.id == req.sender][0]
    # Update last proposed control actions of targetship
    ts.ppsd_psi = req.sender_psi_u[0]
    ts.ppsd_u = req.sender_psi_u[1]
    
    # Prepare and send an answer for the request 
    header, os_psi, os_u, ts_psi, ts_u, msg_id = dsbmpc.answer_request(req, ownship, ts, ts_list)
    return DirectMessageResponse(header, ownship.id, req.sender, [os_psi, os_u], [ts_psi, ts_u], msg_id)


def direct_msg_response(ownship):
    """
        Direct messaging (ROS service server). Initialize direct messaging
    """
    s = rospy.Service(f'direct_msg_{ownship.id}', DirectMessage, response_direct_msg)
    return s



# Defining initial states and creating the ownship class [t, id, x, y, psi, U, colregs18, [trajectory]]
ownship = DynamicModel(ship1_init_states)
sbmpc = SBMPC()
dsbmpc = DSBMPC(ownship)
ts_id_list = []
ts_list = []
all_states = {}
dist_log = np.zeros((2, T_sim))

# create the publisher and subscribe to the server
pub_states = rospy.Publisher('ship_state_topic', ship_states, queue_size=10)
rospy.Subscriber('bcast_states_topic', bcast_sitaw, sitaw_callback)
state_msg = ship_states()

# initialiaze the ship node
rospy.init_node('ship_node', anonymous=True)
rate = rospy.Rate(rate_var)

# starting direct message service server
s = direct_msg_response(ownship)

#while not rospy.is_shutdown():
while t <= T_sim:
    # publish own states [t, id, x, y, psi, U, colregs18, [trajectory]]
    publish_states(t, ownship, state_msg, pub_states)

    # printing sitaw data
    print(all_states)
    
    # Creating target ships from received data
    ts_id_list, ts_list = create_ts_data(ts_id_list, ts_list, all_states, ownship, dt)

    # move ship
    ownship.move(dt)

    # Find best offset course and speed values
    colav(t, ownship, ts_list, sbmpc, dsbmpc, initial_reaction=True)
    #ownship.set_opt_ctrl(0, 1)

    # Distance log for graph
    for i in range(len(ts_list)):
        dist_log[i, t-1] = distance(ownship.x, ownship.y, ts_list[i].x, ts_list[i].y)

    rate.sleep()
    t = t+1

# Wait for manual termination/shutdown
s.spin()

#############################
# VISUALIZATION
#############################
df_anim, anim_length = create_animation_data(ownship, ts_list)
print(df_anim)

ship_markers = []
past_trajectory = []
ship_wps = []
headings = []
speeds = []
colors = ['blue', 'purple', 'darkolivegreen', 'teal', 'darkorange', 'saddlebrown']
fig = plt.figure(figsize=(20, 13))
gs = gridspec.GridSpec(2, 3)
ax1 = fig.add_subplot(gs[:, 0:2])
ax1.grid()
plt.xlim(-2500.0, 2500.0)
plt.ylim(-2500.0, 2500.0)
# Drawing map polygons:
for geom in poly_full.geoms:
    xs, ys = geom.exterior.xy
    ax1.fill(xs, ys, c='gray', alpha=0.8, fc='wheat')
ax2 = fig.add_subplot(gs[0, 2])
ax2.grid()
plt.title("Ship headings over time")
plt.xlabel("Time")
plt.ylabel("Heading [Radians]")
plt.xlim(0, T_sim)
plt.ylim(-4, 4)
ax3 = fig.add_subplot(gs[1, 2])
ax3.grid()
plt.title("Ship speed changes over time")
plt.xlabel("Time")
plt.ylabel("Speed")
plt.xlim(0, T_sim)
plt.ylim(-20, 20)

for j in range(len(ts_list) + 1):
    ship_markers.append(ax1.plot([], [], 'o', mfc='none', markersize=15, c=colors[j])[0])
    past_trajectory.append(ax1.plot([], [], c=colors[j], alpha=0.8)[0])
    ship_wps.append(ax1.plot([], [], "--", c=colors[j], alpha=0.5)[0])
    headings.append(ax2.plot([], [], c=colors[j])[0])
    speeds.append(ax3.plot([], [], c=colors[j])[0])

def init_ani():
    pass

def animate(i):
    for n in range(len(ship_markers)):
        x_vals = df_anim[f"ship_{n+1}_x"].loc[i]
        y_vals = df_anim[f"ship_{n+1}_y"].loc[i]
        ship_markers[n].set_xdata(x_vals)
        ship_markers[n].set_ydata(y_vals)
        past_trajectory[n].set_xdata(df_anim[f"ship_{n+1}_x"][:i])
        past_trajectory[n].set_ydata(df_anim[f"ship_{n+1}_y"][:i])
        try:
            wps = df_anim[f"ship_{n+1}_wp"].loc[i]
            wp_x_vals = [wp[0] for wp in wps]
            wp_y_vals = [wp[1] for wp in wps]
            ship_wps[n].set_xdata(wp_x_vals)
            ship_wps[n].set_ydata(wp_y_vals)
        except:
            ship_wps[n].set_xdata([])
            ship_wps[n].set_ydata([])
        headings[n].set_xdata(df_anim['time'][:i])
        headings[n].set_ydata(df_anim[f"ship_{n + 1}_psi"][:i])
        speeds[n].set_xdata(df_anim['time'][:i])
        speeds[n].set_ydata(df_anim[f"ship_{n + 1}_u"][:i])

ani = animation.FuncAnimation(fig, animate, init_func=init_ani,
                              frames=anim_length, interval=50, blit=False)
ani.save("/home/parallels/catkin_ws/src/ros_mas_test/src/sim_results/animation.mp4")
plt.show()


"""
# Heading graph
plt.figure(figsize=(7, 7))
for j in range(len(ts_list) + 1):
    plt.plot(df_anim['time'][:], df_anim[f"ship_{j + 1}_psi"], c=colors[j])
plt.grid()
plt.ylim(-4, 4)
plt.title("Ship headings over time")
plt.xlabel("Time")
plt.ylabel("Heading [Radians]")
plt.savefig('/home/parallels/catkin_ws/src/ros_mas_test/src/sim_results/Headings.png', dpi=300)

# Speed graph
plt.figure(figsize=(7, 7))
for j in range(len(ts_list) + 1):
    plt.plot(df_anim['time'][:], df_anim[f"ship_{j + 1}_u"], c=colors[j])
plt.grid()
plt.ylim(-5, 20)
plt.title("Speed changes over time")
plt.xlabel("Time")
plt.ylabel("Speed")
plt.savefig('/home/parallels/catkin_ws/src/ros_mas_test/src/sim_results/Speeds.png', dpi=300)

# Distance graph
plt.figure(figsize=(7, 7))
for j in range(len(ts_list)):
    plt.plot(df_anim['time'][:], dist_log[j, :], c=colors[j+1])
#plt.axhline(y=300, color='k', linestyle='--')
plt.grid()
plt.title('Distance between ships over time')
plt.xlabel('Time')
plt.ylabel('Distance')
plt.savefig('/home/parallels/catkin_ws/src/ros_mas_test/src/sim_results/Distance.png', dpi=300)
"""