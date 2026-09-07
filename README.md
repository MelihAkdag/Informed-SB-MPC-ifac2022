# Informed SB-MPC (IFAC 2022) — Collaborative Ship Collision Avoidance

Prototype implementation of **Informed Scenario-Based Model Predictive Control (SB-MPC)** with
**collaborative (negotiated) collision avoidance** for autonomous surface ships, built on
**ROS 1**. Multiple ship agents broadcast their states, and when an encounter arises they
negotiate COLREGs-compliant evasive manoeuvres via direct ship-to-ship messaging.

This is the simplified prototype (2–3 ships, abstract head-on / crossing encounters) that
accompanies the paper below. It was extracted from an earlier `src copy/` folder of a larger
research repository.

---

## Related publication

> M. Akdağ, T. I. Fossen, and T. A. Johansen,
> **"Collaborative Collision Avoidance for Autonomous Ships Using Informed Scenario-Based
> Model Predictive Control,"** *IFAC-PapersOnLine*, vol. 55, no. 31, pp. 249–256, 2022.
> DOI: [10.1016/j.ifacol.2022.10.439](https://doi.org/10.1016/j.ifacol.2022.10.439)
> (IFAC CAMS 2022; funded by the Research Council of Norway, grant 308839.)

---

## What it does

- A central **`server`** node fuses each ship's broadcast state into a shared situational
  awareness picture and re-broadcasts it.
- Each **`ship`** node (`ship1`–`ship3`) follows waypoints and runs the COLAV decision layer:
  - **Informed SB-MPC** — reactive scenario-based avoidance informed by other ships' intent.
  - **Distributed SB-MPC** — cooperative avoidance negotiated between ships.
  - **Negotiation protocol** — ships exchange proposed manoeuvres via the `DirectMessage`
    ROS service until they agree.

**State vector convention** (JSON-encoded on the wire):
```
[t, id, x, y, psi, u, v, r, r18, wps]
```

---

## Repository layout

```
CMakeLists.txt          catkin build config (declares msgs/srvs)
package.xml             ROS package manifest
msg/                    ROS messages: ship_states, bcast_sitaw, rtx_msg
srv/                    ROS services: DirectMessage, RTXmsg
src/
  server.py             central situational-awareness node
  ship1.py ship2.py ship3.py   ship agent nodes
  config.py             sim settings + head-on / crossing scenarios
  ship_model.py         vessel kinematics + LOS guidance
  sbmpc.py              Scenario-Based MPC
  dsbmpc.py             Distributed SB-MPC + negotiation (answer_request)
  colav.py              top-level COLAV decision maker
  map_polygons.py       land polygons for grounding checks
  utility.py            helpers (angles, distances, ROS pub/sub)
  mass_sim.launch       launch file: server + ships
```

> The ROS package is still named `ros_mas_test` (the Python code imports
> `ros_mas_test.msg` / `ros_mas_test.srv`). Rename the package only if you also update those
> imports and the manifest.

---

## Dependencies

**ROS 1** (rospy, catkin) — `rospy` and the `ros_mas_test.msg` / `ros_mas_test.srv` modules
come from the ROS install and the catkin build, **not** pip.

**Python (pip):** see [`requirements.txt`](requirements.txt) — `numpy`, `pandas`,
`matplotlib`, `shapely`.

```bash
pip install -r requirements.txt
```

---

## Build & run (ROS 1 / catkin)

```bash
# 1) Place under a catkin workspace, e.g. ~/catkin_ws/src/ros_mas_test
# 2) Python deps
pip install -r requirements.txt
# 3) Build
cd ~/catkin_ws
catkin_make            # or: catkin build
source devel/setup.bash
# 4) Master
roscore
# 5) Launch server + ships
roslaunch ros_mas_test mass_sim.launch
```

Individual nodes:

```bash
rosrun ros_mas_test server.py
rosrun ros_mas_test ship1.py
rosrun ros_mas_test ship2.py
```

---

## Glossary

- **COLREGs** — International Regulations for Preventing Collisions at Sea.
- **SB-MPC** — Scenario-Based Model Predictive Control.
- **Informed SB-MPC** — SB-MPC informed by other vessels' communicated intentions.
- **Distributed SB-MPC** — cooperative, negotiated variant.
- **LOS** — Line-of-Sight waypoint guidance.
- **DCPA / TCPA** — Distance / Time to Closest Point of Approach.
