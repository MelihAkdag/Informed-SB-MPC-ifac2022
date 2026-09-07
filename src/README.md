# `src/` — source overview

This folder holds the ROS nodes and the supporting modules for the Informed SB-MPC
collision-avoidance prototype. For build/run instructions and the overall design, see the
[top-level README](../README.md).

## Nodes (run as ROS processes)

| File | Role |
| --- | --- |
| `server.py` | Central node. Collects each ship's broadcast state, fuses it into a shared situational-awareness picture, and re-broadcasts it. |
| `ship1.py`, `ship2.py`, `ship3.py` | Ship agents. Each follows its waypoints, runs the COLAV decision layer, and negotiates manoeuvres with other ships. `ship1.py` also renders the plots/animation into `sim_results/`. |

The launch file `mass_sim.launch` starts `server` + `ship1` + `ship2` together.

## Supporting modules (imported, not run directly)

| File | Purpose |
| --- | --- |
| `config.py` | All simulation settings: time step, duration, prediction horizon, and the ships' initial states / scenario (edit this to change the encounter). |
| `colav.py` | Top-level collision-avoidance decision maker; chooses between no action, reactive avoidance, and cooperative negotiation. |
| `sbmpc.py` | Informed Scenario-Based MPC — reactive avoidance. |
| `dsbmpc.py` | Distributed SB-MPC — cooperative, negotiated avoidance (handles the `DirectMessage` request/response protocol). |
| `ship_model.py` | Vessel kinematics and Line-of-Sight (LOS) waypoint guidance. |
| `map_polygons.py` | Land polygons used for grounding checks. |
| `utility.py` | Shared helpers: angle/distance math and ROS publish/subscribe wrappers. |

## Where to start

- **Change the scenario** → edit `config.py` (ship trajectories, headings, speeds).
- **Understand the decision logic** → read `colav.py` (see the "Logical flow" section in the
  top-level README).
- **Run it** → follow the "Build & run" section in the [top-level README](../README.md).
