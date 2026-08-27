# 09 — Space ROS and Canadarm2

## What this is

Space ROS (ROS 2 nodes over DDS) vs traditional NASA cFS (single executive). The **Canadarm2** demo was run in Gazebo with MoveIt (`/open_arm`, `/close_arm`, `/random_arm`). The same arm meshes/URDF were imported into Isaac Sim. This is a **manipulation / motion-planning** stack, not an orbit propagator: GNC/ADCS stay with GMAT, FreeFlyer, Basilisk, or NASA 42.

## Official example this started from

**Space ROS** `demos/canadarm2` (Gazebo: `canadarm_gazebo`, `canadarm_description`, `canadarm_moveit_config`). This benchmark did **not** fork that demo into this repo. It records a successful run and the Isaac Sim import.

## What changed

- Executed the official Canadarm2 Gazebo + RViz/MoveIt flow and captured screenshots.
- Imported the same arm description into Isaac Sim 6.x for a future contact/perception path.
- Architecture note: global planning (MoveIt task) and local collision-aware refinement as separate ROS 2 nodes.

## How to run

Use your local Space ROS workspace (not shipped here):

```bash
cd /path/to/space_ros_ws/demos/canadarm2
# follow upstream run.sh / ros2 launch canadarm_moveit_config demo.launch.py
```

## Results in this repo

`results/spaceros_canadarm_gazebo.png`, `spaceros_canadarm_moveit_rviz.png`, `spaceros_nodes_architecture.png`, run manifests.
