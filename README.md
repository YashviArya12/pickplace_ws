# Vision-Guided Pick-and-Place (ROS 2 + Gazebo + MoveIt 2)
![Pick-and-place sequence](docs/pickplace_demo.gif)

*UR5e performing a pick-and-place motion sequence in Gazebo: above the cube, descend, lift, move to the drop pad, descend, lift, and return to the ready pose. Planned with MoveIt 2 and driven by a Python node using pymoveit2.*
## Status

| Stage | State |
|---|---|
| Simulation cell (UR5e, table, 3 objects, overhead RGB-D camera) | Done |
| MoveIt 2 motion planning with Gazebo execution | Done |
| Pick-and-place motion sequence (Cartesian descend/lift) | Done |
| YOLOv8 object detection on synthetic data | Dataset Generated, Training next |
| Camera-to-robot 3D localization | Planned |
| Gripper and grasp execution | Planned |
| Evaluation (grasp success rate, detection mAP) | Planned |

Object positions in the sequence are currently fixed values. The next steps replace them with positions detected from the camera.

## Run it

```bash
# Terminal 1: simulation
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e \
  world_file:=$HOME/pickplace_ws/src/pickplace_sim/worlds/pickplace.sdf launch_rviz:=false

# Terminal 2: MoveIt 2
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_sim_time:=true

# Terminal 3: pick-and-place sequence
ros2 run pickplace_manipulation pick_place_sequence --ros-args -p use_sim_time:=true
```

Requires ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2 and [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) cloned into `src/`.
![Arm moving above the cube](docs/hover_demo.gif)

*UR5e planning and moving above the red cube using MoveIt 2 and a Python node.*

**Task:** Pick 3 object types (cube, cylinder, box) from a table and place them in a bin.
**Metrics:** grasp success rate, detection mAP, cycle time, pose error (mm).
**v1 constraint:** top-down grasps only.
![Overhead camera view](docs/rviz_camera_3obj.png)
![UR5e in Gazebo](docs/gazebo_ur5e_scene.png)
