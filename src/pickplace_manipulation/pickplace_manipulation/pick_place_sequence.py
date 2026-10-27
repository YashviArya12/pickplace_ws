#!/usr/bin/env python3
"""Pick-and-place motion sequence for the UR5e (no gripper yet)."""
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pymoveit2 import MoveIt2

UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
READY = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
TOP_DOWN = [1.0, 0.0, 0.0, 0.0]  # x, y, z, w: tool points down

# Positions in metres, in the robot base frame. Later these come from the camera.
PICK_XY = (0.5, 0.1)    # red cube
PLACE_XY = (0.7, -0.3)  # drop pad
GRASP_Z = 0.12          # tool height at grasp (room for a future gripper)
HOVER_Z = 0.28          # tool height when travelling


def main():
    rclpy.init()
    node = Node("pick_place_sequence")
    log = node.get_logger()

    moveit2 = MoveIt2(
        node=node,
        joint_names=UR_JOINTS,
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="ur_manipulator",
        callback_group=ReentrantCallbackGroup(),
    )
    moveit2.max_velocity = 0.3
    moveit2.max_acceleration = 0.3

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    time.sleep(1.0)

    def go(name, xy, z, cartesian=False):
        log.info(name)
        moveit2.move_to_pose(
            position=[xy[0], xy[1], z],
            quat_xyzw=TOP_DOWN,
            cartesian=cartesian,
        )
        moveit2.wait_until_executed()

    go("1/6 above cube", PICK_XY, HOVER_Z)
    go("2/6 descend", PICK_XY, GRASP_Z, cartesian=True)
    time.sleep(0.5)  # gripper close goes here later
    go("3/6 lift", PICK_XY, HOVER_Z, cartesian=True)
    go("4/6 above pad", PLACE_XY, HOVER_Z)
    go("5/6 descend", PLACE_XY, GRASP_Z, cartesian=True)
    time.sleep(0.5)  # gripper open goes here later
    go("6/6 lift", PLACE_XY, HOVER_Z, cartesian=True)

    log.info("returning to ready pose")
    moveit2.move_to_configuration(READY)
    moveit2.wait_until_executed()
    log.info("Done")
    rclpy.shutdown()


if __name__ == "__main__":
    main()