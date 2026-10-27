#!/usr/bin/env python3
"""Move the UR5e tool to a top-down pose above the red cube."""
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

CUBE_XYZ = [0.5, 0.1, 0.025]          # matches cube_red in pickplace.sdf
HOVER = 0.15                          # metres above the cube centre
TOP_DOWN_QUAT = [1.0, 0.0, 0.0, 0.0]  # x, y, z, w: tool points down


def main():
    rclpy.init()
    node = Node("move_above_cube")

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

    target = [CUBE_XYZ[0], CUBE_XYZ[1], CUBE_XYZ[2] + HOVER]
    node.get_logger().info(f"Moving to {target}")
    moveit2.move_to_pose(position=target, quat_xyzw=TOP_DOWN_QUAT)
    moveit2.wait_until_executed()
    node.get_logger().info("Done")

    rclpy.shutdown()


if __name__ == "__main__":
    main()