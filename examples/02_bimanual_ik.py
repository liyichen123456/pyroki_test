"""Bimanual IK

Same as 01_basic_ik.py, but with two end effectors!
"""

import time
import viser
from robot_descriptions.loaders.yourdfpy import load_robot_description
import numpy as np

import pyroki as pk
from viser.extras import ViserUrdf
import pyroki_snippets as pks
import yourdfpy

def main():
    """Main function for bimanual IK."""

    # urdf = load_robot_description("yumi_description")
    # target_link_names = ["yumi_link_7_r", "yumi_link_7_l"]

    # dual_xarm7
    # urdf = yourdfpy.URDF.load(
    #     "../assert/xarm7/dual_xarm7.urdf",
    #     mesh_dir="../assert/xarm7"
    # )
    # target_link_names = ["right_link_eef", "left_link_eef"]

    # openarm_bimanual
    urdf = yourdfpy.URDF.load(
        "/home/ps/ros2_ws/openarm_bimanual.urdf",
        mesh_dir="/home/ps/ros2_ws"
    )
    target_link_names = ["openarm_right_hand", "openarm_left_hand"] # 需要移动urdf到阳间位置

    # g1_dual_arm
    urdf = yourdfpy.URDF.load(
        "../thirdparty/unitree_rl_gym/resources/robots/g1_description/g1_dual_arm.urdf",
        mesh_dir="../thirdparty/unitree_rl_gym/resources/robots/g1_description"
    )
    target_link_names = ["right_rubber_hand", "left_rubber_hand"]
    
    # Create robot.
    robot = pk.Robot.from_urdf(urdf)

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)

    # urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    robot_base = server.scene.add_frame("/robot")
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/robot")

    base_offset = np.array([0, 0, -0.1])
    robot_base.position = base_offset

    # Create interactive controller with initial position.
    # for g1_dual_arm ik_target.wxyz = (1,0,0,0) is good, for openarm_bimanual ik_target.wxyz = (0,0,1,0) is good.
    ik_target_0 = server.scene.add_transform_controls(
        "/ik_target_0", scale=0.2, position=(0.41, -0.3, 0.56), wxyz=(1,0,0,0)
    )
    ik_target_1 = server.scene.add_transform_controls(
        "/ik_target_1", scale=0.2, position=(0.41, 0.3, 0.56), wxyz=(1,0,0,0)
    )
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    target0_pos = server.gui.add_text("target0 position", initial_value=str(np.round(ik_target_0.position - base_offset, 3)))
    target0_rot = server.gui.add_text("target0 quaternion", initial_value=str(np.round(ik_target_0.wxyz, 3)))

    target1_pos = server.gui.add_text("target1 position", initial_value=str(np.round(ik_target_1.position - base_offset, 3)))
    target1_rot = server.gui.add_text("target1 quaternion", initial_value=str(np.round(ik_target_1.wxyz, 3)))

    while True:
        # Solve IK.
        start_time = time.time()
        solution = pks.solve_ik_with_multiple_targets(
            robot=robot,
            target_link_names=target_link_names,
            target_positions=np.array([ik_target_0.position - base_offset, ik_target_1.position - base_offset]),
            # target_positions=np.array([ik_target_0.position, ik_target_1.position]),
            target_wxyzs=np.array([ik_target_0.wxyz, ik_target_1.wxyz]),
        )# IK 是在机器人 URDF 的 base frame（root link）坐标系下求解的，不是 Viser 的世界坐标系。

        # Update timing handle.
        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        target0_pos.value = str(np.round(ik_target_0.position, 3))
        target0_rot.value = str(np.round(ik_target_0.wxyz, 3))

        target1_pos.value = str(np.round(ik_target_1.position, 3))
        target1_rot.value = str(np.round(ik_target_1.wxyz, 3))

        # Update visualizer.
        urdf_vis.update_cfg(solution)


if __name__ == "__main__":
    main()
