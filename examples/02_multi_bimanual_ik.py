import time
import viser
import yourdfpy
from robot_descriptions.loaders.yourdfpy import load_robot_description
import numpy as np
import pyroki as pk
from viser.extras import ViserUrdf
import pyroki_snippets as pks

def load_robot(path, mesh_dir):
    urdf = yourdfpy.URDF.load(path, mesh_dir=mesh_dir)
    robot = pk.Robot.from_urdf(urdf)
    return urdf, robot

def main():
    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)

    # dual_xarm7
    urdf1, robot1 = load_robot(
        path = "../assert/xarm7/dual_xarm7.urdf",
        mesh_dir="../assert/xarm7"
    )
    target_link_names_1 = ["right_link_eef", "left_link_eef"]

    # openarm_bimanual
    urdf2, robot2 = load_robot(
        path = "/home/ps/ros2_ws/openarm_bimanual.urdf",
        mesh_dir="/home/ps/ros2_ws"
    )
    target_link_names_2 = ["openarm_right_hand", "openarm_left_hand"] # 需要移动urdf到阳间位置

    # g1_dual_arm
    urdf3, robot3 = load_robot(
        path = "../thirdparty/unitree_rl_gym/resources/robots/g1_description/g1_dual_arm.urdf",
        mesh_dir="../thirdparty/unitree_rl_gym/resources/robots/g1_description"
    )
    target_link_names_3 = ["right_rubber_hand", "left_rubber_hand"]

    vis1 = ViserUrdf(server, urdf1, root_node_name="/dual_xarm7")
    vis2 = ViserUrdf(server, urdf2, root_node_name="/openarm_bimanual")
    vis3 = ViserUrdf(server, urdf3, root_node_name="/g1_dual_arm")

    # Create interactive controller with initial position.
    ik_target_0 = server.scene.add_transform_controls(
        "/ik_target_0", scale=0.2, position=(0.41, -0.3, 0.56), wxyz=(1, 0, 0, 0)
    )
    ik_target_1 = server.scene.add_transform_controls(
        "/ik_target_1", scale=0.2, position=(0.41, 0.3, 0.56), wxyz=(1, 0, 0, 0)
    )
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    while True:
        start_time = time.time()

        # robot1 dual_xarm7
        sol1 = pks.solve_ik_with_multiple_targets(
            robot=robot1,
            target_link_names=target_link_names_1,
            target_positions=np.array([ik_target_0.position, ik_target_1.position]),
            target_wxyzs=np.array([ik_target_0.wxyz, ik_target_1.wxyz]),
        )

        # robot2 openarm_bimanual
        sol2 = pks.solve_ik_with_multiple_targets(
            robot=robot2,
            target_link_names=target_link_names_2,
            target_positions=np.array([ik_target_0.position, ik_target_1.position]),
            target_wxyzs=np.array([ik_target_0.wxyz, ik_target_1.wxyz]),
        )

        # robot3 g1_dual_arm
        sol3 = pks.solve_ik_with_multiple_targets(
            robot=robot3,
            target_link_names=target_link_names_3,
            target_positions=np.array([ik_target_0.position, ik_target_1.position]),
            target_wxyzs=np.array([ik_target_0.wxyz, ik_target_1.wxyz]),
        )

        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # 更新三个机器人
        vis1.update_cfg(sol1)
        vis2.update_cfg(sol2)
        vis3.update_cfg(sol3)


if __name__ == "__main__":
    main()
