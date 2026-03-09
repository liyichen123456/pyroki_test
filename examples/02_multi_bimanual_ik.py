import time
import viser
import yourdfpy
import numpy as np
import pyroki as pk
from viser.extras import ViserUrdf
import pyroki_snippets as pks


def load_robot(path, mesh_dir):
    urdf = yourdfpy.URDF.load(path, mesh_dir=mesh_dir)
    robot = pk.Robot.from_urdf(urdf)
    return urdf, robot


def main():

    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)

    # ------------------------------------------------
    # load robots
    # ------------------------------------------------

    urdf1, robot1 = load_robot(
        "../assert/xarm7/dual_xarm7.urdf",
        "../assert/xarm7"
    )
    target_link_names_1 = ["right_link_eef", "left_link_eef"]

    urdf2, robot2 = load_robot(
        "../thirdparty/ros2_ws_1/openarm_bimanual.urdf",
        "../thirdparty/ros2_ws_1"
    )
    target_link_names_2 = ["openarm_right_hand", "openarm_left_hand"]

    urdf3, robot3 = load_robot(
        "../thirdparty/unitree_rl_gym/resources/robots/g1_description/g1_dual_arm.urdf",
        "../thirdparty/unitree_rl_gym/resources/robots/g1_description"
    )
    target_link_names_3 = ["right_rubber_hand", "left_rubber_hand"]

    # ------------------------------------------------
    # create robot base frames
    # ------------------------------------------------

    base1 = server.scene.add_frame("/robot1")
    base2 = server.scene.add_frame("/robot2")
    base3 = server.scene.add_frame("/robot3")

    offset1 = np.array([0.0, 0.0, -0.3]) # dual_xarm7
    offset2 = np.array([0.0, 0.0, -0.3]) # openarm_bimanual
    offset3 = np.array([0.0, 0.0, -0.1]) # g1_dual_arm

    base1.position = offset1
    base2.position = offset2
    base3.position = offset3

    # ------------------------------------------------
    # attach URDF to frames
    # ------------------------------------------------

    vis1 = ViserUrdf(server, urdf1, root_node_name="/robot1")
    vis2 = ViserUrdf(server, urdf2, root_node_name="/robot2")
    vis3 = ViserUrdf(server, urdf3, root_node_name="/robot3")

    # ------------------------------------------------
    # IK targets
    # ------------------------------------------------

    ik_target_0 = server.scene.add_transform_controls(
        "/ik_target_0",
        scale=0.2,
        position=(0.21, -0.3, 0.26),
        # wxyz=(0, 0, 1, 0),
        wxyz=(-0.001 , 0.702 , 0.001 , 0.713),
    )

    ik_target_1 = server.scene.add_transform_controls(
        "/ik_target_1",
        scale=0.2,
        position=(0.21, 0.3, 0.26),
        # wxyz=(0, 0, 1, 0),
        wxyz=(-0.001 , 0.702 , 0.001 , 0.713),


    )

    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    # ------------------------------------------------
    # loop
    # ------------------------------------------------

    while True:

        start_time = time.time()

        target_world = np.array([
            ik_target_0.position,
            ik_target_1.position
        ])

        target_rot = np.array([
            ik_target_0.wxyz,
            ik_target_1.wxyz
        ])

        # ---------------------------
        # robot1 IK
        # ---------------------------

        # sol1 = pks.solve_ik_with_multiple_targets(
        sol1 = pks.solve_ik_with_multiple_targets(
            robot=robot1,
            target_link_names=target_link_names_1,
            target_positions=target_world - offset1,
            target_wxyzs=target_rot,
        )

        # ---------------------------
        # robot2 IK
        # ---------------------------

        sol2 = pks.solve_ik_with_multiple_targets(
            robot=robot2,
            target_link_names=target_link_names_2,
            target_positions=target_world - offset2,
            target_wxyzs=target_rot,
        )

        # ---------------------------
        # robot3 IK
        # ---------------------------

        sol3 = pks.solve_ik_with_multiple_targets(
            robot=robot3,
            target_link_names=target_link_names_3,
            target_positions=target_world - offset3,
            target_wxyzs=target_rot,
        )

        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # update visualization

        vis1.update_cfg(sol1)
        vis2.update_cfg(sol2)
        vis3.update_cfg(sol3)


if __name__ == "__main__":
    main()