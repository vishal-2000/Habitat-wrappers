from os import path as osp

import numpy as np
import pytest

import examples.settings
import habitat_sim
import panda_robot as panda_robot
from helpers.utils import simulate

def test_panda_robot_wrapper(fixed_base):
    # set this to output test results as video for easy investigation
    produce_debug_video = True
    observations = []
    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"# "/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-sim/data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"# "NONE"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        obj_template_mgr = sim.get_object_template_manager()
        rigid_obj_mgr = sim.get_rigid_object_manager()

        # setup the camera for debug video (looking at 0,0,0)
        sim.agents[0].scene_node.translation = [0.0, -1.0, 2.0]

        # add a ground plane
        cube_handle = obj_template_mgr.get_template_handles("cubeSolid")[0]
        cube_template_cpy = obj_template_mgr.get_template_by_handle(cube_handle)
        cube_template_cpy.scale = np.array([5.0, 0.2, 5.0])
        obj_template_mgr.register_template(cube_template_cpy)
        ground_plane = rigid_obj_mgr.add_object_by_template_handle(cube_handle)
        ground_plane.translation = [0.0, -0.2, 0.0]
        ground_plane.motion_type = habitat_sim.physics.MotionType.STATIC

        # compute a navmesh on the ground plane
        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)
        sim.navmesh_visualization = True

        # add the robot to the world via the wrapper
        robot_path = "/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-sim/Vishal_custom_tests/data/robot_urdfs/robot/panda/panda.urdf"# "/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-sim/Vishal_custom_tests/data/robots/fixed_base_test.urdf" # "data/robots/hab_panda/robots/hab_panda.urdf"
        panda = panda_robot.PandaRobot(robot_path, sim, fixed_base=fixed_base)
        panda.reconfigure()
        assert panda.get_robot_sim_id() == 1  # 0 is the ground plane
        print(panda.get_link_and_joint_names())
        observations += simulate(sim, 1.0, produce_debug_video)

        # retract the arm
        observations += panda._interpolate_arm_control(
            [1.2299035787582397, 2.345386505126953],
            [panda.params.arm_joints[1], panda.params.arm_joints[3]],
            1,
            30,
            produce_debug_video,
        )

        # ready the arm
        observations += panda._interpolate_arm_control(
            [-3.45, 2.1],
            [panda.params.arm_joints[1], panda.params.arm_joints[3]],
            1,
            30,
            produce_debug_video,
        )

        # setting arm motor positions
        panda.arm_motor_pos = np.zeros(len(panda.params.arm_joints))
        observations += simulate(sim, 1.0, produce_debug_video)

        # set base ground position from navmesh
        # NOTE: because the navmesh floats above the collision geometry we should see a pop/settle with dynamics and no fixed base
        target_base_pos = sim.pathfinder.snap_point(panda.sim_obj.translation)
        panda.base_pos = target_base_pos
        assert panda.base_pos == target_base_pos
        observations += simulate(sim, 1.0, produce_debug_video)
        if fixed_base:
            assert np.allclose(panda.base_pos, target_base_pos)
        else:
            assert not np.allclose(panda.base_pos, target_base_pos)

        # arm joint queries and setters
        print(f" Arm joint velocities = {panda.arm_velocity}")
        panda.arm_joint_pos = np.ones(len(panda.params.arm_joints))
        panda.arm_motor_pos = np.ones(len(panda.params.arm_joints))
        print(f" Arm joint positions (should be ones) = {panda.arm_joint_pos}")
        print(f" Arm joint limits = {panda.arm_joint_limits}")
        panda.arm_motor_pos = panda.arm_motor_pos
        observations += simulate(sim, 1.0, produce_debug_video)

        # test gripper state
        panda.open_gripper()
        observations += simulate(sim, 1.0, produce_debug_video)
        # assert panda.is_gripper_open
        #assert not panda.is_gripper_closed
        #panda.close_gripper()
        #observations += simulate(sim, 1.0, produce_debug_video)
        #assert panda.is_gripper_closed
        #assert not panda.is_gripper_open

        # halfway open
        panda.set_gripper_target_state(0.5)
        observations += simulate(sim, 0.5, produce_debug_video)
        #assert not panda.is_gripper_open
        #assert not panda.is_gripper_closed

        # kinematic open/close (checked before simulation)
        panda.gripper_joint_pos = panda.params.gripper_open_state
        #assert np.allclose(panda.gripper_joint_pos, panda.params.gripper_open_state)
        #assert panda.is_gripper_open
        observations += simulate(sim, 0.2, produce_debug_video)
        panda.gripper_joint_pos = panda.params.gripper_closed_state
        #assert panda.is_gripper_closed
        observations += simulate(sim, 0.2, produce_debug_video)

        # end effector queries
        # print(f" End effector link id = {panda.ee_link_id}")
        #print(f" End effector local offset = {panda.ee_local_offset}")
        #print(f" End effector transform = {panda.ee_transform}")
        #print(
        #    f" End effector translation (at current state) = {panda.calculate_ee_forward_kinematics(panda.sim_obj.joint_positions)}"
        #)
        #invalid_ef_target = np.array([100.0, 200.0, 300.0])
        #print(
        #    f" Clip end effector target ({invalid_ef_target}) to reach = {panda.clip_ee_to_workspace(invalid_ef_target)}"
        #)

        # produce some test debug video
        if produce_debug_video:
            from habitat_sim.utils import viz_utils as vut

            vut.make_video(
                observations,
                "color_sensor",
                "color",
                "/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-sim/Vishal_custom_tests/videos/test_panda_robot_wrapper4__fixed_base=" + str(fixed_base),
                open_vid=True,
            )

test_panda_robot_wrapper(True)