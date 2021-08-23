from os import path as osp

import numpy as np
import pytest

import examples.settings
import habitat_sim
import panda_robot as panda_robot
from helpers.utils import simulate

def test_panda_robot_wrapper_arm(fixed_base):
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
        print("Joint poses Initial1: {}".format(panda._get_motor_pos(panda.params.arm_joints[0])))
        observations += simulate(sim, 1.0, produce_debug_video)

        print("Joint poses Initial2: {}".format(panda._get_motor_pos(panda.params.arm_joints[0])))
        # retract the arm
        #observations += panda._interpolate_arm_control(
        #    [1.2299035787582397, 2.345386505126953],
        #    [panda.params.arm_joints[1], panda.params.arm_joints[3]],
        #    1,
        #    30,
        #    produce_debug_video,
        #)

        
        # print("Joint poses Initial3: {}".format(panda._get_motor_pos()))
        # produce some test debug video
        if produce_debug_video:
            from habitat_sim.utils import viz_utils as vut

            vut.make_video(
                observations,
                "color_sensor",
                "color",
                "/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-sim/Vishal_custom_tests/videos/test_panda_simple_arm",
                open_vid=True,
            )

test_panda_robot_wrapper_arm(True)