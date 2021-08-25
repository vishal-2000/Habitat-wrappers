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
        #cube_handle = obj_template_mgr.get_template_handles("cubeSolid")[0]
        #cube_template_cpy = obj_template_mgr.get_template_by_handle(cube_handle)
        #cube_template_cpy.scale = np.array([5.0, 0.2, 5.0])
        #obj_template_mgr.register_template(cube_template_cpy)
        #ground_plane = rigid_obj_mgr.add_object_by_template_handle(cube_handle)
        #ground_plane.translation = [0.0, -0.2, 0.0]
        #ground_plane.motion_type = habitat_sim.physics.MotionType.STATIC

        # compute a navmesh on the ground plane
        #navmesh_settings = habitat_sim.NavMeshSettings()
        #navmesh_settings.set_defaults()
        #sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)
        #sim.navmesh_visualization = True

        # add the robot to the world via the wrapper
        robot_path = "/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-sim/Vishal_custom_tests/data/robot_urdfs/robot/panda/panda.urdf"# "/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-sim/Vishal_custom_tests/data/robots/fixed_base_test.urdf" # "data/robots/hab_panda/robots/hab_panda.urdf"
        panda = panda_robot.PandaRobot(robot_path, sim, fixed_base=fixed_base)
        panda.reconfigure()
        import magnum as mn
        #panda.base_rot = mn.Rad(-1.24) #mn.Quaternion.rotation(
           # mn.Rad(1.2), mn.Vector3(0, 1, 0)
        #)
        panda.sim_obj.rotation = mn.Quaternion.rotation(
            mn.Rad(-1.5708), mn.Vector3(1, 0, 0)
        )
        #assert panda.get_robot_sim_id() == 1  # 0 is the ground plane
        print(panda.get_link_and_joint_names())
        for i, joint in enumerate(panda.params.arm_joints):
            print("Joint poses Initial{}: {}".format(i, panda._get_motor_pos(joint)))
        observations += simulate(sim, 1.0, produce_debug_video)

        for i, joint in enumerate(panda.params.arm_joints):
            print("Joint poses Initial{}: {}".format(i, panda._get_motor_pos(joint)))
        # retract the arm
        observations += panda._interpolate_arm_control(
            [1.2299035787582397, 0.345386505126953],
            [panda.params.arm_joints[2], panda.params.arm_joints[4]],
            1,
            30,
            produce_debug_video,
        )

        print(f" Arm joint limits = {panda.arm_joint_limits}")

        
        for i, joint in enumerate(panda.params.arm_joints):
            print("Joint poses Initial{}: {}".format(i, panda._get_motor_pos(joint)))
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

        print("Panda.sim_obj.translation: {}".format(panda.sim_obj.translation))
        print("Panda.sim_obj.rotation: {}\nPanda.sim_obj.rotation.angle: {}".format(panda.sim_obj.rotation, panda.sim_obj.rotation.angle))
        print("sim.pathfinder.snap_point(panda.sim_obj.translation): {}".format(sim.pathfinder.snap_point(panda.sim_obj.translation)))
        #panda.base_rot = 1.2
        #print("Panda.sim_obj.translation: {}".format(panda.sim_obj.translation))
        #print("Panda.sim_obj.rotation: {}\nPanda.sim_obj.rotation.angle: {}".format(panda.sim_obj.rotation, panda.sim_obj.rotation.angle))
        #panda.base_rot = panda.sim_obj.rotation

test_panda_robot_wrapper_arm(True)