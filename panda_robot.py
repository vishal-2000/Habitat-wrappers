# Wrapper class for Franka Emika Panda
import magnum as mn
import numpy as np

from habitat_sim.robots.mobile_manipulator import (
    MobileManipulator,
    MobileManipulatorParams,
    RobotCameraParams
)

class PandaRobot(MobileManipulator):
    def __init__(self, urdf_path, sim, limit_robo_joints=True, fixed_base=True):
        panda_params = MobileManipulatorParams(
            arm_joints=list(range(0, 7)),
            gripper_joints=[9, 10],
            wheel_joints=None, # [2, 4],
            arm_init_params=[0, 0, 0, -0.10, 0, 3.1416, 1.57],# [ -1.08, 0.1, 0.935, -0.001, 1.573, 0.005],
            gripper_init_params=[0.00, 0.00],
            ee_offset=mn.Vector3(0.107, 0, 0), # 0.08, 0, 0
            ee_link=8,
            ee_constraint=None,# np.array([[0.4, 1.2], [-0.7, 0.7], [0.25, 1.5]]),
            cameras={},#"""{
                #"robot_arm": RobotCameraParams(
                #    cam_offset_pos=mn.Vector3(0, 0.0, 0.1),
                #    cam_look_at_pos=mn.Vector3(0.1, 0.0, 0.0),
                #    attached_link_id=22,
                #    relative_transform=mn.Matrix4.rotation_y(mn.Deg(-90))
                #    @ mn.Matrix4.rotation_z(mn.Deg(90)),
                #),
                #"robot_head": RobotCameraParams(
                #    cam_offset_pos=mn.Vector3(0.17, 1.2, 0.0),
                #    cam_look_at_pos=mn.Vector3(0.75, 1.0, 0.0),
                #    attached_link_id=-1,
                #),
                #"robot_third": RobotCameraParams(
                #    cam_offset_pos=mn.Vector3(-0.5, 1.7, -0.5),
                #    cam_look_at_pos=mn.Vector3(1, 0.0, 0.75),
                #    attached_link_id=-1,
                #),
            #}""",
            gripper_closed_state=[0.0, 0.0],
            gripper_open_state=[0.04, 0.04],
            gripper_state_eps=0.001,
            arm_mtr_pos_gain=0.3,
            arm_mtr_vel_gain=0.3,
            arm_mtr_max_impulse=10.0,
            wheel_mtr_pos_gain=0.0,
            wheel_mtr_vel_gain=0.0,# 1.3,
            wheel_mtr_max_impulse=10.0,
            base_offset=mn.Vector3(0, 0, 0),
            base_link_names={
                "panda_link0"
            },
        )
        super().__init__(panda_params, urdf_path, sim, limit_robo_joints, fixed_base)
        self.back_joint_id = None
        self.head_rot_jid = None
        self.head_tilt_jid = None

    def reconfigure(self) -> None:
        super().reconfigure()

        # NOTE: this is necessary to set locked head and back positions
        self.update()

    def reset(self) -> None:
        super().reset()

        # NOTE: this is necessary to set locked head and back positions
        self.update()

    @property
    def base_transformation(self):
        add_rot = mn.Matrix4.rotation(mn.Rad(-np.pi / 2), mn.Vector3(1.0, 0, 0))
        return self.sim_obj.transformation @ add_rot

    def update(self):
        super().update()
        # Fix the head.
        # self._set_joint_pos(self.head_rot_jid, 0)
        # self._set_motor_pos(self.head_rot_jid, 0)
        # self._set_joint_pos(self.head_tilt_jid, np.pi / 2)
        # self._set_motor_pos(self.head_tilt_jid, np.pi / 2)
        # Fix the back
        # fix_back_val = 0.15
        # self._set_joint_pos(self.back_joint_id, fix_back_val)
        # self._set_motor_pos(self.back_joint_id, fix_back_val)
