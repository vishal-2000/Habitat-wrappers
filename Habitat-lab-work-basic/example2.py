#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat

def get_action(env, action_0=None):
    if env==None:
        print("Environment not initialized, cannot perform get_action")
        return None
    if (action_0==None):
        action_0 = env.action_space.sample()
    if action_0['action']=='BASE_VELOCITY':
        from collections import OrderedDict
        action_0['action_args'] = OrderedDict([('base_vel', action_0['action_args'])])

    return action_0
    


def example():
    # Note: Use with for the example testing, doesn't need to be like this on the README

    with habitat.Env(
        config=habitat.get_config(
            "Vishal_tests/configs/rearrangepick_replica_cad.yaml"# "configs/tasks/rearrangepick_replica_cad_example.yaml"
        )
    ) as env:
        print("Environment creation successful")
        observations = env.reset()  # noqa: F841

        print("Agent acting inside environment.")
        count_steps = 0
        import cv2
        height, width, channels = observations['robot_third_rgb'].shape
        sizee = (width, height)
        img_array = []
        from collections import OrderedDict
        import numpy as np
        while not env.episode_over:
            #print("Action space sample: {}".format(env.action_space.sample()))
            #action_0 = {'action': 'BASE_VELOCITY', 'action_args': OrderedDict([('base_vel', np.array([-12.821854,  16.21554 ], dtype=float))])} # float32
            #print("Sample actions: {}".format(env.action_space.sample()))
            if env._sim.grasp_mgr.is_grasped:
                print("\nGrasped obj id: {}-----------------------------------------------------------------------".format(env._sim.grasp_mgr._snapped_obj_id))
                print("Grasped obj pos: {}\n".format(env._sim.get_translation(env._sim.grasp_mgr._snapped_obj_id)))
            action_0 = get_action(env)
            print("Sample action modified: {}".format(action_0))
            observations = env.step(action_0)# env.step(env.action_space.sample())  # noqa: F841
            #print("Robot third rgb: {}".format(observations['robot_third_rgb'].shape))
            current_pos = env.sim.robot.arm_joint_pos
            print("New pose: {}".format(current_pos))
            ee_pose = env._sim.ik_helper.calc_fk(np.array(current_pos))
            print("New End_effector pose: {}".format(ee_pose))
            print("New End_effector pose: {}".format(env._sim.robot.ee_transform.translation))
            img_array.append(observations['robot_third_rgb'])
            # cv2.imshow('image', observations['robot_head_rgb'])
            #env.render(mode="rgb")
            count_steps += 1
            #print(observations)
        out = cv2.VideoWriter('/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-lab/Vishal_tests/videos/project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, sizee)
        for i in range(len(img_array)):
            im_bgr = cv2.cvtColor(img_array[i], cv2.COLOR_RGB2BGR)
            out.write(im_bgr)
        out.release()
        print("Sample action: {}".format(env.action_space.sample()))
        print("Action space: {}".format(env.action_space))
        print("Episode finished after {} steps.".format(count_steps))


if __name__ == "__main__":
    example()
