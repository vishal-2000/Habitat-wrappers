#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Positive x direction -> Initial facing direction of robot
# Positive y direction -> Upward along the height of the robot
# Positive z direction -> 

from habitat.core.dataset import Episode
from numpy import float32
import habitat
from habitat.tasks.rearrange.utils import IkHelper

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

def move_robot_towards_target(env):
    ag_pos = env.sim.get_agent_state(agent_id=0).position
    target = env.sim.get_target_objs_start()
    
def get_target_joint_pos(env=None, target_ee=[]):
    import numpy as np
    if target_ee==[]:
        target_ee = env.sim.get_target_objs_start()
    else:
        target_ee = np.array([target_ee])
    # print("Target_EE: {}")
    # target_ee = np.array(target_ee, dtype=float32)
    print("Target EE.{}".format(target_ee))
    print("Type of target_ee.{}".format(type(target_ee)))
    return env._sim.ik_helper.calc_ik(target_ee[0])


def example():
    # Note: Use with for the example testing, doesn't need to be like this on the README

    with habitat.Env(
        config=habitat.get_config(
            "Vishal_tests/configs/rearrangepick_replica_cad.yaml"# "configs/tasks/rearrangepick_replica_cad_example.yaml"
        )
    ) as env:
        print("Environment creation successful")
        observations = env.reset()  # noqa: F841

        import numpy as np
        env._sim.ik_helper = IkHelper(
                                env._sim.habitat_config.IK_ARM_URDF,
                                np.array(env._sim.robot.params.arm_init_params),
                            )

        print("Agent acting inside environment.")
        count_steps = 0
        import cv2
        height, width, channels = observations['robot_third_rgb'].shape
        sizee = (width, height)
        img_array = []
        from collections import OrderedDict
        import numpy as np
        #env.task.
        print("get_agent_state: {}".format(env.sim.get_agent_state(agent_id=0)))
        #print("get_agent_state position: {}".format(env.sim.get_agent_state(agent_id=0).position))

        # Get obj ids
        #print("Object ids: {}".format(env.sim.get_existing_object_ids(scene_id=env._current_episode.scene_id)))
        #env.task._sim.
        print("Scene obj ids: {}".format(env.sim.scene_obj_ids))
        print("Get targets: {}".format(env.sim.get_targets()))
        print("Get target objs start: {}".format(env.sim.get_target_objs_start()))
        #print("Viz id object: {}".format(env.sim.visualize_position(position=env.sim.get_target_objs_start(), viz_id=None, r=0.05)))
        #print("Episode_info: {}".format(env.current_epi))
        #print("Get target transforms: {}".format(env.sim._get_target_trans()))
        #exit()
        #print("Object ids: {}".format(print("Object ids: {}".format(env.sim.get_existing_object_ids(scene_id=env._current_episode.scene_id)))))
        print("Get metrics: {}".format(env.get_metrics()))
        #exit()
        #targets = []
        #for i, obj_id in enumerate(env.sim.scene_obj_ids):
        #    #print("Targets: {}".format(env._dataset.episodes[0]. ))#.targets))
        #    print("{}. Object ID: {}\tPosition: {}".format(i+1, obj_id, env._sim.get_translation(obj_id)))
        #    dist = np.sqrt(np.sum((np.array(env._sim.get_translation(obj_id)) - ee_pose)**2))
        #    if dist < closest_dist:
        #        closest_dist = dist
        #        closest_obj_id = obj_id
        #    if obj_id>=29:
        #        targets.append([obj_id, np.array(env._sim.get_translation(obj_id))])
        target1 = np.array([0.2, 0, 0.9 ]) # np.array([0.7167317867279053, -0.21682316064834595, 0.9539435505867004]) #[] #np.array([0, 1.39, 0 ])
        #for target in targets:

        target_joint_pos = np.array(get_target_joint_pos(env=env, target_ee = target1))
        target_pos = env.sim.get_target_objs_start()[0]
        # step_freq = 20
        current_pos = env.sim.robot.arm_joint_pos
        ee_pose = env._sim.ik_helper.calc_fk(np.array(current_pos))
        diff_j = np.sqrt(np.sum((target_joint_pos - current_pos)**2))
        diff_ee = np.sqrt(np.sum((target1 - ee_pose)**2))
        print("EE distance: {}\t Joint Distance: {}".format(diff_ee, diff_j))
        no_of_collisions = 0

        closest_obj_id = 0
        closest_dist = 1000000000
        for i, obj_id in enumerate(env.sim.scene_obj_ids):
            #print("Targets: {}".format(env._dataset.episodes[0]. ))#.targets))
            print("{}. Object ID: {}\tPosition: {}".format(i+1, obj_id, env._sim.get_translation(obj_id)))
            dist = np.sqrt(np.sum((np.array(env._sim.get_translation(obj_id)) - ee_pose)**2))
            if dist < closest_dist:
                closest_dist = dist
                closest_obj_id = obj_id
        print("Closest obj id: {}\tClosest Distance: {}".format(closest_obj_id, closest_dist))

        #exit()
        while (not env.episode_over) and (diff_ee > 0.15) and (no_of_collisions==0):
            #base_dist = target_pos - 
            current_pos = env.sim.robot.arm_joint_pos
            #print("Current pos: {}".format(current_pos))
            #print("Target pos: {}".format(target_joint_pos))
            joint_transition = target_joint_pos - current_pos
            action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', joint_transition)])}
            #print("Action space sample: {}".format(env.action_space.sample()))
            #print("SAMPLE navigable point: {}".format(env.sim.sample_navigable_point()))
            #action_0 = {'action': 'BASE_VELOCITY', 'action_args': OrderedDict([('base_vel', np.array([-0.5,  0 ], dtype=float))])} # float32
            #print("Sample actions: {}".format(env.action_space.sample()))
            #print("Episode items: {}".format(env.sim.get_observations_at()))
            #action_0 = {'action': 'ARM_EE_ACTION', 'action_args': OrderedDict([('arm_action', np.array([0.1,  0, -0.08], dtype=float)), ('grip_action', None)])} # x, y, z (forward, left, up) # np.array([-0.1], dtype=float)
            

            #action_0 = get_action(env)
            #action_0 = {'action': 'ARM_EE_ACTION', 'action_args': OrderedDict([('ee_pos', np.array([0.2, 0, -0.3 ], dtype=float))])} # np.array([0.2, 0, -0.3 ]
            #print("Sample action modified: {}".format(action_0))
            observations = env.step(action_0)# env.step(env.action_space.sample())  # noqa: F841
            #print("TASK agent RGB head sensor state: {}".format(env.task._sim.get_agent_state(0).sensor_states['robot_head_rgb'].position))
            #print("Get metrics: {}".format(env.get_metrics()))
            #print("get_agent_state position: {}".format(env.sim.get_agent_state(agent_id=0).position))
            #print("Robot third rgb: {}".format(observations['robot_third_rgb'].shape))
            img_array.append(observations['robot_third_rgb'])
            # cv2.imshow('image', observations['robot_head_rgb'])
            #env.render(mode="rgb")
            #print("Elapsed steps, Elapsed seconds: {}".format(env._elapsed_steps, env._elapsed_seconds))
            count_steps += 1
            current_pos = env.sim.robot.arm_joint_pos
            ee_pose = env._sim.ik_helper.calc_fk(np.array(current_pos))
            diff_j = np.sqrt(np.sum((target_joint_pos - current_pos)**2))
            diff_ee = np.sqrt(np.sum((target1 - ee_pose)**2))
            no_of_collisions = env.get_metrics()['robot_collisions']['total_collisions']
            print("EE distance: {}\t Joint Distance: {}\t Collisions: {}".format(diff_ee, diff_j, no_of_collisions))
            #if count_steps >= 65:
            #    break
            #if env.episode_over == True:
            #    env._episode_over = False
            #    env.current_episode =  env.episode_iterator.__next__()
            #    env._current_episode_index = env._current_episode.episode_id
            #print(observations)
        # The object is in contact
        # Now grasp it
        action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', np.zeros(shape=7, dtype=float)), ('grip_action', np.array([0.91156936], dtype=float32))])}
        observations = env.step(action_0)
        img_array.append(observations['robot_third_rgb'])
        count_steps += 1
        gripped = True
        # Gripped, now move it out of your natural way
        print("Final:")
        print("EE distance: {}\t Joint Distance: {}\t Collisions: {}".format(diff_ee, diff_j, no_of_collisions))
        print("New End_effector pose: {}".format(env._sim.robot.ee_transform.translation))
        current_pos = env.sim.robot.arm_joint_pos
        ee_pose = env._sim.ik_helper.calc_fk(np.array(current_pos))
        print("EE_pos: {}".format(ee_pose))
        while not env.episode_over:
            print("\n------------------------------------------------------------------------------------\n")
            print("POSITION REACHED\n")
            print("------------------------------------------------------------------------------------\n")
            action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', np.array([-1, -1, -1, -1, -1, -1, -1], dtype=float))])}
            observations = env.step(action_0)
            img_array.append(observations['robot_third_rgb'])
            count_steps += 1
            if count_steps>=150 and gripped: # Now place it
                action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', np.zeros(shape=7, dtype=float)), ('grip_action', np.array([-0.91156936], dtype=float32))])}
                observations = env.step(action_0)
                img_array.append(observations['robot_third_rgb'])
                count_steps += 1
                gripped = False


        print("CUrrent episode: {}".format(env._current_episode_index))
        out = cv2.VideoWriter('/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-lab/Vishal_tests/videos/pick_place_pos_vis.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, sizee)
        for i in range(len(img_array)):
            im_bgr = cv2.cvtColor(img_array[i], cv2.COLOR_RGB2BGR)
            out.write(im_bgr)
        out.release()
        print("Sample action: {}".format(env.action_space.sample()))
        print("Action space: {}".format(env.action_space))
        print("Episode finished after {} steps.".format(count_steps))
        print("NO of episodes: {}".format(env.number_of_episodes))


if __name__ == "__main__":
    example()
