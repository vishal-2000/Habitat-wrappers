# Created by Vishal Reddy Mandadi

from habitat.core.dataset import Episode
from numpy import float32
import habitat
from habitat.tasks.rearrange.utils import IkHelper
from collections import OrderedDict
import numpy as np
import cv2

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
    env._sim.ik_helper.set_arm_state(env.sim.robot.arm_joint_pos, joint_vel=None)
    print("Target EE.{}".format(target_ee))
    print("Type of target_ee.{}".format(type(target_ee)))
    while len(target_ee)==1:
        target_ee = target_ee[0]
    return env._sim.ik_helper.calc_ik(target_ee)

def get_obj_positions(env):
    targets = []
    for i, obj_id in enumerate(env.sim.scene_obj_ids):
        print("{}. Object ID: {}\tPosition: {}".format(i+1, obj_id, env._sim.get_translation(obj_id)))
        targets.append([obj_id, np.array(env._sim.get_translation(obj_id))])
    return targets

def get_closest_objects(env, max_dist=4.0):
    targets = []
    current_pos = env.sim.robot.arm_joint_pos
    ee_pose = env._sim.ik_helper.calc_fk(np.array(current_pos))
    for i, obj_id in enumerate(env.sim.scene_obj_ids):
        obj_pos = np.array(env._sim.get_translation(obj_id))
        print("{}. Object ID: {}\tPosition: {}".format(i+1, obj_id, obj_pos))
        dist = np.sqrt(np.sum((obj_pos - ee_pose)**2))
        if dist < max_dist:
            targets.append([obj_id, obj_pos])
    return targets    

def get_closest_obj(env, exclude_list=[]):
    targets = []
    current_pos = env.sim.robot.arm_joint_pos
    ee_pose = env._sim.ik_helper.calc_fk(np.array(current_pos))
    min_dist = 1000000000
    min_obj_id = 0
    for i, obj_id in enumerate(env.sim.scene_obj_ids):
        if obj_id in exclude_list:
            continue
        obj_pos = np.array(env._sim.get_translation(obj_id))
        dist = np.sqrt(np.sum((obj_pos - ee_pose)**2))
        print("{}. Object ID: {}\tPosition: {}\tDistance: {}".format(i+1, obj_id, obj_pos, dist))
        if dist < min_dist:
            min_dist = dist
            min_obj_id = obj_id
            target = [min_obj_id, obj_pos]
    print("Closest object id: {} | Distance: {} | Position: {}".format(min_obj_id, min_dist, target[1]))
    return target

def convert_to_video_and_save(img_array=[], dest='/home/vishal/Volume_E/Active/Undergrad_research/Harish_Y/Project_delan/Initial_work/Habitat/habitat-lab/Vishal_tests/videos/pick_place_pos_vis3.avi'):
    height, width, channels = img_array[0].shape
    sizee = (width, height)
    out = cv2.VideoWriter(dest ,cv2.VideoWriter_fourcc(*'DIVX'), 15, sizee)
    for i in range(len(img_array)):
        im_bgr = cv2.cvtColor(img_array[i], cv2.COLOR_RGB2BGR)
        out.write(im_bgr)
    out.release()

def print_static_obj_list(env):
    print("Static objects: {}".format(env._dataset.episodes[0].static_objs))

def find_closest_obj_index(env=None, excluded_list=[]):
    scene_obj_pos = env._sim.get_scene_pos()
    for id in excluded_list:
        sim_index = None
        for i in range(len(scene_obj_pos)):
            sim_idx = env._sim.scene_obj_ids[i]
            if sim_idx==id:
                sim_index = i
                break
        scene_obj_pos = np.delete(scene_obj_pos, sim_index, 0)
    ee_pos = env._sim.robot.ee_transform.translation
    if len(scene_obj_pos) != 0:
        # Get the target the EE is closest to.
        closest_obj_idx = np.argmin(
            np.linalg.norm(scene_obj_pos - ee_pos, ord=2, axis=-1)
        )
        closest_obj_pos = scene_obj_pos[closest_obj_idx]
        to_target = np.linalg.norm(ee_pos - closest_obj_pos, ord=2)
        sim_idx = env._sim.scene_obj_ids[closest_obj_idx]

        return sim_idx, closest_obj_pos


def vanilla_pick_place():
    # Load environment
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
        img_array = []

        #get_static_obj_list(env)
        #print_static_obj_list(env) # TO print static objs in the dataset - https://github.com/facebookresearch/habitat-lab/blob/aa96b33b0cba4f227b8e70b9f965795e5457d676/habitat/datasets/rearrange/rearrange_dataset.py#L25
        print("Static objects: {}".format(env.sim.ep_info['static_objs']))
        print("\n-----------------------------------------------------------------------------------------------------\n")
        print("VIz object ids: {}".format(env.sim.viz_obj_ids))
        print("\n-----------------------------------------------------------------------------------------------------\n")
        print("No of static objects: {}".format(len(env.sim.ep_info['static_objs'])))
        print("\n-----------------------------------------------------------------------------------------------------\n")
        obj_id, obj_pos = find_closest_obj_index(env)
        print("Closest object id: {}\t Distance: {}".format(obj_id, obj_pos))
        print("SCENE obj pos: {}".format(env._sim.get_scene_pos()))
        #exit()
        #exit()


        # Single episode
        targets = get_closest_objects(env)
        #for obj_id, target in targets:
        excluded_list = []
        rest_pose = np.array([-2.50183, 1.5, -0.256687], dtype=float) # [-2.10183, 1.11991, -0.296687]
        for i in range(10):
            #obj_id, target = get_closest_obj(env, exclude_list=excluded_list)
            obj_id, target = find_closest_obj_index(env, excluded_list)
            print("Object to be picked: {}\t Position: {}".format(obj_id, target))
            #obj_id = 16
            #target = np.array(env._sim.get_translation(obj_id), dtype=float)
            #target[2] = target[2]
            print("Object id: {}\t Target EE: {}".format(obj_id, target))
            excluded_list.append(obj_id)
            if env.episode_over:
                print("Episode over!!!")
                break
            # Pick object
            print("Trying to reach obj -------------------------------------------------------------------------------------------------------")
            target_joint_pos = np.array(get_target_joint_pos(env=env, target_ee = target))
            current_pos = env.sim.robot.arm_joint_pos
            ee_pose = env._sim.robot.ee_transform.translation # env._sim.ik_helper.calc_fk(np.array(current_pos))
            diff_j = np.sqrt(np.sum((target_joint_pos - current_pos)**2))
            diff_ee = np.sqrt(np.sum((target - ee_pose)**2))
            while diff_ee>0.15 and (not env.episode_over):
                current_pos = env._sim.robot.arm_joint_pos
                joint_transition = target_joint_pos - current_pos
                action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', joint_transition)])}
                observations = env.step(action_0)
                img_array.append(observations['robot_third_rgb'])
                count_steps += 1
                current_pos = env.sim.robot.arm_joint_pos
                ee_pose = env._sim.robot.ee_transform.translation # env._sim.ik_helper.calc_fk(np.array(current_pos))
                diff_j = np.sqrt(np.sum((target_joint_pos - current_pos)**2))
                diff_ee = np.sqrt(np.sum((target - ee_pose)**2))
                no_of_collisions = env.get_metrics()['robot_collisions']['total_collisions']
                print("EE distance: {}\t Joint Distance: {}\t Collisions: {}\tEE_pos: {}".format(diff_ee, diff_j, no_of_collisions, ee_pose))
            # Now pick the object if you are close enough
            gripped = False
            if diff_ee<=0.15 and (not env.episode_over):
                action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', np.zeros(shape=7, dtype=float)), ('grip_action', np.array([0.91156936], dtype=float32))])}
                observations = env.step(action_0)
                img_array.append(observations['robot_third_rgb'])
                count_steps += 1
                gripped = True
                print("Object PICKED -------------------------------------------------------------------------------------------------------")
            # If gripped, move the object to rest pose
            print("Trying to reach rest -------------------------------------------------------------------------------------------------------")
            #rest_pose = env.task.desired_resting
            target = rest_pose = np.array([-2.08722, 1.51473, -0.270025], dtype=float)
            target_joint_pos = np.array(get_target_joint_pos(env=env, target_ee = target))
            print("Target joint pos: {}".format(target_joint_pos))
            current_pos = env.sim.robot.arm_joint_pos
            ee_pose = env._sim.robot.ee_transform.translation # env._sim.ik_helper.calc_fk(np.array(current_pos))
            diff_j = np.sqrt(np.sum((target_joint_pos - current_pos)**2))
            diff_ee = np.sqrt(np.sum((target - ee_pose)**2))
            while diff_ee>0.08 and (not env.episode_over) and gripped:
                current_pos = env._sim.robot.arm_joint_pos
                joint_transition = target_joint_pos - current_pos
                action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', joint_transition)])}
                observations = env.step(action_0)
                img_array.append(observations['robot_third_rgb'])
                count_steps += 1
                current_pos = env.sim.robot.arm_joint_pos
                ee_pose = env._sim.robot.ee_transform.translation # env._sim.ik_helper.calc_fk(np.array(current_pos))
                diff_j = np.sqrt(np.sum((target_joint_pos - current_pos)**2))
                diff_ee = np.sqrt(np.sum((target - ee_pose)**2))
                no_of_collisions = env.get_metrics()['robot_collisions']['total_collisions']
                print("EE distance: {}\t Joint Distance: {}\tCollisions: {}\tEE_pose: {}".format(diff_ee, diff_j, no_of_collisions, ee_pose))
                print("Current joint pos: {}".format(current_pos))
            # Now release obj
            if diff_ee<0.08 and (not env.episode_over):
                action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', np.zeros(shape=7, dtype=float)), ('grip_action', np.array([-0.91156936], dtype=float32))])}
                observations = env.step(action_0)
                img_array.append(observations['robot_third_rgb'])
                count_steps += 1
                gripped = False
                print("Object RELEASED -------------------------------------------------------------------------------------------------------")
                c = 10
                #while not env.episode_over and c>0:
                #    c -= 1
                #    action_0 = {'action': 'ARM_ACTION', 'action_args': OrderedDict([('arm_action', np.zeros(shape=7, dtype=float))])}
                #    observations = env.step(action_0)
                #    img_array.append(observations['robot_third_rgb'])
                #    count_steps += 1


        # Write and save the video file
        convert_to_video_and_save(img_array)


if __name__ == "__main__":
    vanilla_pick_place()
