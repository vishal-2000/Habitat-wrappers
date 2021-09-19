#!/usr/bin/env python3

import habitat

from habitat.tasks.rearrange.utils import IkHelper


def example():
    # Note: Use with for the example testing, doesn't need to be like this on the README

    with habitat.Env(
        config=habitat.get_config(
            "Vishal_tests/configs/rearrangepick_replica_cad.yaml"
        )
    ) as env:
        print("Environment creation successful")
        observations = env.reset()  # noqa: F841

        print("Agent acting inside environment.")
        count_steps = 0
        import cv2
        height, width, channels = observations['robot_head_rgb'].shape
        sizee = (width, height)
        img_array = []


        import numpy as np
        env._sim.ik_helper = IkHelper(
                    env._sim.habitat_config.IK_ARM_URDF,
                    np.array(env._sim.robot.params.arm_init_params),
                )


        while not env.episode_over:
            observations = env.step(env.action_space.sample())  # noqa: F841
            #print("Robot head rgb: {}".format(observations['robot_head_rgb'].shape))
            img_array.append(observations['robot_head_rgb'])
            print("IK status: {}".format(env._sim.ik_helper.calc_ik([1, 2, 1])))
            count_steps += 1
        out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, sizee)
        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()
        print("Sample action: {}".format(env.action_space.sample()))
        print("Action space: {}".format(env.action_space))
        print("Episode finished after {} steps.".format(count_steps))
        print("Object ids in the scene: {}".format(env._sim.scene_obj_ids))


if __name__ == "__main__":
    example()
