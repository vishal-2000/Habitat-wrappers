import habitat
from habitat.sims.habitat_simulator.actions import HabitatSimActions
import cv2 

FORWARD_KEY="f"
LEFT_KEY="l"
RIGHT_KEY="r"
FINISH="q"

def transform_rgb_bgr(image):
    return image[:, :, [2, 1, 0]] 

def example():
    env = habitat.Env(
        config=habitat.get_config("configs/tasks/pointnav.yaml")
    )
    print("Environment created successfully") 

    observations = env.reset()

    print("Destination, distance: {:3f}, theta(radians): {:.2f}".format(
        observations["pointgoal_with_gps_compass"][0],
        observations["pointgoal_with_gps_compass"][1])) 

    cv2.imshow("RGB", transform_rgb_bgr(observations["rgb"]))
    print("Agent is stepping around inside the environment.") 

    from time import sleep
    sleep(1)
    exit()

    count_steps = 0  #variable to keep track of number of steps
    while not env.episode_over:  #while the dataset is not empty
        keystroke = cv2.waitKey(0)
        #Move forward if user presses ‘f’ key
        if keystroke == ord(FORWARD_KEY):   
            action = HabitatSimActions.MOVE_FORWARD
            print("action: FORWARD")
        #Move left if user presses ‘l’ key
        elif keystroke == ord(LEFT_KEY):
            action = HabitatSimActions.TURN_LEFT
            print("action: LEFT")
        #Move right if user presses ‘r’ key
        elif keystroke == ord(RIGHT_KEY):
            action = HabitatSimActions.TURN_RIGHT
            print("action: RIGHT")
        #Agent should stop if user presses ‘q’ key
        elif keystroke == ord(FINISH):
            action = HabitatSimActions.STOP
            print("action: FINISH")
        #Display proper message if user presses any key other than ‘f’, 
        #  ‘l’, ‘r’ or ‘q’*/
        else:
            print("INVALID KEY")
            continue 
        
        observations = env.step(action)
        count_steps += 1

        print("Destination, distance: {:3f}, theta(radians): {:.2f}".format(
            observations["pointgoal_with_gps_compass"][0],
            observations["pointgoal_with_gps_compass"][1])) 

        cv2.imshow("RGB", transform_rgb_bgr(observations["rgb"]))
        print("Episode finished after {} steps.".format(count_steps))

    if (
        action == HabitatSimActions.STOP
        and observations["pointgoal_with_gps_compass"][0] < 0.3
    ):
        print("you successfully navigated to destination point")
    else:
        print("your navigation was unsuccessful") 

if __name__ == "__main__":
    example() 