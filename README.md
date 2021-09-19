# Habitat-wrappers
Wrapper classes and manipulator tests for Habitat simulator

Warning!!!: Bleeding-edge code (Please check the code files and correct them where and when necessary based on your system and usage requirements)

## TODO
1. [x] Wrapper class for Franka Panda
2. [x] Change the orientation of franka panda base 
3. [x] Load panda in a scene
4. [ ] Load various objects and place them in appropriate positions
5. [x] Simple pick and place with Fetch with or without a realistic scene 
6. [ ] Generic IK planner integration
7. [x] Add IK planner developed by Dr. Arun and Prajwal
8. [ ] Train DeLan on Habitat-sim

## Joint limits
- (taken from Panda urdf) (for testing purposes)
- Arm joint lower limits = ([-2.8973000049591064, -1.7627999782562256, -2.8973000049591064, -3.0717999935150146, -2.8973000049591064, -0.017500000074505806, -2.8973000049591064]

- Arm joint upper limits= ([2.8973000049591064, 1.7627999782562256, 2.8973000049591064, -0.0697999969124794, 2.8973000049591064, 3.752500057220459, 2.8973000049591064])

## Habitat-basic-lab-work
- pick_and_place2.py contains all the helper functions
- Use pick_and_place2.py as a template or reference code to write your own code files as it contains almost all the important APIs (including those that are commented within the code). Occasionally, refer to example2.py too as it may contain one or two APIs that are absent in pick_place2.py
