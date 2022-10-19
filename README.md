# Overview 
This ROS node is responsible for generating formation positions (formation generator and consensus path algos). It listens from the mode_manager node for "Go_There", "Follow_Me" and agent numbers for that command.

# Code structure 
The code structure is quite messy, the guys from DSO had a sample code and I modified quite a bit to make it work in Borealis. There is only 2 header (1 unused) files despite there being more .cpp files. There is only 1 class in this folder and that is the TeamingPlanner class.

- teaming_planner.h contains TeamingPlanner class and function declerations. 
- teaming_planner_constants.h is legacy code that is probably unused. 
- teaming_planner.cpp contains TeamingPlanner class code initialization(subscribe, publish, params,etc) and the ros loop after finish initializing.
- teaming_planner_main contains the main state machine logic that is running in the ros loop
- teaming_planner_node.cpp calls the teaming planner class and initialize rosnode
- teaming_planner_middleware.cpp contains function definitions for teaming planner class. 
- teaming_planner_controller.cpp is legacy code. The old planner is called via ros::Timer() function that triggers every x times. I feel this is a bit weird so i made it to run in a ros loop instead. 
- common_bind_funcs.cpp contains common functions to be binded by both the consensus path planner and the formation generator.
- formation_bind_funcs.cpp contains class functions to be binded by the formation generator
- consensus_path_bind_funcs.cpp contains functions to be binded by the consensus path

Each of the two algorithm contains 2 classes such as mDistributedFormation and mDistributedFormationHandler. One is the handler class and the other is the main class. The handler class contains functions that is to be binded. The main class initializes params and takes in mDistributedFormationHandler for the custom funcs, the main algorithm is also here.

# Params

- Params are adjusted in /config folder
- Launch file does not adjust any params other than rostopic names 

# To be improved

- The node will crash if the tf lookup tree isn't completed, the try catch except block should have sort this out but not working
- The convex hull flickers in rviz (empty convex hull is being published??)
- During physical demo, sometimes it has trouble working when go there positions is in / near obstacles ??

# The main repo that contains the inner working algorithms is private
