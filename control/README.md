# Control Package Detail

## NODES: `arm_control`
his package creates enviroment for Moveit and controls the arm and pincer movement based on information published from `thermal_image_processing` package

### SUBSCRIBER & PUBLISHER

Subscriber:
- object_y (std_msgs/Float32): y-coordinate of the heat source in 2D thermal image 
- object_x (std_msgs/Float32): x-coordinate of the heat source in 2D thermal image 
- highest_T_detected (std_msgs/Float32): the highest tempertaure detected in the thermal image

Publisher:
- /hdt_arm/pincer_joint_position_controller/command (std_msgs/Float64): angle for the pincer postion controller


### SERVICES

- `rosservice call /ready` : It moves the robot to its `ready` position.
- `rosservica call /grab_ready` : It moves the robot to its `grab_ready` position.
- `rosservica call /search` : It scans the robot's enviroment and executes joint movements to align the center of the camera view with the heat source, and it remembers the robot's current joint positions.
- `rosservica call /grab` : It grabs the fire extingushier.
- `rosservica call /locate`: It moves to stored joint positions from `search`. This service is for to aim the fire with the fire extinguisher after grabbing it.
- `rosservica call /press` : It closes the grippers so that it will press the lever of the fire extinguisher.
- `rosservica call /open` : It opend its grippers wide. 

## LAUNCH

- `hdt_arm_bringup.launch` starts the robot
- `fire_fight.launch` starts `arm_control` node and `fire_detect.launch` from the `thermal_image_processing` package

