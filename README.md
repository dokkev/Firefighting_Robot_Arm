### Demo Video
[![1](http://img.youtube.com/vi/1XVxniZMIpI/0.jpg)](http://www.youtube.com/watch?v=1XVxniZMIpI)

This is a repository for Autonomous Fire Fighting Robot Arm project.
You can checkout insights about this project at my [portfolio](https://rubberdk.github.io/firefigther-robot/)
Refer to the packages for more details for codes.

Unfortunately, I cannot share source codes for HDT Adroit 6DOF A24 Pincer Manipulation. If you have a question about the robot, please contact [Northwestern University MSR Program](https://www.mccormick.northwestern.edu/robotics/)


# Overview

## `control` package
`control` package takes care of controling the robot arm along with communication between the user and the robot.
Also, it provides serveral `rosservices` to control the robot

## `thermal_image_processing` package

`thermal_image_processing` package detects the fire (heat source) using `FLIR Lepton 2.5` thermal imaging module.


### Update on `thermal_image_processing`
This package is updated for better sensing of fire (heat source). It can utilize `intel realsense D435` to localize the fire by combing thermal image, aligned color to depth image, and point cloud. You can checkout more detial at [Fire_3D_Tracking](https://github.com/dokkev/Fire_3D_Tracking) repository!