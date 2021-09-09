### Demo Video
[![1](http://img.youtube.com/vi/1XVxniZMIpI/0.jpg)](http://www.youtube.com/watch?v=1XVxniZMIpI)


# BACKGROUND & MOTIVATION
The idea of a firefighting robot is not a new concept, and there have been a lot of propositions of using firefighter robots. In fact, a robot called Colossus helped save much of the historic structure by braving conditions deemed too dangerous for human firefighters after Notre Dame Cathedral in Paris caught fire in 2019 [1], and LA Fire Department tested autonomous fire fighting robots in the major Downtown blaze in 2020 [2].

Firefighting Robot "Colossus"
<div class="post-flex-display">
    <img src="/img/colossus.jpg" alt="colossus">
</div>


The problems with currently existing firefighting robots are that they are not suitable for domestic or commercial usages because they are big and built specifically to operate in firefighting missions. I believe that manipulating a robot arm to detect a fire and use a fire extinguisher can be beneficial in industrial settings such as warehouses that have already adapted using robot arms. Especially, maintenance costs of fire sprinkles due to frozen pipes can be deducted in warehouses. Additionally, implementing a fire safety feature to domestic service robot arms can appeal to the general crowd.

Therefore, I implemented a firefighting mechanism using HDT Adroit 6 DOF A24 Pincer robot arm (I will refer it as an Adroit through out this post) and FLIR Lepton 2.5 thermal camera for this project. The goal of this project is to detect a heat source (fire), grab the fire extinguisher, and operate it.

# HARDWARES
HDT Adroit 6DOF A24 Pincer
![fire_extinguisher](https://github.com/rubberdk/Firefighting_Robot_Arm/blob/master/images/adroit.jpg?raw=true)



Fixed-hose fire extinguisher
![fire_extinguisher](https://github.com/rubberdk/Firefighting_Robot_Arm/blob/master/images/fe.jpg?raw=true)



FLIR Lepton 2.5 - Thermal Imaging Module
![thermal_camera](https://github.com/rubberdk/Firefighting_Robot_Arm/blob/master/images/lepton2.5.jpg?raw=true)



PureThermal 2 - FLIR Lepton Smart I/O Board
![purethermal](https://github.com/rubberdk/Firefighting_Robot_Arm/blob/master/images/purethermal2.jpg?raw=true)



# SYSTEM OVERVIEW

The Adroit operates on ROS. I used `ROS-noetic` version for this project. 

<div class="post-flex-display">
    <img src="/img/block.jpg" alt="block_diagram">
</div>

This block diagram shows the overall flow and communication between different ROS packages for the project. I implemented two packages which are `thermal_image_processing` and `control`. Besides, I used packages provided by HDT to control the basic movements of the robot.

# CODES COMPONENTS EXPLANATION

## Package: `thermal_image_processing`
This package takes care of detecting a highest spot in the camera view and reading its temperature from the thermal camera. I implemented the package from PureThermal 1 / PureThermal 2 FLIR Lepton Dev Kit (https://github.com/groupgets/purethermal1-uvc-capture).

### Thermal Image Processing

<div class="post-flex-display">
    <img src="/img/self.jpg" alt="example">
</div>

*Image of a human through a thermal camera*

FLIR Lepton 2.5 thermal camera can detect the temperature using its radiometry in a gray scale. The pixel value in the gray scale, for example, shows some value such as 40,000 which represents 400.00 K. The node `thermal_detection` detects the pixel with the highest temperature and convers into celcious scale. In addtion, it publishes the temperature value and 2D x-y coordinates of the image as `rostopic` so that the Adroit can execute its movements based on those values.


## Package: `control`
This package controls the Adroit's arm joints through moveit and pincer through the pincer joint controller.
I have implemented the node `arm_control` for Adroit to detect the fire, grab the fire extiguisher, aims it to the fire, and press the lever.

### Fire Detection
`arm_control` node subscribes to the temperature value and x-y coordinates that `thermal_detection` is publishing.

<div class="post-flex-display">
    <img src="/img/search.gif" alt="search">
</div>

*Adroit Searching Mode*


<div class="post-flex-display">
    <img src="/img/joints.jpg" alt="joints">
</div>

*Joints Movements*


The Adroit makes joint movements on its joint 1 and 6 to scan the enviroment until it detects the something with a high temperature value. The threshold for the temperature was set to 70 deg C (~156 deg F) for this project since I used a heater as my heat source, but this value would have been set higher if it were to work with a real fire.


<div class="post-flex-display">
    <img src="/img/search2.gif" alt="search2">
</div>

*Thermal Camera View*


<div class="post-flex-display">
    <img src="/img/algo.jpg" alt="algo">
</div>

*Align Mechanism*

When the Adroit detects something above 70 deg C, it will try to align the center of the camera view to the heat source by adjusting joint 1 and 3. The figure above represets its mechanism. When the robot aligns wit the heat source, it remembers joints positions so that Adroit can comeback to this position after grabbing the fire extinguisher. In this case, the Adroit's joint 1 is a little bit smaller than remebered joint position because the fire extingushier will be located slightly left to the thermal camera.


### Grab & Aim

<div class="post-flex-display">
    <img src="/img/attach.jpg" alt="attach">
</div>

*Safey pins and laser pointer*

Since I am working with a single arm robot, it is necessary for the picners to be able to two tasks: picking up the fire extinguisher & pressing the lever. Therfore, I implemented custom safey pins to prevent the Adroit to accidently press the lever when it is trying to grab the fire extingshiher. I made two types of the pins that the first one is an electric wire coverd by rubber, and the other one is 16-gauge copper wire. The copper wire one is more sturdy that the Adorit needs to exert more force to press the lever compared to the rubber one. I attached anti-slip stickers on the levers of the fire-extinguisher to get better grip

I used a hard coordinate for the location of the fire extinghisher, and I found this reasonable because fire extinguihsers are usually located at specific and fixed spots. The Adroit sucessfully grabs the fire extinguisher in a rate of 8 out of 10 times, and this error was due to incorrect orientation of the fire extinguisher.

<div class="post-flex-display">
    <img src="/img/aim.gif" alt="aim.gif">
</div>


I installed a laser pointer to accurately evaluate the Adroit's aim to the heat source. When the Adorit goes back to its remembered joint positions, the fire extingusher hose's aim is aligned to the heat source.

### Press

<div class="post-flex-display">
    <img src="/img/press_laser.gif" alt="press.gif">
</div>

`arm_control` publishes `std_msgs/Float64` to `picner_poistion_controller` to control the pincers.
When the Adroit presses the lever, the fire extinguisher's aim goes down a little bit. Besides, this project achieved its goal.
