# hdt_6dof_a24_pincer
control packages for 6DOF manipulator with pincer, using A24 actutators

### Initial setup

0. Install [ROS Melodic](http://wiki.ros.org/ROS/Installation) on [Ubuntu 18.04](http://releases.ubuntu.com/18.04/)

Follow directions [here](http://wiki.ros.org/melodic/Installation/Ubuntu)

1. Create catkin workspace

```console
$ source /opt/ros/melodic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace .
$ cd ..
$ catkin_make
```

2. Copy this repositories into `src` folder

 ~/catkin_ws/src/


3. Install required packages and build

```console
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make
```

### Running the system

Use the following command to start the system in simulation:

```console
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch hdt_6dof_a24_pincer_bringup hdt_arm_bringup_1.launch simulation:=true controller_type:=xbox
```

This will start the `hdt_adroit_control` interface, bring up an `RViz` visualization of the system, and allow users to control the arm with an example joystick interface. The simulation argument accepts true/false arguments; true connects to the hardare, false loops back the commanded position as the current position in the `/joint_states` topic. The controller_type argument accepts a string corresponding to a hdt_6dof_a24_pincer_description/config/joystick_map_*.yaml parameter file which maps control functions to buttons and axes in the `/joy` topic. Support is provided for xbox and ds4 joysticks.

