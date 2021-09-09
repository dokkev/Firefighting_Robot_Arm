# hdt_adroit

The `hdt_adroit` package provides a communication interface to a generic HDT arm. The `hdt_adroit_control` package implements both [`joint_state_controller/JointStateController`](http://wiki.ros.org/joint_state_controller) and [`position_controllers/JointTrajectoryController`](http://wiki.ros.org/joint_trajectory_controller) controllers, for compatibility with [MoveIt!](http://moveit.ros.org/).

The package depends on several configuration parameters, which are provided by the arm's ROS package. A typical launch file for an arm package called `hdt_robot` is shown below:

```xml
<launch>
  <arg name="debug" default="false" />
  <arg name="fake_execution" default="false" />
  
  <!-- load robot description -->
  <param name="robot_description"
    command="$(find xacro)/xacro.py '$(find hdt_robot_description)/urdf/hdt_robot.xacro'" />

  <!-- load the hardware configuration -->
  <rosparam command="load" file="$(find hdt_robot_description)/config/hardware.yaml" />
  
  <!-- load the controller configuration -->
  <rosparam command="load" file="$(find hdt_robot_description)/config/controllers.yaml" />
  
  <!-- load the serial node -->
  <node name="socket_node" pkg="rosserial_server" type="socket_node" respawn="false" output="screen" unless="$(arg fake_execution)">
  </node>

  <!-- load the hardware interface -->
  <node name="adroit_control" pkg="hdt_adroit_control" type="adroit_control" respawn="false" output="screen">
    <param name="robot_name" value="hdt_robot" /> 
    <param name="fake_execution" value="$(arg fake_execution)" /> 
  </node>

  <!-- load the controllers -->
  <node name="controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="spawn 
  /hdt_robot/joint_state_controller 
  /hdt_robot/arm_controller"/>
  
  <!-- publish tf for the robot links -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />
</launch>
```

The `hardware.yaml` file referenced above should resemble the following (for a five joint arm):

```yaml
hdt_robot:
  hardware_interface:
    loop_hz: 50
    joints:
      - drive1_joint
      - drive2_joint
      - drive3_joint
      - drive4_joint
      - drive5_joint
    drive1_joint:
      addr: 1
    drive2_joint:
      addr: 2
    drive3_joint:
      addr: 3
    drive4_joint:
      addr: 4
    drive5_joint:
      addr: 5
```

And a typical `controllers.yaml` file (for a five joint arm) should be of the form:

```yaml
hdt_robot:
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50
  arm_controller:
    type: position_controllers/JointTrajectoryController
    state_publish_rate: 50
    joints:
      - drive1_joint
      - drive2_joint
      - drive3_joint
      - drive4_joint
      - drive5_joint
```

