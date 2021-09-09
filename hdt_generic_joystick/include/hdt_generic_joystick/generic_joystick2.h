#ifndef HDT_GENERIC_JOYSTICK2
#define HDT_GENERIC_JOYSTICK2

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_model/joint_model_group.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <moveit_msgs/DisplayRobotState.h>

#include <map>
#include <mutex>
#include <vector>

#include <hdt_generic_joystick/adroit_constants.h>

#define LOAD_CONTROLLER "/controller_manager/load_controller"

// joint_limits_interface
#include <joint_limits_interface/joint_limits_interface.h>

// future expansion - look at adding interactive markers to joints for on screen control
//#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

//JD additions
#include <hdt_generic_joystick/MsgWrapper.h>
#include <hdt_generic_joystick/JoystickMap.h>

#include <stdlib.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

class JoystickControl
{
public:
  enum Mode {
    MODE_NONE,
    MODE_NAMED,
    MODE_JOINT_POSITION,
    MODE_JOINT_VELOCITY,
    MODE_ENDPOINT,
    MODE_ENDPOINT_EE,
    MODE_RESET,
    NUM_MODES,
  };
  std::vector<std::string> Mode_Names_ = {
    "MODE_NONE",
    "MODE_NAMED",
    "MODE_JOINT_POSITION",
    "MODE_JOINT_VELOCITY",
    "MODE_ENDPOINT",
    "MODE_ENDPOINT_EE",
    "MODE_RESET",
    "NUM_MODES"
  };
    
  JoystickControl(const ros::NodeHandle& nh);
  ~JoystickControl();
  bool init();
  void run();
  Mode switchMode(const Mode& new_mode);


private:

  std_msgs::ColorRGBA joystick_display_colors[NUM_MODES];

  void joyCallback(const sensor_msgs::Joy& msg);
  void spaceCallback(const geometry_msgs::Twist& msg);
  void connectionCallback(const std_msgs::Bool& msg);

  void jointstateCallback(const sensor_msgs::JointState& msg);
  void updateCollision(robot_state::RobotState& current_state);
  void jointByJointControlPosition(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene);
  void jointByJointControlVelocity(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene);
  void endPointControl(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene);
  void endPointControl_EE(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene);
  std::vector<double> processControlInputInternal(const moveit::core::JointModelGroup* jModelGroup);
  std::vector<double> processControlInputInternalEE();
  void publishPositionCommands(std::vector<std::string>& name_vector,  std::vector<double>& command_vector);
  void publishVelocityCommands(std::vector<std::string>& name_vector,  std::vector<double>& command_vector);
  void dualEndPointControl(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene,geometry_msgs::Twist input1);
  void dualEndPointControlInternal(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene,geometry_msgs::Twist input1, std::vector<std::pair<moveit::core::JointModelGroup*, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>> working_groups, moveit::core::JointModelGroup* original_joint_model_group);
  bool updateJointState();
  ros::NodeHandle nh_;

  std::mutex mutex_;

  ros::Subscriber joy_sub_;
  sensor_msgs::Joy joy_msg_, joy_prev_;
  ros::Subscriber space_sub_;
  geometry_msgs::Twist space_holder_, space_prev_;
  TwistWrapper space_msg_;
  JoyWrapper joy_wrap_msg_;
  JointStateWrapper joint_state_;
  
  ros::Subscriber connection_sub_;
  
  bool use_spacemouse;
  bool manipulator_connected;
  bool collision_enable;
  //dictionary of joystick and button mappsing
  std::map<std::string, int> button_map;
  JoystickMap joystick_map_;


  // moveit
  robot_model_loader::RobotModelLoader* model_loader;
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotState* kinematic_state;
  // control groups (joint group and move groups)
  std::vector<moveit::core::JointModelGroup*> joint_model_groups_;
  int joint_group_index;
  moveit::core::JointModelGroup* current_joint_model_;
  std::vector<moveit::planning_interface::MoveGroupInterfacePtr> move_groups_;
  std::map<std::string, std::string> end_effector_map_;
  moveit::core::JointModelGroup* EE_group;
  
	std::map<std::string,moveit::core::VariableBounds> joint_bounds_;
	std::vector<std::string> named_targets;
	int named_target_index;
	
	

  // vectors to store joint names in arm and end effector
  std::vector<std::string> joint_names_;
  std::vector<std::string> EE_names_;

  std::size_t num_joints_;
  std::size_t num_EE_;


  std::vector<double> joint_positions_, joint_commands, EE_positions, EE_commands;
  
  // vectors to store actuator bounds (position limits) as described in URDF model
  std::vector<moveit::core::VariableBounds> joint_bounds;
  std::vector<moveit::core::VariableBounds> EE_bounds;

  geometry_msgs::PoseStamped endpoint_pose_;

  std::vector<std::string> joint_controllers_;

  std::map<std::string, ros::Publisher> joint_position_pubs_;
  std::map<std::string, ros::Publisher> joint_velocity_pubs_;
  
  //ros::Publisher pincer_pub_;

  Mode mode_;
  
  
  std::mutex joint_state_mutex_;
  ros::Publisher display_pub_;
  ros::Publisher pose_pub_;
  // marker for displaying joystick node status in Rviz
  ros::Publisher status_marker_pub_;
  visualization_msgs::Marker status_marker;
  ros::Subscriber joint_state_sub_;
  sensor_msgs::JointState joint_state_last_;
};

#endif // HDT_GENERIC_JOYSTICK2
