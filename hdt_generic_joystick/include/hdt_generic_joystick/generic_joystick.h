#ifndef HDT_GENERIC_JOYSTICK
#define HDT_GENERIC_JOYSTICK

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_model/joint_model_group.h>
#include <sensor_msgs/Joy.h>

#include <map>
#include <mutex>
#include <vector>

#define NUM_AXES        8

#define NAMED_BUTTON    0
#define JOINT_BUTTON    1
#define ENDPOINT_BUTTON 2
#define NUM_BUTTONS     3

#define MAX_VEL         0.12 // Only used for Endpoint at present.  Best to keep low!  Real max is for each actuator is 0.7854
#define VEL_SCALE       0.5 
#define LOOP_RATE       20.0
#define JOINT_SCALE     VEL_SCALE/LOOP_RATE // MAX_VEL*VEL_SCALE/LOOP_RATE
#define LINEAR_SCALE    0.1
#define ANGULAR_SCALE   MAX_VEL
//#define JOY_TOPIC       "hdt_arm/joy"

#define TRAJ_CONTROLLER "/hdt_arm/arm_controller"

#define MOVE_GROUP      "arm"
#define ENDPOINT_LINK   "endpoint_link"

#define LOAD_CONTROLLER "/controller_manager/load_controller"

// joint_limits_interface
#include <joint_limits_interface/joint_limits_interface.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

class JoystickControl
{
public:
  enum Mode {
    MODE_NONE = 0,
    MODE_NAMED = 1,
    MODE_JOINT = 2,
    MODE_ENDPOINT = 3,
    NUM_MODES
  };
  JoystickControl(const ros::NodeHandle& nh);
  ~JoystickControl();
  bool init();
  void run();
  Mode switchMode(const Mode& new_mode);


private:
  void joyCallback(const sensor_msgs::Joy& msg);
  void setValues(const std::vector<double>& joint_values);
  
  PositionJointInterface pos_cmd_interface_;
  PositionJointSoftLimitsInterface jnt_limits_interface_;
  
  ros::NodeHandle nh_;

  std::mutex mutex_;

  ros::Subscriber joy_sub_;

  ros::Publisher joy_pub_;

  sensor_msgs::Joy joy_msg_, joy_prev_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  
  const moveit::core::JointModelGroup* joint_model_group_;

  // vectors to store joint names in arm and end effector
  std::vector<std::string> joint_names_;
  std::vector<std::string> EE_names_;

  std::size_t num_joints_;
  std::size_t num_EE_;

  std::vector<double> joint_values_;

  // vectors to store actuator bounds (position limits) as described in URDF model
  std::vector<moveit::core::VariableBounds> joint_bounds;
  std::vector<moveit::core::VariableBounds> EE_bounds;

  geometry_msgs::PoseStamped endpoint_pose_;
  
  double pincer_value_;

  std::vector<std::string> joint_controllers_;

  std::map<std::string, ros::Publisher> joint_pubs_;
  
  ros::Publisher pincer_pub_;

  Mode mode_;
};

#endif // HDT_GENERIC_JOYSTICK
