#ifndef adroit_interface_h
#define adroit_interface_h

#include <can_msgs/Frame.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <map>
#include <mutex>
#include <queue>

// custom message definitions for this node
#include "hdt_adroit_control/HDTDebugLSTelem.h"
#include "hdt_adroit_control/HDTDebugHSTelem.h"
#include "hdt_adroit_control/HDTLSTelem.h"
#include "hdt_adroit_control/HDTMSTelem.h"
#include "hdt_adroit_control/HDTHSTelem.h"

#define MAX_DRIVES  127
// message types
#define STATE_CMD 0
#define ERROR_TELEM 1
#define HIGH_SPEED_TELEM 3
#define CONTROL_CMD 4
#define MEDIUM_SPEED_TELEM 5
#define IMPEDANCE_CMD 6
#define LOW_SPEED_TELEM 7
#define DEBUG_TELEM 9
#define CURRENT_CMD 10
#define PARAMETER_TELEM 11
#define PARAMETER_CMD 12
#define STATUS_TELEM 14
#define STATUS_CMD 15

class AdroitInterface : public hardware_interface::RobotHW
{
public:
  AdroitInterface(std::string robot_name, bool fake_execution=false);
  ~AdroitInterface();
  
  bool init(ros::NodeHandle& nh);
  bool read();
  bool write();
  bool reset();
  
private:
  std::string robot_name_;
  
  void msgCallback(const can_msgs::Frame::ConstPtr& msg);
  
  bool processMsg(const can_msgs::Frame *msg);
  
  std::map<std::string, ros::Subscriber> msg_sub_;
  
  std::map<std::string, ros::Publisher> msg_pub_;
  
  hardware_interface::JointStateInterface state_interface_;
  hardware_interface::PositionJointInterface pos_interface_;
  hardware_interface::VelocityJointInterface vel_interface_;
  
  std::map<int, bool> addr_map_;
  std::map<int, std::string> namspace_map_;
  std::vector<std::string> namespace_list_;
  
  std::map<int, std::string> addr_to_name_map;
  std::map<std::string,int> name_to_add_map;
  
  double cmd_pos_[MAX_DRIVES];
  double cmd_vel_[MAX_DRIVES];
  double pos_[MAX_DRIVES];
  double vel_[MAX_DRIVES];
  double eff_[MAX_DRIVES];
  
  int drive_type_[MAX_DRIVES];
  
  enum joint_modes
  {
    NONE_MODE,
    JOINT_POSITION_MODE,
    JOINT_VELOCITY_MODE
  };
  joint_modes joint_mode_current[MAX_DRIVES];
  
  //hardware_interface::JointModeHandle mode_handle_[MAX_DRIVES];
  
  bool fake_execution_;
  bool control_enabled_;
  
  std::mutex mutex_;
  std::queue<can_msgs::Frame> msg_queue_;
  
  // variables for HDT telemetry
  bool hdt_telem_enable_;
  std::map<std::string, ros::Publisher> hdt_telem_pub;
  // assume that only drive is sending debug telemtry at a time
  uint8_t debug_telem_buffer[128];
  uint8_t debug_buffer_bytes;
  
  // variables for services 
  bool hdt_service_enable_;
  // parameter read-write, status, state change, appload
  
  
  bool read_drive_mode(int addr);
  bool set_drive_mode(int addr, joint_modes new_mode);

};

#endif // adroit_interface_h

