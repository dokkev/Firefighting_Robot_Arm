#ifndef hdt_adroit_console_h
#define hdt_adroit_console_h

#include <ros/ros.h>
#include <can_msgs/Frame.h>

// static value and message structure definitions
#include <hdt_adroit_debug/AdroitComs.h>
#include <hdt_adroit_debug/adroit_params.h>

// custom service definitions for this node
#include "hdt_adroit_debug/ReadDriveParam.h"
#include "hdt_adroit_debug/ReadAllDriveParam.h"
#include "hdt_adroit_debug/WriteDriveParam.h"
#include "hdt_adroit_debug/Commit.h"
#include "hdt_adroit_debug/StateSelect.h"
#include "hdt_adroit_debug/GetStatus.h"

#define CALLBACK_DEBUG true

class AdroitConsole: public AdroitComs {


public:
  AdroitConsole() : AdroitComs(0) {};
  //~AdroitConsole();
  bool init();
  
  //int Init(void) = 0;
  int SendMsg(AdroitMsg *adroit_msg);
  //int ReceiveMsg(AdroitMsg *msg) = 0;
  int Write(void);

private:
  std::map<std::string ,int> drive_names;
  AdroitComs::AdroitDrive drive_data[MAX_DRIVES];
  
  uint16_t ParameterIndexListLocal[ParameterListLength];
  std::vector<std::string> ParameterNameListLocal;
  uint8_t ParameterDataTypeListLocal[ParameterListLength];
  
  ros::Publisher CAN_pub;
  void CANmessageCallback(const can_msgs::Frame::ConstPtr& msg);
  
  /*
  
  bool process_parameter_telem(AdroitComs::ParameterMsg *telem, const can_msgs::Frame::ConstPtr& msg);
  void send_parameter_cmd(uint8_t addr, AdroitComs::ParameterMsg *cmd);
  int read_parameter(int addr, int index);
  */
  bool ReadParamServiceCallback(hdt_adroit_debug::ReadDriveParam::Request& req, hdt_adroit_debug::ReadDriveParam::Response& res);
  bool ReadAllParamServiceCallback(hdt_adroit_debug::ReadAllDriveParam::Request& req, hdt_adroit_debug::ReadAllDriveParam::Response& res);
  bool WriteParamServiceCallback(hdt_adroit_debug::WriteDriveParam::Request& req, hdt_adroit_debug::WriteDriveParam::Response& res);
  bool WriteAllParamServiceCallback(hdt_adroit_debug::ReadAllDriveParam::Request& req, hdt_adroit_debug::ReadAllDriveParam::Response& res);
  bool CommitServiceCallback(hdt_adroit_debug::Commit::Request& req, hdt_adroit_debug::Commit::Response& res);
  bool StateSelectServiceCallback(hdt_adroit_debug::StateSelect::Request& req, hdt_adroit_debug::StateSelect::Response& res);
  bool GetStatusServiceCallback(hdt_adroit_debug::GetStatus::Request& req, hdt_adroit_debug::GetStatus::Response& res);
  // helper functions
  bool ResolveCANid(std::string input_drive_name, uint8_t input_CAN_ID, std::string *return_drive_name, uint8_t *return_CAN_ID);
  bool ResolveParameter(std::string input_param_name, uint16_t input_param_num, std::string *return_param_name, uint16_t *return_param_num, uint8_t *return_param_type);
};

#endif // hdt_adroit_console_h
