#define _USE_MATH_DEFINES

#include <hdt_adroit_control/adroit_interface.h>

#include <math.h>
#include <sstream>

/*
// message types
static const int ERROR_TELEM = 1;
static const int HIGH_SPEED_TELEM = 3;
static const int CONTROL_CMD = 4;
*/

// fixed point conversions
static const double POSITION_CONV = 8.0*M_PI;
static const double VELOCITY_CONV = 8.0;
static const double EFFORT_CONV = 256.0;
static const double CURRENT_CONV = 64.0;
static const double VOLTAGE_CONV = 64.0;
static const double TEMPERATURE_CONV = 128.0;
/*----------------------------------------------------------------------------
 * constructor
 *----------------------------------------------------------------------------*/
AdroitInterface::AdroitInterface(std::string robot_name, bool fake_execution): robot_name_(robot_name), fake_execution_(fake_execution), control_enabled_(false)
{
  // initialize data
  for (int addr = 0; addr < MAX_DRIVES; addr++)
  {
    cmd_pos_[addr] = 0.0;
    cmd_vel_[addr] = std::nanf(""); // assume joint starts in position mode
    pos_[addr] = 0.0;
    vel_[addr] = 0.0;
    eff_[addr] = 0.0;
    
    // assume joint starts in position mode
    joint_mode_current[addr] = NONE_MODE;
  }
}

/*----------------------------------------------------------------------------
 * destructor
 *----------------------------------------------------------------------------*/
AdroitInterface::~AdroitInterface()
{
}

/*----------------------------------------------------------------------------
 * msg callback
 *----------------------------------------------------------------------------*/
void AdroitInterface::msgCallback(const can_msgs::Frame::ConstPtr& msg)
{
  //ROS_INFO("AdroitInterface:msgCallback");
  
  // lock mutex
  std::lock_guard<std::mutex> lock(mutex_);
  
  // push to msg queue
  msg_queue_.push(*msg);
}

/*----------------------------------------------------------------------------
 * init
 *----------------------------------------------------------------------------*/
bool AdroitInterface::init(ros::NodeHandle& nh)
{ 
  //ROS_INFO("AdroitInterface:init robot name = %s", robot_name_.c_str());
  
  ros::NodeHandle robot_nh(nh, robot_name_);
  
  // get joints for given robot
  std::vector<std::string> joints;
  if (!robot_nh.getParam("hardware_interface/joints", joints))
  {
    ROS_ERROR("AdroitInterface:init could not get joints param");
    return false;
  }
  
  //ROS_INFO("AdroitInterface:init found %d joints", (int)joints.size());

  // iterate through joints
  for (std::vector<std::string>::const_iterator joint = joints.begin(); joint != joints.end(); ++joint)
  {
    // look for id param associated with joint
    std::stringstream addr_param;
    addr_param << "hardware_interface/" << *joint << "/addr";
    
    // check for valid addr
    int addr;
    if (robot_nh.getParam(addr_param.str(), addr))
    {
     if ((addr > 0) && (addr < MAX_DRIVES))
     {
        //ROS_INFO("AdroitInterface:init registering %s with id %d", joint->c_str(), id);
        // set enabled to false initially
        addr_map_[addr] = false;
        
        // connect and register state interface
        hardware_interface::JointStateHandle state_handle(*joint, &pos_[addr], &vel_[addr], &eff_[addr]);
        state_interface_.registerHandle(state_handle);

        // connect and register position interface
        hardware_interface::JointHandle pos_handle(state_interface_.getHandle(*joint), &cmd_pos_[addr]);
        pos_interface_.registerHandle(pos_handle);
        
        // connect and register velocity interface
        hardware_interface::JointHandle vel_handle(state_interface_.getHandle(*joint), &cmd_vel_[addr]);
        vel_interface_.registerHandle(vel_handle);
        
        /*
        // tyring to read joint modes - does not get populated by position or velocity controllers
        hardware_interface::JointCommandModes test_mode;
        hardware_interface::JointModeHandle mode_handle(*joint,&test_mode);
        mode_handle_[addr] = mode_handle;
        ROS_INFO("AdroitInterface:init mode handle getName %s",mode_handle_[addr].getName().c_str());
        ROS_INFO("AdroitInterface:init mode handle getModeName %s",mode_handle_[addr].getModeName(mode_handle_[addr].getMode()).c_str());
        */
        
        // build map of joint names and CAN ID's
        addr_to_name_map.insert(std::pair<int, std::string>(addr,*joint));
        name_to_add_map.insert(std::pair<std::string,int>(*joint,addr));
      }
      // report invalid id
      else
      {
        ROS_ERROR("AdroitInterface:init invalid addr = %d", addr);
        return false;
      }
    }
    else
    {
      ROS_ERROR("AdroitInterface:init could not find param %s", addr_param.str().c_str());
      return false;
    }

    // look for namespace param associated with joint
    std::stringstream namespace_param;
    namespace_param << "hardware_interface/" << *joint << "/namespace";

    // check for namespace
    std::string temp_namespace;
    if (robot_nh.getParam(namespace_param.str(), temp_namespace))
    {
      //ROS_INFO("AdroitInterface:init found param %s is %s", namespace_param.str().c_str(),temp_namespace.c_str());
      namspace_map_[addr] = temp_namespace;
      if(std::find(namespace_list_.begin(), namespace_list_.end(), temp_namespace) != namespace_list_.end()){
        // namespace is already in the list?
      }else{
        namespace_list_.push_back(temp_namespace);
      }
    }
    else
    {
      // namespace is optional for now, if not specfied in hardware_interface.yaml file assume there is no namespace
      // use empty string as place holder for now
      namspace_map_[addr] = "";
      if(std::find(namespace_list_.begin(), namespace_list_.end(), "") != namespace_list_.end()){
        // namespace is already in the list?
      }else{
        namespace_list_.push_back("");
      }
      //ROS_ERROR("AdroitInterface:init could not find param %s", namespace_param.str().c_str());
      //return false;
    }
  }
  
  // register interfaces
  registerInterface(&state_interface_);
  registerInterface(&pos_interface_);
  registerInterface(&vel_interface_);
  
  // link to hardware topics, or warn about fake execution
  if (fake_execution_)
  {
    ROS_WARN("AdroitInterface:init fake execution enabled");
  }    
  else
  {
    ROS_INFO("AdroitInterface:init creating topics for %d namespaces",(int)namespace_list_.size());
    for (std::vector<std::string>::iterator it = namespace_list_.begin() ; it != namespace_list_.end(); ++it){
      if(it->empty()){
        // handle non-namespaced drives
        // subscribers
        std::string temp_string;
        temp_string="/received_messages";
        ROS_INFO("\tsubscriber %s in namespace %s",temp_string.c_str(),it->c_str());
        msg_sub_[*it] = nh.subscribe(temp_string.c_str(), 10, &AdroitInterface::msgCallback, this);
  
        // publishers
        temp_string="/sent_messages";
        ROS_INFO("\tpublisher %s in namespace %s",temp_string.c_str(),it->c_str());
        msg_pub_[*it] = nh.advertise<can_msgs::Frame>(temp_string.c_str(), 10);
      }else{
        // handle namespaced drives
        // subscribers
        std::string temp_string;
        temp_string="/";
        temp_string+=*it;
        temp_string+="/received_messages";
        ROS_INFO("\tsubscriber %s in namespace %s",temp_string.c_str(),it->c_str());
        msg_sub_[*it] = nh.subscribe(temp_string.c_str(), 10, &AdroitInterface::msgCallback, this);
  
        // publishers
        temp_string="/";
        temp_string+=*it;
        temp_string+="/sent_messages";
        ROS_INFO("\tpublisher %s in namespace %s",temp_string.c_str(),it->c_str());
        msg_pub_[*it] = nh.advertise<can_msgs::Frame>(temp_string.c_str(), 10);
      }
    }
  }
  
  // get private params
  // TODO is making another node handle here conflict with the nh that's passed into the init function?
  ros::NodeHandle private_nh("~");
  // check for parameters to enable additional HDT acutator interface topics and services
  if (ros::param::has("~hdt_telem_enable")){
    ros::param::get("~hdt_telem_enable",hdt_telem_enable_);
    ROS_INFO("AdroitInterface::init additional telemetry is %d",hdt_telem_enable_);
  }else{
    hdt_telem_enable_ = false;
  }
  
  /*
  if (ros::param::has("~hdt_service_enable")){
    ros::param::get("~hdt_service_enable",hdt_service_enable_);
    //ROS_INFO("AdroitInterface::init ROS services is %d",hdt_service_enable_);
  }else{
    hdt_service_enable_ = false;
  }
  */
  
  return true;
}

/*----------------------------------------------------------------------------
 * process msg
 *----------------------------------------------------------------------------*/
bool AdroitInterface::processMsg(const can_msgs::Frame *msg)
{
    // get message id and addr
    uint8_t id = (uint8_t)((msg->id >> 7) & 0x0F);
    uint8_t addr = (uint8_t)(msg->id & 0x7F);
    
    switch (id)
    {
      case HIGH_SPEED_TELEM:
      {
        // get values
	      int16_t pos = msg->data[0] | (msg->data[1] << 8);
	      int16_t vel = msg->data[2] | (msg->data[3] << 8);
	      int16_t eff = msg->data[4] | (msg->data[5] << 8);
	      int16_t current = msg->data[6] | (msg->data[7] << 8);
	      
	      // set values
	      pos_[addr] = pos/(double)INT16_MAX*POSITION_CONV;
	      vel_[addr] = vel/(double)INT16_MAX*VELOCITY_CONV;
	      eff_[addr] = eff/(double)INT16_MAX*EFFORT_CONV;
	      
	      //ROS_INFO("AdroitInterface:msgCallback drive %d, pos = %3.3f, vel  %3.3f, eff = %3.3f", addr, pos_[addr], vel_[addr], eff_[addr]);
	      
	      // update initial command
	      if (!addr_map_[addr])
	      {
	        //cmd_pos_[addr] = pos_[addr];
	        addr_map_[addr] = true;
	        //ROS_INFO("AdroitInterface:msgCallback received telemetry from drive %d", addr);
	      }
	      
	      if(hdt_telem_enable_){
	        //ROS_INFO("drive %d HIGH_SPEED_TELEM recieved",addr);
	        
	        // create topic name
	        char pub[50];
	        if(addr_to_name_map.find(addr) != addr_to_name_map.end()){
	          sprintf(pub,"%s/%s/hdt_hs_telem",robot_name_.c_str(),addr_to_name_map[addr].c_str());
	        }else{
	          // if the joint name isn't known, guess
	          //TODO check for name collisions
	          sprintf(pub,"%s/joint%d/hdt_hs_telem",robot_name_.c_str(),(int)addr);
	        }
	        
	        hdt_adroit_control::HDTHSTelem hdt_hs_telem_msg;
	        hdt_hs_telem_msg.header.stamp = ros::Time::now();
	        hdt_hs_telem_msg.position = pos/(double)INT16_MAX*POSITION_CONV;
	        hdt_hs_telem_msg.velocity = vel/(double)INT16_MAX*VELOCITY_CONV;
	        hdt_hs_telem_msg.effort = eff/(double)INT16_MAX*EFFORT_CONV;
	        hdt_hs_telem_msg.motor_current = current/(double)INT16_MAX*CURRENT_CONV;
	        
	        
          if(hdt_telem_pub.find(pub) == hdt_telem_pub.end()){
            // TODO is making another node handle here conflict with the nh that's passed into the init function?
            ros::NodeHandle nh;
            hdt_telem_pub[pub] = nh.advertise<hdt_adroit_control::HDTHSTelem>(pub, 10);
          }
          hdt_telem_pub[pub].publish(hdt_hs_telem_msg);
          
	      }
	      
	      return true;
	    }
	    case MEDIUM_SPEED_TELEM:
	    {
	      if(hdt_telem_enable_){
	        //ROS_INFO("drive %d MEDIUM_SPEED_TELEM recieved",addr);
	        
	        // create topic name
	        char pub[50];
	        if(addr_to_name_map.find(addr) != addr_to_name_map.end()){
	          sprintf(pub,"%s/%s/hdt_ms_telem",robot_name_.c_str(),addr_to_name_map[addr].c_str());
	        }else{
	          // if the joint name isn't known, guess
	          //TODO check for name collisions
	          sprintf(pub,"%s/joint%d/hdt_ms_telem",robot_name_.c_str(),(int)addr);
	        }
	        
	        // get values
	        int16_t var1 = msg->data[0] | (msg->data[1] << 8);
	        int16_t var2 = msg->data[2] | (msg->data[3] << 8);
	        int16_t var3 = msg->data[4] | (msg->data[5] << 8);
	        int16_t var4 = msg->data[6] | (msg->data[7] << 8);
	        
	        hdt_adroit_control::HDTMSTelem hdt_ms_telem_msg;
	        hdt_ms_telem_msg.header.stamp = ros::Time::now();
	        hdt_ms_telem_msg.var1 = var1;
	        hdt_ms_telem_msg.var2 = var2;
	        hdt_ms_telem_msg.var3 = var3;
	        hdt_ms_telem_msg.var4 = var4;
	        
	        
          if(hdt_telem_pub.find(pub) == hdt_telem_pub.end()){
            // TODO is making another node handle here conflict with the nh that's passed into the init function?
            ros::NodeHandle nh;
            hdt_telem_pub[pub] = nh.advertise<hdt_adroit_control::HDTMSTelem>(pub, 10);
          }
          hdt_telem_pub[pub].publish(hdt_ms_telem_msg);
          
	      }
	      return true;
	    }
	    case LOW_SPEED_TELEM:
	    {
	      if(hdt_telem_enable_){
	        //ROS_INFO("drive %d LOW_SPEED_TELEM recieved",addr);
	        
	        // create topic name
	        // TODO replace jointX assumption with names from hardware yaml file
	        char pub[50];
	        if(addr_to_name_map.find(addr) != addr_to_name_map.end()){
	          sprintf(pub,"%s/%s/hdt_ls_telem",robot_name_.c_str(),addr_to_name_map[addr].c_str());
	        }else{
	          // if the joint name isn't known, guess
	          //TODO check for name collisions
	          sprintf(pub,"%s/joint%d/hdt_ls_telem",robot_name_.c_str(),(int)addr);
	        }
	        
	        
	        // get values
	        int16_t bus_voltage = msg->data[0] | (msg->data[1] << 8);
	        int16_t bus_current = msg->data[2] | (msg->data[3] << 8);
	        int16_t temperature = msg->data[4] | (msg->data[5] << 8);
	      
	        hdt_adroit_control::HDTLSTelem hdt_ls_telem_msg;
	        
	        hdt_ls_telem_msg.header.stamp = ros::Time::now();
					hdt_ls_telem_msg.bus_voltage = bus_voltage/(double)INT16_MAX*VOLTAGE_CONV;
					hdt_ls_telem_msg.bus_current = bus_current/(double)INT16_MAX*CURRENT_CONV;
					hdt_ls_telem_msg.temperature = temperature/(double)INT16_MAX*TEMPERATURE_CONV;
	        
	        
          if(hdt_telem_pub.find(pub) == hdt_telem_pub.end()){
            // TODO is making another node handle here conflict with the nh that's passed into the init function?
            ros::NodeHandle nh;
            hdt_telem_pub[pub] = nh.advertise<hdt_adroit_control::HDTLSTelem>(pub, 10);
          }
          hdt_telem_pub[pub].publish(hdt_ls_telem_msg);
	      }
	      return true;
	    }
	    case DEBUG_TELEM:
	    {
	      if(hdt_telem_enable_){
	        
	        uint8_t header = msg->data[0];
	        //ROS_INFO("drive %d DEBUG_TELEM recieved, index %d",addr,index);
	        
	        uint8_t telem_size = 7 - (header & 0x0E);
	        memcpy(&debug_telem_buffer[debug_buffer_bytes], &msg->data[1], telem_size);
          debug_buffer_bytes += telem_size;
          // check for bit flag in last message
          if (header & 0x01){
            // create topic name
            // TODO replace jointX assumption with names from hardware yaml file
            char pub_ls[50];
	          sprintf(pub_ls,"%s/joint%d/hdt_debug_ls_telem",robot_name_.c_str(),(int)addr);
	          if(hdt_telem_pub.find(pub_ls) == hdt_telem_pub.end()){
              // TODO is making another node handle here conflict with the nh that's passed into the init function?
              ros::NodeHandle nh;
              hdt_telem_pub[pub_ls] = nh.advertise<hdt_adroit_control::HDTDebugLSTelem>(pub_ls, 1000);
            }
            char pub_hs[50];
	          sprintf(pub_hs,"%s/joint%d/hdt_debug_hs_telem",robot_name_.c_str(),(int)addr);
	          if(hdt_telem_pub.find(pub_hs) == hdt_telem_pub.end()){
              // TODO is making another node handle here conflict with the nh that's passed into the init function?
              ros::NodeHandle nh;
              hdt_telem_pub[pub_hs] = nh.advertise<hdt_adroit_control::HDTDebugHSTelem>(pub_hs, 1000);
            }
            // copy header, containing time stamp
            // TODO how does this work out with sequence numbers?
            hdt_adroit_control::HDTDebugLSTelem hdt_debug_ls_telem_msg;
            hdt_debug_ls_telem_msg.header = msg->header;
          
            uint8_t index = debug_telem_buffer[0];
            uint8_t hs_samples = debug_telem_buffer[1];
            hdt_debug_ls_telem_msg.ls_var1 = debug_telem_buffer[2] | (debug_telem_buffer[3] << 8);
            hdt_debug_ls_telem_msg.ls_var2 = debug_telem_buffer[4] | (debug_telem_buffer[5] << 8);
            hdt_debug_ls_telem_msg.ls_var3 = debug_telem_buffer[6] | (debug_telem_buffer[7] << 8);
            
            hdt_telem_pub[pub_ls].publish(hdt_debug_ls_telem_msg);
              
            hdt_adroit_control::HDTDebugHSTelem hdt_debug_hs_telem_msg;
            hdt_debug_hs_telem_msg.header = msg->header;
            int16_t hs_var1[hs_samples];
            int16_t hs_var2[hs_samples];
            // copy 16 values (32 bytes) of data becuase that's how big the data object in drive code
            // use hs_samples to know how many of those values are real/populated
            memcpy(&hs_var1[0], &debug_telem_buffer[8], 32);
            memcpy(&hs_var2[0], &debug_telem_buffer[40], 32);
            // copy CRC check from AdroitComs.cpp
            //memcpy(&telem->crc, &debug_telem_buffer[72], sizeof(uint32_t));
            for(uint8_t i=0; i<hs_samples; i++){
              hdt_debug_hs_telem_msg.hs_var1 = hs_var1[i];
              hdt_debug_hs_telem_msg.hs_var2 = hs_var2[i];
              hdt_telem_pub[pub_hs].publish(hdt_debug_hs_telem_msg);
            
            }
            
          
            // clear out buffer data
            memset(&debug_telem_buffer, 0, sizeof(debug_telem_buffer));
            debug_buffer_bytes=0;
          }
          
	      }
	      return true;
	    }
	    case ERROR_TELEM:
	    {
	      uint8_t code = msg->data[0];
		    uint8_t severity = msg->data[1] & 0x0F;
		    uint8_t type = (msg->data[1] & 0xF0) >> 4;
		    uint32_t value = msg->data[2] + (msg->data[3] << 8) + (msg->data[4] << 16) + (msg->data[5] << 24);
		    
	      std::stringstream error_msg;
	      int addr_int = addr;
	      int code_int = code;
	      error_msg << "AdroitInterface:msgCallback drive " << addr_int << " reported error code " << code_int;
	      
	      // get type dependent value
	      switch (type)
	      {
	        case 0:
	          error_msg << " with value " << value;
	          break;
	        case 1:
	        {
	          float value_float;
	          memcpy(&value_float, &value, sizeof(float));
	          error_msg << " with value " << value_float;
	          break;
	        }
	        default:
	          break;
	      }
	      
        // report error
        switch (severity)
        {
          case 0:
            ROS_INFO_STREAM(error_msg.str());
            break;
          case 1:
            ROS_WARN_STREAM(error_msg.str());
            break;
          default:
            ROS_ERROR_STREAM(error_msg.str());
            break;
        }
        return true;
	    }
	    case PARAMETER_TELEM:
	    {
	      if(msg->data[0] == 0x43 and msg->data[1] == 0x04 and msg->data[2] == 0x20){
	        drive_type_[addr]= msg->data[4];
          ROS_INFO("AdroitInterface:processMsg drive %d reports drive type %d",addr,drive_type_[addr]);
	        switch(msg->data[5]){
	          case 0x00:
	            joint_mode_current[addr]=JOINT_POSITION_MODE;
              ROS_INFO("AdroitInterface:processMsg drive %d reports position mode",addr);
	            break;
	          case 0x02:
	            joint_mode_current[addr]=JOINT_VELOCITY_MODE;
              ROS_INFO("AdroitInterface:processMsg drive %d reports velocity mode",addr);
	            break;
	          default:
	            ROS_ERROR("AdroitInterface:processMsg drive %d reports unknown mode %d", addr,msg->data[5]);
	        }
	        
	      }
	      break;
	    }
	    default:
	      return false;
	  }
}

/*----------------------------------------------------------------------------
 * read
 *----------------------------------------------------------------------------*/
bool AdroitInterface::read()
{
  // clear flags in drive address list
  // use to check if we have active data from the drives this control cycle
  if (!fake_execution_)
  {
  
    //ROS_INFO("AdroitInterface:read resetting flags for drive telemetry");  
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      addr_map_[addr] = false;
      //ROS_INFO("AdroitInterface::read drive %d cmd %f telem %f", addr,cmd_pos_[addr],pos_[addr]);
    }
    
    control_enabled_ =false;
  }
  
  // lock mutex
  std::lock_guard<std::mutex> lock(mutex_);
  
  // process all messages in queue
  while (!msg_queue_.empty())
  {
    // get message at top of queue
    can_msgs::Frame msg = msg_queue_.front();
    
    // process message
    processMsg(&msg);

    // pop msg
    msg_queue_.pop();
  }
  
  // update "telem" for fake execution
  if (fake_execution_)
  {
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      pos_[addr] = cmd_pos_[addr];
      vel_[addr] = 0.0;
      eff_[addr] = 0.0;
    }
    
    return true;
  }
  // otherwise check if we can enable control
  else if (!control_enabled_)
  {
    bool temp = true;
    
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      bool enabled = it->second;
      if (!enabled)
        temp = false;
    }
    
    // update control enabled
    control_enabled_ = temp;
    //if (control_enabled_)
      //ROS_INFO("AdroitInterface:read control enabled");  
  }
  
  
  // return control enabled
  return control_enabled_;
}
 
 /*----------------------------------------------------------------------------
 * write
 *----------------------------------------------------------------------------*/
bool AdroitInterface::write()
{ 
  if ((!fake_execution_) && (control_enabled_))
  {
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      can_msgs::Frame msg;
      
      // build msg
      msg.header.stamp = ros::Time::now();
      msg.id = (CONTROL_CMD << 7) | (addr & 0x7F);
      msg.dlc = 8;
      
      //ROS_INFO("drive %d cmd_pos_ [%f] cmd_vel_ [%f]",addr,cmd_pos_[addr],cmd_vel_[addr]);
      //ROS_INFO("AdroitInterface:write drive %d getModeName %s",addr,mode_handle_[addr].getModeName(mode_handle_[addr].getMode()).c_str());
      
      if(joint_mode_current[addr] == NONE_MODE){
        // try to read drive mode
        read_drive_mode(addr);
      }else{
        // use NaN as flag to tell when position or velocity command has been updated
        // work around due to lack for information coming from ROS controller interface
        if(!isnan(cmd_pos_[addr])){
          // should be in position mode
          if(joint_mode_current[addr] != JOINT_POSITION_MODE){
            ROS_INFO("AdroitInterface:write drive %d switch to position mode",addr);
            //joint_mode_current[addr] = JOINT_POSITION_MODE;
            joint_mode_current[addr] = NONE_MODE;
            cmd_vel_[addr] = std::nanf("");
            // read and set joint control mode parameter
            set_drive_mode(addr, JOINT_POSITION_MODE);
          }
        }
        if(!isnan(cmd_vel_[addr])){
          // should be in velocity mode
          if(joint_mode_current[addr] != JOINT_VELOCITY_MODE){
            ROS_INFO("AdroitInterface:write drive %d switch to velocity mode",addr);
            //joint_mode_current[addr] = JOINT_VELOCITY_MODE;
            joint_mode_current[addr] = NONE_MODE;
            cmd_pos_[addr] = std::nanf("");
            set_drive_mode(addr, JOINT_VELOCITY_MODE);
            /*
            // check for continuous rotation drive type
            // TODO updated drive code to handle non-continuous rotation velocity mode
            if(drive_type_[addr] & 0x04){
              // read and set joint control mode parameter
              ROS_INFO("\t yup continuous rotation drive");
              set_drive_mode(addr, JOINT_VELOCITY_MODE);
            }else{
              ROS_INFO("\t nope-a-nope");
            }
            */
          }
        }
      }
      
      int16_t pos = 0;
      int16_t vel = 0;
      switch(joint_mode_current[addr]){
        case JOINT_POSITION_MODE:
          pos = (int16_t)(cmd_pos_[addr]*(double)INT16_MAX/POSITION_CONV);
          vel = (int16_t)(VELOCITY_CONV*(double)INT16_MAX/VELOCITY_CONV);
          break;
        case JOINT_VELOCITY_MODE:
          /*
          // work around needed if doing velocity control inside of position mode
          if(cmd_vel_[addr]>0){
            pos = (int16_t)(POSITION_CONV*(double)INT16_MAX/POSITION_CONV);
          }else{
            pos = (int16_t)(-POSITION_CONV*(double)INT16_MAX/POSITION_CONV);
          }
          */
          vel = (int16_t)(cmd_vel_[addr]*(double)INT16_MAX/VELOCITY_CONV);
          break;
      }
      // set data
      //int16_t pos = (int16_t)(cmd_pos_[addr]*(double)INT16_MAX/POSITION_CONV);
      //int16_t vel = (int16_t)(VELOCITY_CONV*(double)INT16_MAX/VELOCITY_CONV);
      int16_t eff = (int16_t)(0.0*(double)INT16_MAX/EFFORT_CONV);
      int16_t cur = (int16_t)(CURRENT_CONV*(double)INT16_MAX/CURRENT_CONV);

      msg.data[0] = pos & 0xFF;
      msg.data[1] = (pos >> 8) & 0xFF;
      msg.data[2] = vel & 0xFF;
      msg.data[3] = (vel >> 8) & 0xFF;
      msg.data[4] = eff & 0xFF;
      msg.data[5] = (eff >> 8) & 0xFF;
      msg.data[6] = cur & 0xFF;
      msg.data[7] = (cur >> 8) & 0xFF;

      // check if this drive ID has a known namespace
      if(namspace_map_.find(addr) != namspace_map_.end()){
        //ROS_INFO("AdroitInterface:write drive %d is in namespace [%s]",addr,namspace_map_[addr].c_str());  
        // send message
        msg_pub_[namspace_map_[addr]].publish(msg);
      }
    }
    
    return true;
  }
  
  return false;
}
 /*----------------------------------------------------------------------------
 * reset - set commanded position back to telem position
 *----------------------------------------------------------------------------*/
bool AdroitInterface::reset()
{ 
  if ((!fake_execution_) && (control_enabled_))
  {
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      
      //ROS_INFO("AdroitInterface::reset drive %d cmd %f telem %f", addr,cmd_pos_[addr],pos_[addr]);
      cmd_pos_[addr] = pos_[addr];
      cmd_vel_[addr] = std::nanf("");
      // assume joint starts in position mode
      joint_mode_current[addr] = NONE_MODE;
    }
    
    return true;
  }
  
  return false;
}
 /*----------------------------------------------------------------------------
 * read_drive_mode - read drive mode parameter 4
 *----------------------------------------------------------------------------*/
bool AdroitInterface::read_drive_mode(int addr){

  can_msgs::Frame msg;
  
  // build msg
  msg.header.stamp = ros::Time::now();
  msg.id = (PARAMETER_CMD << 7) | (addr & 0x7F);
  msg.dlc = 8;
      
  
  msg.data[0] = 0x40; // parameter read command
  msg.data[1] = 0x04; // parameter number
  msg.data[2] = 0x20; // ?
  msg.data[3] = 0x00; // ?
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;
      

      
  // check if this drive ID has a known namespace
  if(namspace_map_.find(addr) != namspace_map_.end()){
    //ROS_INFO("AdroitInterface:write drive %d is in namespace [%s]",addr,namspace_map_[addr].c_str());  
    // send message
    msg_pub_[namspace_map_[addr]].publish(msg);
  }
}

 /*----------------------------------------------------------------------------
 * set_drive_mode - set drive mode parameter 4
 *----------------------------------------------------------------------------*/
bool AdroitInterface::set_drive_mode(int addr, joint_modes new_mode)
{

  can_msgs::Frame msg;
  
  // build msg
  msg.header.stamp = ros::Time::now();
  msg.id = (PARAMETER_CMD << 7) | (addr & 0x7F);
  msg.dlc = 8;

  switch(new_mode){
    case JOINT_POSITION_MODE:
      msg.data[5] = 0x00; // control mode
      break;
    case JOINT_VELOCITY_MODE:
      msg.data[5] = 0x02; // control mode
  
  }
  msg.data[0] = 0x23; // parameter write command
  msg.data[1] = 0x04; // parameter number LSB
  msg.data[2] = 0x20; // parameter type and parameter number MSB
  msg.data[3] = 0x00; // ?
  msg.data[4] = drive_type_[addr]; // device type (should copy existing value)
  //msg.data[5] = 0x02; // control mode
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;
      
  // check if this drive ID has a known namespace
  if(namspace_map_.find(addr) != namspace_map_.end()){
    //ROS_INFO("AdroitInterface:write drive %d is in namespace [%s]",addr,namspace_map_[addr].c_str());  
    // send message
    msg_pub_[namspace_map_[addr]].publish(msg);
  }

}


