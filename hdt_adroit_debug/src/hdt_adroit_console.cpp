

#include <hdt_adroit_debug/hdt_adroit_console.h>


/*----------------------------------------------------------------------------
 * init
 *----------------------------------------------------------------------------*/
bool AdroitConsole::init()
{
  // get private params
  ros::NodeHandle nh;
  

  ros::Subscriber CAN_sub = nh.subscribe("received_messages", 10, &AdroitConsole::CANmessageCallback, this);
  ROS_INFO("AdroitConsole::init CAN subscriber created");
        
  // publishersReadParamServiceCallback
  CAN_pub = nh.advertise<can_msgs::Frame>("sent_messages", 10);
  ROS_INFO("AdroitConsole::init CAN publisher created");
        
  // read parameter services
  ros::ServiceServer read_param_srv = nh.advertiseService("hdt_adroit_debug/read_drive_param", &AdroitConsole::ReadParamServiceCallback, this);
  ros::ServiceServer readall_param_srv = nh.advertiseService("hdt_adroit_debug/readall_drive_param", &AdroitConsole::ReadAllParamServiceCallback, this);
  // write parameter services
  ros::ServiceServer write_param_srv = nh.advertiseService("hdt_adroit_debug/write_drive_param", &AdroitConsole::WriteParamServiceCallback, this);
  ros::ServiceServer writeall_param_srv = nh.advertiseService("hdt_adroit_debug/writeall_drive_param", &AdroitConsole::WriteAllParamServiceCallback, this);
  // commit parameters to flash memory service
  ros::ServiceServer commit_srv = nh.advertiseService("hdt_adroit_debug/commit", &AdroitConsole::CommitServiceCallback, this);
  // state, status, appload
  //state select service
  ros::ServiceServer state_select_srv=nh.advertiseService("hdt_adroit_debug/state_select", &AdroitConsole::StateSelectServiceCallback, this);
  // get status service
  ros::ServiceServer get_status_srv=nh.advertiseService("hdt_adroit_debug/get_status", &AdroitConsole::GetStatusServiceCallback, this);
  
  ROS_INFO("AdroitConsole::init services created");
  
  
  std::string robot_name;
  if (!nh.getParam("robot_name", robot_name))
  {
    ROS_WARN("Robot name not provided");
  }
  
  ros::NodeHandle sub_nh(robot_name.c_str());
  // get joints for given robot
  std::vector<std::string> joints;
  if (!sub_nh.getParam("hardware_interface/joints", joints))
  {
    ROS_WARN("AdroitConsole:init no joint parameters specified");
    //return false;
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
    if (sub_nh.getParam(addr_param.str(), addr))
    {
      if ((addr > 0) && (addr < MAX_DRIVES))
      {
        if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole:init registering %s with id %d", joint->c_str(), addr);
        drive_names[*joint] = addr;
      }
      // report invalid id
      else
      {
        ROS_ERROR("AdroitInterface:init invalid addr = %d", addr);
        return false;
      }
    }
  }
  
  // copy parameter definitions from #defines in adroit_params.h into local arrays
  // TODO there's probably more efficent way to copy the data from a #define
  uint16_t temp_array[ParameterListLength] = ParameterIndexList;
  memcpy(ParameterIndexListLocal,temp_array,ParameterListLength*sizeof(uint16_t));
  std::string temp_array2[ParameterListLength] = ParameterNameList;
  for(int i=0; i<ParameterListLength; i++){
    ParameterNameListLocal.push_back(temp_array2[i]);
  }
  uint8_t temp_array3[ParameterListLength] = ParameterDataTypeList;
  memcpy(ParameterDataTypeListLocal,temp_array3,ParameterListLength*sizeof(uint8_t));
  
  
  
  //ros::waitForShutdown();
  //ros::spin();
  // start spinner

  // start multiple threads to handle ROS spin activities
  // useful for handling ROS side services and CAN interface communications at the same time
  ros::AsyncSpinner spinner(3);
  spinner.start();
  // asyncspinners don't block so add a waitforshutdown here to keep the node open
  ros::waitForShutdown();
  return true;
}

/*--------------------------------------------------------------------------
  CAN message subscriber callback
* -----------------------------------------------------------------------------*/
void AdroitConsole::CANmessageCallback(const can_msgs::Frame::ConstPtr& msg){
  // get message id and addr
  uint8_t id = (uint8_t)((msg->id >> 7) & 0x0F);
  uint8_t addr = (uint8_t)(msg->id & 0x7F);
  //if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::CANmessageCallback message recieved addr %d id %d",addr,id);
    
  // copy CAN message into adroit message format
  // use a normal variable not a pointer, copying the ConstPtr into another pointer crashes the program (I'm not sure why)
  AdroitComs::AdroitMsg message;
  message.id = msg->id;
  message.dlc = msg->dlc;
  memcpy(&message.data, &msg->data,8);
  switch (id)
  {
    case AdroitComs::STATE_CHANGE_CMD:
      break;
    case AdroitComs::ERROR_TELEM:
      break;
    case AdroitComs::HIGH_SPEED_TELEM:
      break;
    case AdroitComs::CONTROL_CMD:
      break;
    case AdroitComs::MEDIUM_SPEED_TELEM:
      break;
    case AdroitComs::IMPEDANCE_CMD:
      break;
    case AdroitComs::LOW_SPEED_TELEM:
      break;
    case AdroitComs::DEBUG_TELEM:
      break;
    case AdroitComs::CURRENT_CMD:
      break;
    case AdroitComs::PARAMETER_TELEM:
      // process parameter telemetry and reset conditional variable used to wait for parameter response
      // this is used to check if the read was successful
      if(AdroitComs::process_parameter_telem(&drive_data[addr].parameter_telem, &message)) {
        drive_data[addr].msg_update[id] = true;
        pthread_mutex_lock(&drive_data[addr].mutex);
        pthread_cond_signal(&drive_data[addr].cond_var);
        pthread_mutex_unlock(&drive_data[addr].mutex);
      }
      //ROS_INFO("param telem %d",drive_data[addr].parameter_telem.data);
      break;
    case AdroitComs::PARAMETER_CMD:
      break;
    case AdroitComs::STATUS_CMD:
      break;
    case AdroitComs::STATUS_TELEM:
        if(AdroitComs::process_status_telem(&drive_data[addr].status_telem, &message)) {

        drive_data[addr].msg_update[id] = true;
        /*
        hdt_status_telem_msg[addr].state=drive_data[addr]status_telem.state;
        //ROS_INFO("drive %d is in state %d",addr,adroit_drives[addr]->status_telem.state);
        
        if(hdt_status_telem_pub[addr])
          hdt_status_telem_pub[addr].publish(hdt_status_telem_msg[addr]);
          //else
          //  std::cout<<"empty publisher"<<std::endl;

        }
        */
        }else{
          ROS_INFO("received an invalid status telem message from drive");
        }
      break;
  }
}

/*--------------------------------------------------------------------------
  read parameter service callback
* -----------------------------------------------------------------------------*/
int AdroitConsole::SendMsg(AdroitMsg *adroit_msg) {

  //if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::SendMsg");
  can_msgs::Frame message;
  message.id = adroit_msg->id;
  message.dlc= adroit_msg->dlc;
  memcpy(&message.data, &adroit_msg->data,8);
  CAN_pub.publish(message);
  return 1;
}
/*--------------------------------------------------------------------------
  read parameter service callback
* -----------------------------------------------------------------------------*/
int AdroitConsole::Write(void) {
  //if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::Write");
  return 1;
}

/*--------------------------------------------------------------------------
  read parameter service callback
* -----------------------------------------------------------------------------*/
bool AdroitConsole::ReadParamServiceCallback(hdt_adroit_debug::ReadDriveParam::Request& req, hdt_adroit_debug::ReadDriveParam::Response& res) {
  
  if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::ReadParamServiceCallback - drive %s or CAN_ID %d parameter %d",req.name.c_str(),req.can_id,req.param_number);


  // check for valid CAN ID or known joint name
  std::string return_drive_name;
  uint8_t return_CAN_ID;
  if(ResolveCANid(req.name, req.can_id, &return_drive_name, &return_CAN_ID)){
    //ROS_INFO("input %s %d but using %s %d",req.name.c_str(),req.can_id,return_drive_name.c_str(),return_CAN_ID);
  }else{
    ROS_ERROR("AdroitConsole::ReadParamServiceCallback - invalid input, joint");
    //res.ret=1;
    return false;
  }
  
  int addr = return_CAN_ID;
  
  std::string return_param_name;
  uint16_t return_param_num;
  uint8_t return_param_type;

  if(AdroitConsole::ResolveParameter(req.param_name, req.param_number, &return_param_name, &return_param_num, &return_param_type)){
    //ROS_INFO("input %s %d but using %s %d",req.param_name.c_str(),req.param_number,return_param_name.c_str(),return_param_num);
  }else{
    ROS_ERROR("AdroitConsole::ReadParamServiceCallback - invalid input, parameter");
    //res.ret=1;
    return false;
  }


  //int param_number=req.param_number;
  if(AdroitComs::read_parameter(addr, return_param_num,&drive_data[addr])) {
    
    res.param_name = return_param_name;
    
    switch(return_param_type){
      case ARRAY_PARAMETER:
        res.param_type = hdt_adroit_debug::ReadDriveParam::Response::ARRAY_PARAMETER;
        res.value_uint32 = drive_data[addr].parameter_telem.data;
        break;
      case UINT32_PARAMETER:
        res.param_type = hdt_adroit_debug::ReadDriveParam::Response::UINT32_PARAMETER;
        res.value_uint32 = drive_data[addr].parameter_telem.data;
        break;
      case INT32_PARAMETER:
        res.param_type = hdt_adroit_debug::ReadDriveParam::Response::INT32_PARAMETER;
        memcpy(&res.value_int32,&drive_data[addr].parameter_telem.data,sizeof(uint32_t));
        break;
      case FLOAT32_PARAMETER:
        res.param_type = hdt_adroit_debug::ReadDriveParam::Response::FLOAT32_PARAMETER;
        memcpy(&res.value_float32,&drive_data[addr].parameter_telem.data,sizeof(uint32_t));
        break;
      case UINT16_PARAMETER:
        res.param_type = hdt_adroit_debug::ReadDriveParam::Response::UINT16_PARAMETER;
        memcpy(&res.value_uint16,&drive_data[addr].parameter_telem.data,sizeof(uint32_t));
        break;
      case UINT8_PARAMETER:
        res.param_type = hdt_adroit_debug::ReadDriveParam::Response::UINT8_PARAMETER;
        memcpy(&res.value_uint8,&drive_data[addr].parameter_telem.data,sizeof(uint32_t));
        break;
    }
    
    return true;
  }else{
    ROS_ERROR("parameter read unsuccessful. Drive: %d. Index: %d.",addr, req.param_number);
    res.value_uint32 = 0;
    return false;
  }

}

/*--------------------------------------------------------------------------
  write parameter service callback
* -----------------------------------------------------------------------------*/
bool AdroitConsole::WriteParamServiceCallback(hdt_adroit_debug::WriteDriveParam::Request& req, hdt_adroit_debug::WriteDriveParam::Response& res) {

  if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::WriteParamServiceCallback - drive %s or CAN_ID %d parameter %s or %d",req.name.c_str(),req.can_id,req.param_name.c_str(),req.param_number);
  
  
  // check for valid CAN ID or known joint name
  std::string return_drive_name;
  uint8_t return_CAN_ID;
  if(ResolveCANid(req.name, req.can_id, &return_drive_name, &return_CAN_ID)){
    //ROS_INFO("input %s %d but using %s %d",req.name.c_str(),req.can_id,return_drive_name.c_str(),return_CAN_ID);
  }else{
    ROS_ERROR("AdroitConsole::WriteParamServiceCallback - invalid input");
    res.ret=1;
    return false;
  }
  
  int addr = return_CAN_ID;
  
  std::string return_param_name;
  uint16_t return_param_num;
  uint8_t return_param_type;

  if(AdroitConsole::ResolveParameter(req.param_name, req.param_number, &return_param_name, &return_param_num, &return_param_type)){
    //ROS_INFO("input %s %d but using %s %d",req.param_name.c_str(),req.param_number,return_param_name.c_str(),return_param_num);
  }else{
    ROS_ERROR("AdroitConsole::WriteParamServiceCallback - invalid input, parameter");
    //res.ret=1;
    return false;
  }
  

  // check if name and type match
  if(req.param_type!=return_param_type){
    ROS_WARN("attempt to write drive %s or CAN_ID %d parameter %s or %d as incorrect type %d!=%d",req.name.c_str(),req.can_id,req.param_name.c_str(),req.param_number,req.param_type,return_param_type);
    return false;
  }
  
  // check if name and type match
  if(req.param_name!=return_param_name and req.param_name.length()>0){
    ROS_WARN("attempt to write drive %s or CAN_ID %d parameter %d as with incorrect name %s!=%s",req.name.c_str(),req.can_id,req.param_number,req.param_name.c_str(),return_param_name.c_str());
    return false;
  }
  
  
  uint32_t data;
  // case statements defined by enums in the WriteDriveParam.srv because that is the reference for write requests
  switch(return_param_type){
    case hdt_adroit_debug::ReadDriveParam::Response::ARRAY_PARAMETER:
      memcpy(&data,&req.value_uint32,sizeof(uint32_t));
      break;
    case hdt_adroit_debug::ReadDriveParam::Response::UINT32_PARAMETER:
      memcpy(&data,&req.value_uint32,sizeof(uint32_t));
      break;
    case hdt_adroit_debug::ReadDriveParam::Response::INT32_PARAMETER:
      memcpy(&data,&req.value_int32,sizeof(uint32_t));
      break;
    case hdt_adroit_debug::ReadDriveParam::Response::FLOAT32_PARAMETER:
      memcpy(&data,&req.value_float32,sizeof(uint32_t));
      break;
    case hdt_adroit_debug::ReadDriveParam::Response::UINT16_PARAMETER:
      memcpy(&data,&req.value_uint16,sizeof(uint32_t));
      break;
    case hdt_adroit_debug::ReadDriveParam::Response::UINT8_PARAMETER:
      memcpy(&data,&req.value_uint8,sizeof(uint32_t));
      break;
  }
  if(AdroitComs::write_parameter(addr, return_param_num, &drive_data[addr], &data)) {
    if(AdroitComs::read_parameter(addr, return_param_num, &drive_data[addr])) {
      // TODO should there be a check here that the read value matches the write value?
      res.ret=0;
    }
  }
  else {
    res.ret=1;
    ROS_ERROR("parameter write failed");
  }

  return true;
  
}

/*--------------------------------------------------------------------------
  read all parameter service callback
* -----------------------------------------------------------------------------*/
bool AdroitConsole::ReadAllParamServiceCallback(hdt_adroit_debug::ReadAllDriveParam::Request& req, hdt_adroit_debug::ReadAllDriveParam::Response& res) {
  
  if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::ReadAllParamServiceCallback - drive %s or CAN_ID %d parameter file %s",req.name.c_str(),req.can_id,req.param_file.c_str());


  // check for valid CAN ID or known joint name
  std::string return_drive_name;
  uint8_t return_CAN_ID;
  if(ResolveCANid(req.name, req.can_id, &return_drive_name, &return_CAN_ID)){
    //ROS_INFO("input %s %d but using %s %d",req.name.c_str(),req.can_id,return_drive_name.c_str(),return_CAN_ID);
  }else{
    ROS_ERROR("AdroitConsole::ReadAllParamServiceCallback - invalid input, joint");
    //res.ret=1;
    return false;
  }
  
  char temp_path[150];
  // check if parameter file path is valid
  if(req.param_file[0]=='/'){
    sprintf(temp_path,"%s",req.param_file.c_str());
    //ROS_INFO("using absolute path %s",temp_path);;
  }else if(req.param_file[0]=='~'){
    req.param_file.erase(0,1);
    sprintf(temp_path,"%s%s",std::getenv("HOME"),req.param_file.c_str());
    //ROS_INFO("using home path %s",temp_path);
  }else{
    sprintf(temp_path,"%s/%s",std::getenv("HOME"),req.param_file.c_str());
    //ROS_INFO("not specified, assuming home path %s",temp_path);
  }
  
  hdt_adroit_debug::ReadDriveParam::Request internal_req;
  hdt_adroit_debug::ReadDriveParam::Response internal_res;
  internal_req.can_id = return_CAN_ID;
  
  std::ofstream output_file;
  output_file.open(temp_path , std::ofstream::out);
  if(output_file.is_open()){
    char temp_param[150];
    uint8_t temp_size;
    output_file << "parameter number, parameter name, parameter type, parameter\n";
    for(uint16_t i=0; i<ParameterCount; i++){
      //if(AdroitComs::read_parameter(return_CAN_ID, i,&drive_data[return_CAN_ID])) {
      internal_req.param_number = i;
      if(ReadParamServiceCallback(internal_req,internal_res)){
        // case statements defined by parameter types in adroit_params.h because that is the reference for reading from drives
        switch(internal_res.param_type){
          case ARRAY_PARAMETER:
            temp_size = sprintf(temp_param,"%d,%s,%s,%u\n",i,internal_res.param_name.c_str(),"ARRAY_PARAMETER",internal_res.value_uint32);
            break;
          case UINT32_PARAMETER:
            temp_size = sprintf(temp_param,"%d,%s,%s,%u\n",i,internal_res.param_name.c_str(),"UINT32_PARAMETER",internal_res.value_uint32);
            break;
          case INT32_PARAMETER:
            temp_size = sprintf(temp_param,"%d,%s,%s,%d\n",i,internal_res.param_name.c_str(),"INT32_PARAMETER",internal_res.value_int32);
            break;
          case FLOAT32_PARAMETER:
            temp_size = sprintf(temp_param,"%d,%s,%s,%f\n",i,internal_res.param_name.c_str(),"FLOAT32_PARAMETER",internal_res.value_float32);
            break;
          case UINT16_PARAMETER:
            temp_size = sprintf(temp_param,"%d,%s,%s,%u,%u\n",i,internal_res.param_name.c_str(),"UINT16_PARAMETER",internal_res.value_uint16[0],internal_res.value_uint16[1]);
            break;
          case UINT8_PARAMETER:
            temp_size = sprintf(temp_param,"%d,%s,%s,%u,%u,%u,%u\n",i,internal_res.param_name.c_str(),"UINT8_PARAMETER",internal_res.value_uint8[0],internal_res.value_uint8[1],internal_res.value_uint8[2],internal_res.value_uint8[3]);
            break;
          default:
            ROS_WARN("AdroitConsole::ReadAllParamServiceCallback - unrecognized parameter type");
        }
        output_file.write(temp_param,temp_size);
      }else{
        ROS_ERROR("AdroitConsole::ReadAllParamServiceCallback - read failure");
        output_file.close();
        //res.ret=1;
        return false;
      }
    }
    output_file.close();
  }else{
    ROS_ERROR("AdroitConsole::ReadAllParamServiceCallback - invalid input, param file");
    //res.ret=1;
    return false;
  }
  
}
/*--------------------------------------------------------------------------
  write all parameter service callback
* -----------------------------------------------------------------------------*/
bool AdroitConsole::WriteAllParamServiceCallback(hdt_adroit_debug::ReadAllDriveParam::Request& req, hdt_adroit_debug::ReadAllDriveParam::Response& res) {
  
  if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::WriteAllParamServiceCallback - drive %s or CAN_ID %d parameter file %s",req.name.c_str(),req.can_id,req.param_file.c_str());


  // check for valid CAN ID or known joint name
  std::string return_drive_name;
  uint8_t return_CAN_ID;
  if(ResolveCANid(req.name, req.can_id, &return_drive_name, &return_CAN_ID)){
    //ROS_INFO("input %s %d but using %s %d",req.name.c_str(),req.can_id,return_drive_name.c_str(),return_CAN_ID);
  }else{
    ROS_ERROR("AdroitConsole::WriteAllParamServiceCallback - invalid input, joint");
    //res.ret=1;
    return false;
  }
  
  char temp_path[150];
  // check if parameter file path is valid
  if(req.param_file[0]=='/'){
    sprintf(temp_path,"%s",req.param_file.c_str());
    //ROS_INFO("using absolute path %s",temp_path);;
  }else if(req.param_file[0]=='~'){
    req.param_file.erase(0,1);
    sprintf(temp_path,"%s%s",std::getenv("HOME"),req.param_file.c_str());
    //ROS_INFO("using home path %s",temp_path);
  }else{
    sprintf(temp_path,"%s/%s",std::getenv("HOME"),req.param_file.c_str());
    //ROS_INFO("not specified, assuming home path %s",temp_path);
  }
  
  
  hdt_adroit_debug::WriteDriveParam::Request internal_req;
  hdt_adroit_debug::WriteDriveParam::Response internal_res;
  internal_req.can_id = return_CAN_ID;
  uint16_t line_counter =0;
  std::ifstream input_file;
  input_file.open(temp_path , std::ifstream::in);
  if(input_file.is_open()){
    char temp_param[256];
    //uint8_t temp_size;
    
    std::string temp_field;
    // read out the header line
    input_file.getline(temp_param,256);
    ROS_INFO("header line : %s",temp_param);
    while(input_file.getline(temp_param,256)){
      line_counter++;
      ROS_INFO("reading line %d",line_counter);
      std::stringstream temp_stream(temp_param);
      // get parameter number
      std::getline(temp_stream,temp_field,',');
      internal_req.param_number = std::stoul(temp_field,nullptr,0);
      // get parameter name
      std::getline(temp_stream,temp_field,',');
      internal_req.param_name = temp_field;
      // get parameter type
      std::getline(temp_stream,temp_field,',');
      if(temp_field.compare("ARRAY_PARAMETER")==0){
        internal_req.param_type = hdt_adroit_debug::ReadDriveParam::Response::ARRAY_PARAMETER;
        std::getline(temp_stream,temp_field,',');
        internal_req.value_uint32 = std::stoul(temp_field,nullptr,0);
      }
      if(temp_field.compare("UINT32_PARAMETER")==0){
        internal_req.param_type = hdt_adroit_debug::ReadDriveParam::Response::UINT32_PARAMETER;
        std::getline(temp_stream,temp_field,',');
        internal_req.value_uint32 = std::stoul(temp_field,nullptr,0);
      }
      if(temp_field.compare("INT32_PARAMETER")==0){
        internal_req.param_type = hdt_adroit_debug::ReadDriveParam::Response::INT32_PARAMETER;
        std::getline(temp_stream,temp_field,',');
        internal_req.value_int32 = std::stol(temp_field,nullptr,0);
      }
      if(temp_field.compare("FLOAT32_PARAMETER")==0){
        internal_req.param_type = hdt_adroit_debug::ReadDriveParam::Response::FLOAT32_PARAMETER;
        std::getline(temp_stream,temp_field,',');
        internal_req.value_float32 = std::stof(temp_field); 
      }
      if(temp_field.compare("UINT16_PARAMETER")==0){
        internal_req.param_type = hdt_adroit_debug::ReadDriveParam::Response::UINT16_PARAMETER;
        std::getline(temp_stream,temp_field,',');
        internal_req.value_uint16[0] = std::stoul(temp_field,nullptr,0); 
        std::getline(temp_stream,temp_field,',');
        internal_req.value_uint16[1] = std::stoul(temp_field,nullptr,0); 
      }
      if(temp_field.compare("UINT8_PARAMETER")==0){
        internal_req.param_type = hdt_adroit_debug::ReadDriveParam::Response::UINT8_PARAMETER;
        std::getline(temp_stream,temp_field,',');
        internal_req.value_uint8[0] = std::stoul(temp_field,nullptr,0); 
        std::getline(temp_stream,temp_field,',');
        internal_req.value_uint8[1] = std::stoul(temp_field,nullptr,0); 
        std::getline(temp_stream,temp_field,',');
        internal_req.value_uint8[2] = std::stoul(temp_field,nullptr,0); 
        std::getline(temp_stream,temp_field,',');
        internal_req.value_uint8[3] = std::stoul(temp_field,nullptr,0); 
      }
      
      if(!WriteParamServiceCallback(internal_req,internal_res)){
        ROS_ERROR("AdroitConsole::WriteAllParamServiceCallback - error writing line %d",line_counter);
        //res.ret=1;
        return false;
      }
    }
    
    input_file.close();
  }else{
    ROS_ERROR("AdroitConsole::WriteAllParamServiceCallback - invalid input, param file");
    //res.ret=1;
    return false;
  }
  
  return true;
}
/*---------------------------------------------------------------------------
  commit callback
 *--------------------------------------------------------------------------*/

bool AdroitConsole::CommitServiceCallback(hdt_adroit_debug::Commit::Request& req, hdt_adroit_debug::Commit::Response& res) {
  if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::CommitServiceCallback - drive %s or CAN_ID %d",req.name.c_str(),req.can_id);
  
  // check for valid CAN ID or known joint name
  std::string return_drive_name;
  uint8_t return_CAN_ID;
  if(ResolveCANid(req.name, req.can_id, &return_drive_name, &return_CAN_ID)){
    //ROS_INFO("input %s %d but using %s %d",req.name.c_str(),req.can_id,return_drive_name.c_str(),return_CAN_ID);
  }else{
    ROS_ERROR("AdroitConsole::CommitServiceCallback - invalid input");
    res.ret=1;
    return false;
  }
  
  int addr = return_CAN_ID;
  
  // commit parameters
  if(AdroitComs::commit_parameters(addr, &drive_data[addr])) {
    ROS_INFO("commit successful");
    res.ret=0;
  }
  else {
    ROS_ERROR("commit failed");
    res.ret=1;
  }
  
  return true;
}
/*---------------------------------------------------------------------------
  select state callback
 *--------------------------------------------------------------------------*/

bool AdroitConsole::StateSelectServiceCallback(hdt_adroit_debug::StateSelect::Request& req, hdt_adroit_debug::StateSelect::Response& res) {
  if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::StateSelectServiceCallback - drive %s or CAN_ID %d",req.name.c_str(),req.can_id);
  
  // check for valid CAN ID or known joint name
  std::string return_drive_name;
  uint8_t return_CAN_ID;
  if(ResolveCANid(req.name, req.can_id, &return_drive_name, &return_CAN_ID)){
    //ROS_INFO("input %s %d but using %s %d",req.name.c_str(),req.can_id,return_drive_name.c_str(),return_CAN_ID);
  }else{
    ROS_ERROR("AdroitConsole::StateSelectServiceCallback - invalid input");
    res.ret=1;
    return false;
  }
  
  // send state command
  if(req.state > AdroitComs::NO_STATE and req.state < AdroitComs::NUM_STATES) {
    AdroitComs::StateCmd cmd;
    cmd.state = req.state;
    AdroitComs::send_state_cmd(return_CAN_ID, &cmd);
  }
  else {
    ROS_ERROR("invalid arguments...");
    res.ret=1;
    return false;
  }

  res.ret=0;
  return true;
}

/*---------------------------------------------------------------------------
  get status callback
 *--------------------------------------------------------------------------*/

bool AdroitConsole::GetStatusServiceCallback(hdt_adroit_debug::GetStatus::Request& req, hdt_adroit_debug::GetStatus::Response& res) {
  if(CALLBACK_DEBUG) ROS_INFO("AdroitConsole::GetStatusServiceCallback");
  
  // reset msg_update
  for(int i = 0; i < MAX_DRIVES; i++) {
    drive_data[i].msg_update[AdroitComs::STATUS_TELEM] = false;
  }

  // send status command and sleep for a moment
  AdroitComs::send_status_cmd(BROADCAST_ADDR);
  usleep(100*1000);
  // check for responses
  for(int i = 0; i < MAX_DRIVES; i++) {
    if(drive_data[i].msg_update[AdroitComs::STATUS_TELEM] == true) {
      res.addr.push_back((uint32_t)i);
      res.state.push_back((uint32_t)drive_data[i].status_telem.state);
    }
    else {
      AdroitComs::reset_drive(&drive_data[i]);
    }
  }
  
  return true;
}

/*---------------------------------------------------------------------------
  helper function - check drive name and CAN id against joints in hardware yaml file, if present
 *--------------------------------------------------------------------------*/
bool AdroitConsole::ResolveCANid(std::string input_drive_name, uint8_t input_CAN_ID, std::string *return_drive_name, uint8_t *return_CAN_ID){

  // if joint name is populated and CAN ID is zero, use name
  // else use CAN ID

  if(input_CAN_ID==0 and input_drive_name.length()>0){
    // try to use joint name from hardware yaml file
    // check for valid drive name
    if(drive_names.count(input_drive_name)){
      // known drive name
      *return_CAN_ID = drive_names[input_drive_name];
      *return_drive_name = input_drive_name;
    }else{
      // unknown drive name
      ROS_WARN("AdroitConsole - specified drive name %s and CAN id %d are invalid",input_drive_name.c_str(),input_CAN_ID);
      return false;
    } 
  }else{
    // use CAN ID
    if(input_CAN_ID >= 0 and input_CAN_ID < MAX_DRIVES){
      // valid CAN ID - check if name matches
      *return_CAN_ID = input_CAN_ID;
      auto it = drive_names.begin();
      while(it != drive_names.end()){
        if(it->second == input_CAN_ID){
          *return_drive_name = it->first;
          break;
        }
        it++;
      }
      if(*return_drive_name != input_drive_name and input_drive_name.length()>0){
        ROS_WARN("AdroitConsole - specified drive name %s but CAN id %d is named %s",input_drive_name.c_str(),input_CAN_ID,return_drive_name->c_str());
      }
    }else{
      // invalid CAN ID
      ROS_WARN("AdroitConsole - specified drive name %s and CAN id %d are invalid",input_drive_name.c_str(),input_CAN_ID);
      return false;
    }
  
  }

  return true;

}

/*---------------------------------------------------------------------------
  helper function - parameter name and parameter number against #defines in adroit_params.h
 *--------------------------------------------------------------------------*/
bool AdroitConsole::ResolveParameter(std::string input_param_name, uint16_t input_param_num, std::string *return_param_name, uint16_t *return_param_num, uint8_t *return_param_type){

  // if param name is populated and param number is zero, use name
  // else use param number 


  uint16_t temp_index = 0;
  
  // index 0 is a real parameter but not used often, check if parameter name has been specified
  if(input_param_num == 0 and input_param_name.length()>0){
    // TODO this std::find doesn't seem to be working
    //std::vector<std:: string>::iterator temp_pointer = std::find(ParameterNameListLocal.begin(),ParameterNameListLocal.end(),req.param_name);
    
    auto it = ParameterNameListLocal.begin();
    while(it != ParameterNameListLocal.end()){
      if(input_param_name.compare(*it)==0){
        *return_param_name = *it;
        //ROS_INFO("%s == %s bingo",input_param_name.c_str(),it->c_str());
        temp_index = std::distance(ParameterNameListLocal.begin(),it);
        *return_param_num = ParameterIndexListLocal[temp_index]; 
        break;
      }else{
        //ROS_INFO("%s != %s",input_param_name.c_str(),it->c_str());
      }
      it++;
    }
    if(it == ParameterNameListLocal.end()){
      // didn't find a match
      ROS_WARN("AdroitConsole - specified parameter name %s is invalid",input_param_name.c_str());
      return false;
    }
  }else{
    // check parameter number
    uint16_t *temp_pointer = std::find(ParameterIndexListLocal,std::end(ParameterIndexListLocal),input_param_num);
        // find index in ParameterIndexList
    if (temp_pointer == std::end(ParameterIndexListLocal)){
      for(int i=0;i<ParameterListLength; i++){
        if(ParameterIndexListLocal[i]<input_param_num){
          temp_index = i;
        }else{
          break;
        }
      }
      ROS_WARN("couldn't find param_number %u in list so inherit from %u",input_param_num,ParameterIndexListLocal[temp_index]);
      *return_param_name = "spare";
      *return_param_num = input_param_num; 
      *return_param_type = ParameterDataTypeListLocal[temp_index];
    }else{
      temp_index = std::distance(ParameterIndexListLocal,temp_pointer);
      *return_param_name = ParameterNameListLocal[temp_index];
      *return_param_num = ParameterIndexListLocal[temp_index]; 
      *return_param_type = ParameterDataTypeListLocal[temp_index];
    }
    if(input_param_name.compare(*return_param_name)!=0 and input_param_name.length()>0){
      ROS_WARN("AdroitConsole - specified parameter name %s but parameter number %d is named %s",input_param_name.c_str(),input_param_num,return_param_name->c_str());
    }
  }
  
  return true;
}

/*----------------------------------------------------------------------------
  main function
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{

  ros::init(argc, argv, "hdt_adroit_debug");
  ros::NodeHandle node_handle;
  AdroitConsole console;
  console.init();
  
  // wait for shutdown
  //ros::spin();
    
  return 0;
}
