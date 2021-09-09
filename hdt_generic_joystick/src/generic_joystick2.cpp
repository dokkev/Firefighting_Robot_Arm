/*
joystick controller for HDT maniuplator arms, using Moveit! and ROSserial TCP connection

TODO list:
  be able to control end effector when controlling the manipulator it's on (currently have to control manipulator and end effector descretely)
  add support for dual arm joint-by-joint control, potentially through combined joint group in Moveit! setup assistant
  add support for dual arm end-point control, potentially through combined joint group in Moveit! setup assistant or parameters in launch file

*/

#include <hdt_generic_joystick/generic_joystick2.h>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
//TODO: Are all 3 of these needed?
#include <ros/ros.h>
//#include <joint_limits_interface/joint_limits.h>
//#include <joint_limits_interface/joint_limits_urdf.h>
//#include <joint_limits_interface/joint_limits_rosparam.h>

#include <iostream>
#include <utility> //was stdlib.h

#include <moveit_msgs/DisplayRobotState.h>
#include <hdt_generic_joystick/MsgWrapper.h>
#include <hdt_generic_joystick/JoystickMap.h>
#include <hdt_generic_joystick/adroit_constants.h>


#define wait_time 2.0 // how long to wait, in seconds, for the moveit getCurrentState() calls

//std_msgs::ColorRGBA color_current,color_collision_, color_wait_, color_joint_position_,color_joint_velocity_, color_endpoint_, color_endpoint_relative_, color_trajectory_;
std_msgs::ColorRGBA color_current;

collision_detection::CollisionRequest collision_request;
collision_detection::CollisionResult collision_result;


// future expansion - look at adding interactive markers to joints for on screen control
/*----------------------------------------------------------------------------
  handle feedback from interactive markers
 *----------------------------------------------------------------------------
void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  
  //ROS_INFO_STREAM(feedback->marker_name << " is now at " << feedback->pose.position.x << " , " << feedback->pose.position.y
    //<< " , " << feedback->pose.position.z << " : " << feedback->pose.orientation.w << " , " << feedback->pose.orientation.x
     //<< " , " << feedback->pose.orientation.y << " , " << feedback->pose.orientation.z);
     
  //ROS_INFO("marker %s at [%1.2f,%1.2f,%1.2f,%1.2f]",feedback->marker_name.c_str(),feedback->pose.orientation.w,feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z);
  
  tf2::Quaternion q;
  tf2::fromMsg(feedback->pose.orientation,q);
  //ROS_INFO("RPY [%1.2f,%1.2f,%1.2f]",q.
  ROS_INFO("marker %s angle [%1.2f]",feedback->marker_name.c_str(),q.getAngle());
}
*/


/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
JoystickControl::JoystickControl(const ros::NodeHandle& nh) : nh_(nh), space_msg_()
{

	mode_ = MODE_NONE;

	//  joy_wrap_msg_.getMsg() = joy_msg_;
	//  joy_wrap_msg_.getMsg().axes[5] = (float)1.0;

	  joy_sub_ = nh_.subscribe("joy", 10, &JoystickControl::joyCallback, this);
	  // check for private parameter indicating if end point control should use a spacemouse or joystick
	  if(! ros::param::get("~use_spacemouse", use_spacemouse)){
	    use_spacemouse = true;
	  }
	  if(use_spacemouse){
	    space_sub_ = nh_.subscribe("spacenav/twist", 10, &JoystickControl::spaceCallback, this);
	  }
	  
	  connection_sub_ = nh_.subscribe("manipulator_connected", 10, &JoystickControl::connectionCallback, this);

	  // check for robot description
	  std::string description;
	  if(nh_.getParam("robot_description", description)) {
	    // are we using the robot_description parameter to just checking if it exist?
	    // load robot model
	    model_loader = new robot_model_loader::RobotModelLoader("robot_description");
	    kinematic_model = model_loader->getModel();

	    // get kinematic state
	    kinematic_state = new robot_state::RobotState(kinematic_model);

	    // check for avalible control groups in the robot model joystick_control
	    std::vector<moveit::core::JointModelGroup*> temp_groups = kinematic_model->getJointModelGroups();
	    ROS_INFO("JoystickControl::init found %i control groups",(int)temp_groups.size());
	    for(std::size_t it = 0; it < temp_groups.size(); ++it){

	      // check if control group is an end effector
	      if(temp_groups[it]->isEndEffector()){
	        // don't add end effectors to joint group list, plan on controlling it joint-by-joint with the arm it's on
	        ROS_INFO("%s is an end effector attached to %s",temp_groups[it]->getName().c_str(),temp_groups[it]->getEndEffectorParentGroup().first.c_str());
	        // build dictionary of arm to end effector mapping
	        end_effector_map_[temp_groups[it]->getEndEffectorParentGroup().first] = temp_groups[it]->getName();
	      }else{
	        //ROS_INFO("%s is not an end effector",temp_groups[it]->getName().c_str());
	        //const std::vector<unsigned int> bijection = temp_groups[it]->getKinematicsSolverJointBijection();
	        if(temp_groups[it]->getKinematicsSolverJointBijection().size()>0){
	          ROS_INFO("%s is an arm group with an IK solver",temp_groups[it]->getName().c_str());
	        }else{
	          ROS_INFO("%s is an arm group",temp_groups[it]->getName().c_str());
	        }
	        // add arms (non-end-effectors) to joint and move groups
	        // why do we need both a joint and move group for each arm?
	        joint_model_groups_.push_back(temp_groups[it]);
	        move_groups_.push_back(std::make_shared<moveit::planning_interface::MoveGroupInterface>(temp_groups[it]->getName()));
	      }

	      // read through joint names in each joint model group
	      joint_names_ = temp_groups[it]->getActiveJointModelNames();
	      const moveit::core::JointBoundsVector& bounds_vector  = temp_groups[it]->getActiveJointModelsBounds();
	      for(std::size_t k=0; k<joint_names_.size(); ++k){
	        moveit::core::JointModel::Bounds temp_bounds = *bounds_vector[k];
	        // build dictionary of moveit::core::VariableBounds
	        joint_bounds_[joint_names_[k]]=temp_bounds[0];
	        //ROS_INFO("%s has position limits [%2.2f,%2.2f]",joint_names_[k].c_str(),joint_bounds_[joint_names_[k]].max_position_,joint_bounds_[joint_names_[k]].min_position_);
	      }
	    }

	  }else{
	    ROS_ERROR("No model found, exiting");
//	    return false;
	  }


	  // initialize move group
	  //ROS_INFO("move group %s has endpoint %s",move_group_->getName().c_str(),move_group_->getEndEffectorLink().c_str());

	  // Debug statement to print out joint bounds
	  for(std::map<std::string,moveit::core::VariableBounds>::iterator it=joint_bounds_.begin(); it!=joint_bounds_.end(); ++it){

	    ROS_INFO("%s has position limits [%2.2f,%2.2f]",it->first.c_str(),it->second.max_position_,it->second.min_position_);
	  }


	  // load position controllers for individual drives, used for joint-by-joint control
	  std::string controller_prefix;
	  ros::param::get("~controller_prefix", controller_prefix);

	  for(std::map<std::string,moveit::core::VariableBounds>::iterator it=joint_bounds_.begin(); it!=joint_bounds_.end(); ++it){
	    /*
	    // move controller loading to launch file
	    // add joint to controller list
	    std::string controller = controller_prefix +"/" + it->first + "_position_controller";
	    joint_controllers_.push_back(controller);

      */
	    // add joint to pub list
	    std::string topic_pos = controller_prefix +"/" + it->first + "_position_controller" + "/command";
	    ros::Publisher pub_pos = nh_.advertise<std_msgs::Float64>(topic_pos, 10);
	    joint_position_pubs_[it->first] = pub_pos;
	    
	    std::string topic_vel = controller_prefix +"/" + it->first + "_velocity_controller" + "/command";
	    ros::Publisher pub_vel = nh_.advertise<std_msgs::Float64>(topic_vel, 10);
	    joint_velocity_pubs_[it->first] = pub_vel;
	    //ROS_INFO("creating publisher for %s", joint_name.c_str());
	  }

    /*
	    // move controller loading to launch file
	  // load joint controllers
	  ros::service::waitForService(LOAD_CONTROLLER);
	  for (auto& controller : joint_controllers_)
	  {
	    ros::ServiceClient load_controller = nh_.serviceClient<controller_manager_msgs::LoadController>(LOAD_CONTROLLER, true);

	    controller_manager_msgs::LoadController srv;
	    srv.request.name = controller;
	    if (load_controller.call(srv))
	    {
	      if (!srv.response.ok)
	      {
	        ROS_WARN("could not load controller %s", controller.c_str());
	        //return false;
	      }
	    }
	  }
    */
    
	  // ready dictionary of button mapping for changing functions/modes
	  // have to use workround because the roscpp getParam() is not as flexiable as the rospy version
	  std::string ns = ros::this_node::getNamespace();
	  std::string prefix;
	  prefix = ns;
	  prefix += "/button_map/";
	  prefix.erase(0, 1);
	  ROS_INFO_STREAM("prefix: " << prefix);
	  std::vector<std::string> keys;
	  nh_.getParamNames(keys);
	  //std::map<transport::Address, std::string> JAUS_addresses;
	  std::size_t found;
	  for(unsigned i=0; i < keys.size(); i++) {
	    found = keys[i].find(prefix);
	    if (found!=std::string::npos){
	      found = keys[i].find("axisA");
	      if (found!=std::string::npos){
	        //ROS_INFO_STREAM("key: " << keys[i]);
	        std::string temp_string = keys[i].substr(prefix.length(),keys[i].length());
	        temp_string = temp_string.substr(0,temp_string.find("/"));
	        //ROS_INFO_STREAM("key_short: " << temp_string);
	        int temp_int;
	        nh_.getParam(keys[i],temp_int);
	        button_map[temp_string]=temp_int;
	      }
	    }
	  }

	  for(std::map<std::string, int>::iterator it=button_map.begin(); it!=button_map.end(); ++it){
	    ROS_INFO("%s maps to button %d",it->first.c_str(),it->second);
	  }

	  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &JoystickControl::jointstateCallback, this);
	  display_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("joystick_control_display", 10);
	  pose_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("joystick_control_poses", 10);
	  
	  
	  std::string status_message_frame;
	  ros::param::get("~status_message_frame", status_message_frame);
    if(status_message_frame.length()>0){
      status_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("joystick_control_status", 10);
      status_marker.header.frame_id = status_message_frame;
      status_marker.header.stamp = ros::Time();
      //status_marker.ns="feedback_namespace";
      status_marker.id=0;
      status_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      status_marker.action = visualization_msgs::Marker::ADD;
      // arbitrarily pick stome static sizes (in meters) to get started
      status_marker.scale.z = 0.05;
      status_marker.pose.position.y = 0.25;
    }
	  
	  joint_group_index = 0;
	  current_joint_model_= joint_model_groups_[joint_group_index];

	  joystick_map_  = JoystickMap(nh);
	  for(std::map<std::string, JoyMapEntry >::const_iterator it=joystick_map_.begin(); it!=joystick_map_.end(); ++it){
	  	    ROS_INFO("%s maps to joystick %i",it->first.c_str(),it->second.axisA);
	  	  }


  // read color for Rviz feedback from paramaters, should be defined in a yaml file
  // TODO push all of this into a helper function or joystick feedback class
  
  //std::map<std::string,std::vector<double>> color_dict;
  //ros::param::get("joystick_feedback_colors", color_dict);
  
  std::vector<double> temp_color;
  std::string temp_param;
  for(unsigned i=0; i < Mode_Names_.size(); i++) {
    temp_param = "joystick_feedback_colors/" + Mode_Names_[i];
    if (ros::param::has(temp_param)){
      ros::param::get(temp_param, temp_color);
      joystick_display_colors[i].a=temp_color[0];
      joystick_display_colors[i].r=temp_color[1];
      joystick_display_colors[i].b=temp_color[2];
      joystick_display_colors[i].g=temp_color[3];
      ROS_DEBUG("JoystickControl::JoystickControl found color for %s",Mode_Names_[i].c_str());
    }else{
      // default to a grey color
      joystick_display_colors[i].a=1;
      joystick_display_colors[i].r=0.5;
      joystick_display_colors[i].b=0.5;
      joystick_display_colors[i].g=0.5;
      ROS_WARN("JoystickControl::JoystickControl no color defined for %s",Mode_Names_[i].c_str());
    }
  }
 
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
JoystickControl::~JoystickControl()
{
}

/*----------------------------------------------------------------------------
  run
 *----------------------------------------------------------------------------*/
void JoystickControl::run()
{
  std::string name;
  ros::Rate rate(LOOP_RATE);
  std::string temp_string;
  std::vector<unsigned int> bijection;

  ROS_INFO("JoystickControl::run starting to loop");
  
  // future expansion - look at adding interactive markers to joints for on screen control
  /*
  // interactive marker to show which joints are being controlled3
  interactive_markers::InteractiveMarkerServer marker_server("joystick_control");
  bool new_markers = true;
  */
  
  //TODO check if system starts in self-collision or outside position limits
  // give user feedback and method to move back into usable configuration
  planning_scene::PlanningScene planning_scene(kinematic_model);
  // using local RobotState only seems to check collision against zero position of other arms
  // try using global state so it's udpated every cycle?
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  updateCollision(current_state);
  
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request,collision_result);
  if(collision_result.collision){
    ROS_WARN("starting in collision:");
    std::vector<std::string> colliding_links;
    planning_scene.getCollidingLinks(colliding_links,current_state);
    for(int i=0; i<colliding_links.size(); ++i){
      ROS_WARN("\t%s",colliding_links[i].c_str());
    }
    ROS_WARN("use collision override and joint-by-joint to move out of collision");
  }
  
  
  moveit_msgs::DisplayRobotState temp_display;
  moveit_msgs::DisplayRobotState pose_display;
  // populate and publish a pose_display with all links in the robot model set transparent
  moveit_msgs::ObjectColor pose_object;
  pose_object.color = joystick_display_colors[MODE_NAMED];
  std::vector<std::string> temp_links = kinematic_model->getLinkModelNames();
  for (std::size_t j = 0; j < temp_links.size(); j++){
    pose_object.id = temp_links[j];
    pose_object.color.a = 0;
    pose_display.highlight_links.push_back(pose_object);
  }
  pose_pub_.publish(pose_display);
  
  
  color_current = joystick_display_colors[MODE_NONE];
  
  // this first_cycle flag keeps the controller from trying to move a group that hasn't bee initialiazed (joint names, positions, etc) 
  // but it also makes the control group jump to the second position when any button on the controller is pressed
  // TODO find a better setup
  bool first_cycle = true;
  joint_group_index=0;
  collision_enable = false;
  while (ros::ok())
  {
  
  

	  
    // acquire mutex
    std::unique_lock<std::mutex> lock(mutex_);
    // update joy prev
    joy_wrap_msg_.setMsg(joy_msg_);
    space_prev_ = space_holder_;
    space_msg_.setMsg(space_holder_);
    lock.unlock();
    
    // check if we've lost conenction to the manipulator
    if(!manipulator_connected){
      if(mode_ != MODE_RESET){
        ROS_WARN("JoystickControl::run manipulator NOT connected");
        color_current = joystick_display_colors[MODE_RESET];
        mode_ = switchMode(MODE_RESET);
      }

    }else{
      if(mode_ == MODE_RESET){
        ROS_WARN("JoystickControl::run manipulator reconnected connected");
        mode_ = switchMode(MODE_NONE);
        color_current = joystick_display_colors[MODE_NONE];
      }
    }
    
    // change control group, cycle through list read from robot model
    // check for button existance in button_map and that joystick button array is large enough
    // TODO this check doesn't need to happen every cycle, find a way to only do it once?
    temp_string = "change_drive_set";
    if(button_map.count(temp_string)!=0){
      if(button_map[temp_string]>=0 and button_map[temp_string]<joy_wrap_msg_.getMsg().buttons.size()){
        if (((joy_wrap_msg_.getMsg().buttons[button_map[temp_string]] == 0) && (joy_wrap_msg_.getPrevMsg().buttons[button_map[temp_string]] == 1)) or first_cycle)
        {
          ROS_INFO("JoystickControl::run change_drive_set");
          first_cycle = false;
          joint_group_index++;
          if(joint_group_index>=joint_model_groups_.size()){
            joint_group_index=0;
          }
          current_joint_model_= joint_model_groups_[joint_group_index];
          joint_names_ = current_joint_model_->getActiveJointModelNames(); //important!
          ROS_INFO("controlling joint group %s:",current_joint_model_->getName().c_str());
          for(std::size_t k=0; k<joint_names_.size(); ++k){
            ROS_INFO("\t%s",joint_names_[k].c_str());
          }
          if(end_effector_map_.count(current_joint_model_->getName()) != 0){
            ROS_INFO("\twith end effector %s:",end_effector_map_[current_joint_model_->getName()].c_str());
            EE_group = kinematic_model->getJointModelGroup(end_effector_map_[current_joint_model_->getName()]);
            EE_names_ = EE_group->getActiveJointModelNames();
            for(std::size_t k=0; k<EE_names_.size(); ++k){
              ROS_INFO("\t%s",EE_names_[k].c_str());
            }
          }else{
            EE_names_.clear();
          }
          //move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(joint_model_groups_[joint_model_index]->getName());
          named_targets = move_groups_[joint_group_index]->getNamedTargets();
          named_target_index = named_targets.size(); // set index to last entry, when named_cycle button pressed it will roll over to first entry

          // update positions of all the drives in the "current_state" RobotState so that collision avoidance will work properly
          updateCollision(current_state);
          mode_ = switchMode(MODE_NONE);
          color_current = joystick_display_colors[MODE_NONE];
          
          // clear named pose display state when changing modes
          for (size_t j = 0; j < pose_display.highlight_links.size(); j++){
            pose_display.highlight_links[j].color.a = 0;
          }
          pose_pub_.publish(pose_display);
        }
      }
    }

    // toggle between joint by joint and end point controler
    temp_string = "control_mode";
    if(button_map.count(temp_string)!=0){
      if(button_map[temp_string]>=0 and button_map[temp_string]<joy_wrap_msg_.getMsg().buttons.size()){
        if ((joy_wrap_msg_.getMsg().buttons[button_map[temp_string]] == 0) && (joy_wrap_msg_.getPrevMsg().buttons[button_map[temp_string]] == 1))
        {
          ROS_INFO("JoystickControl::run control_mode");
          // toggle joint mode
          switch (mode_)
          {// future expansion - look at adding interactive markers to joints for on screen control
            case MODE_JOINT_POSITION:
              mode_ = switchMode(MODE_JOINT_VELOCITY);
              color_current = joystick_display_colors[MODE_JOINT_VELOCITY];
              break;
            case MODE_JOINT_VELOCITY:
              bijection = current_joint_model_->getKinematicsSolverJointBijection();
              //TODO replaced hardcoded names "dual_arms" with a more configurable sollution
              if(bijection.size()>0 or current_joint_model_->getName() == "dual_arms"){
                ROS_INFO("joint group %s supports end point control",current_joint_model_->getName().c_str());
                mode_ = switchMode(MODE_ENDPOINT);
                color_current = joystick_display_colors[MODE_ENDPOINT];
              }else{
                ROS_INFO("joint group %s does not support end point control",current_joint_model_->getName().c_str());
                //mode_ = switchMode(MODE_NONE);
              }
              break;
            case MODE_ENDPOINT:
              mode_ = switchMode(MODE_ENDPOINT_EE);
              color_current = joystick_display_colors[MODE_ENDPOINT_EE];
              break;
            case MODE_ENDPOINT_EE:
              mode_ = switchMode(MODE_JOINT_POSITION);
              color_current = joystick_display_colors[MODE_JOINT_POSITION];
              break;
            default:
              mode_ = switchMode(MODE_JOINT_POSITION);
              color_current = joystick_display_colors[MODE_JOINT_POSITION];
              break;
          }
          // update positions of all the drives in the "current_state" RobotState so that collision avoidance will work properly
          updateCollision(current_state);
          
        }
      }
    }

    // cycle through named poses
    temp_string = "named_cycle";
    if(button_map.count(temp_string)!=0){
      if(button_map[temp_string]>=0 and button_map[temp_string]<joy_wrap_msg_.getMsg().buttons.size()){
        if ((joy_wrap_msg_.getMsg().buttons[button_map[temp_string]] == 0) && (joy_wrap_msg_.getPrevMsg().buttons[button_map[temp_string]] == 1))
        {
          named_target_index++;
          // assume pose display topic will be visiable
          bool pose_alpha = 1;
          if(named_target_index>named_targets.size()){
            // if index has exceed list and dead state go back to zero
            named_target_index = 0;
          }else if(named_target_index==named_targets.size()){
            // use first position after end of list as dead state, make pose display topic transparent
            pose_alpha = 0;
          }
          // handle groups that don't have named poses, make pose display topic transparent
          if(named_targets.size()==0){
            pose_alpha = 0;
          }
          // create display state for named pose
          std::unique_lock<std::mutex> lock2(joint_state_mutex_);
          pose_display.state.joint_state = joint_state_last_;
          lock2.unlock();
          // loop though highlighted links and set those in this move group to the trajactory color
          // set alpha color based on named pose list, if in null position (index=list.size()) make the links transparent
          std::vector<std::string> temp_links = move_groups_[joint_group_index]->getLinkNames();
          for (size_t j = 0; j < pose_display.highlight_links.size(); j++){
            pose_display.highlight_links[j].color.a = 0;
            for (size_t i = 0; i < temp_links.size(); i++){
              if(pose_display.highlight_links[j].id == temp_links[i]){
                pose_display.highlight_links[j].color = joystick_display_colors[MODE_NAMED];
                pose_display.highlight_links[j].color.a = pose_alpha;
              }
            }
            //ROS_INFO("\t%s has alpha %f",pose_display.highlight_links[j].id.c_str(),pose_display.highlight_links[j].color.a);
          }
 
          if(named_target_index < named_targets.size()){
            ROS_INFO("named_cycle %s (%d out of %d)",named_targets[named_target_index].c_str(),named_target_index,named_targets.size());
            // plan and show motion in RVIZ so user can decide if they want to execute
            move_groups_[joint_group_index]->setNamedTarget(named_targets[named_target_index]);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            move_groups_[joint_group_index]->plan(plan);
            
            // set joint positions in pose display state to match current position
            std::map<std::string,double> pose_display_data;
            // set joint positions for this move group to the positions of the named pose
            pose_display_data = move_groups_[joint_group_index]->getNamedTargetValues(named_targets[named_target_index].c_str());
            for(std::map<std::string, double>::iterator it=pose_display_data.begin(); it!=pose_display_data.end(); ++it){
              ROS_INFO("\t%s at %f",it->first.c_str(),it->second);
              for (size_t j = 0; j < pose_display.state.joint_state.name.size(); j++){
                if(pose_display.state.joint_state.name[j]==it->first){
                  //ROS_INFO("\t change %s from %f to %f",it->first.c_str(),pose_display.state.joint_state.position[j],it->second);
                  pose_display.state.joint_state.position[j] = it->second;
                  //ROS_INFO("\t yes ?%f",pose_display.state.joint_state.position[j]);
                }
              }
            }
          }else{
            ROS_WARN("no named targets");
          }
          pose_pub_.publish(pose_display);
        }
      }
    }

    // execute named pose
    temp_string = "named_execute";
    if(button_map.count(temp_string)!=0){
      if(button_map[temp_string]>=0 and button_map[temp_string]<joy_wrap_msg_.getMsg().buttons.size()){
        if ((joy_wrap_msg_.getMsg().buttons[button_map[temp_string]] == 0) && (joy_wrap_msg_.getPrevMsg().buttons[button_map[temp_string]] == 1))
        {
          if(named_target_index<named_targets.size()){
            ROS_INFO("JoystickControl::run named_execute %s",named_targets[named_target_index].c_str());
            // set named mode and name
            mode_ = switchMode(MODE_NAMED);
            name = named_targets[named_target_index];
            color_current = joystick_display_colors[MODE_NAMED];
          }else{
            ROS_INFO("JoystickControl::run named_execute, no pose specified");
          }
          // go to end of pose list, so next named_cycle button press starts at the begining of the list
          named_target_index=named_targets.size();
          // clear named pose display state when changing modes
          for (size_t j = 0; j < pose_display.highlight_links.size(); j++){
            pose_display.highlight_links[j].color.a = 0;
          }
          pose_pub_.publish(pose_display);
        }
      }
    }

    // execute named pose
    temp_string = "stop_control";
    if(button_map.count(temp_string)!=0){
      if(button_map[temp_string]>=0 and button_map[temp_string]<joy_wrap_msg_.getMsg().buttons.size()){
        if ((joy_wrap_msg_.getMsg().buttons[button_map[temp_string]] == 0) && (joy_wrap_msg_.getPrevMsg().buttons[button_map[temp_string]] == 1))
        {
          ROS_INFO("JoystickControl::run stop_control");
          // call move_group.stop() method incase a trajector is being executed
          for(auto move_group : move_groups_){
        	  move_group->stop();
          }
          mode_ = switchMode(MODE_NONE);
          color_current = joystick_display_colors[MODE_NONE];
        }
      }
    }
    
    // toggle collison avoidance on/off
    temp_string = "collision_enable";
    if(button_map.count(temp_string)!=0){
      if(button_map[temp_string]>=0 and button_map[temp_string]<joy_wrap_msg_.getMsg().buttons.size()){
        if ((joy_wrap_msg_.getMsg().buttons[button_map[temp_string]] == 0) && (joy_wrap_msg_.getPrevMsg().buttons[button_map[temp_string]] == 1))
        {
          collision_enable = !collision_enable;
          if(collision_enable){
            ROS_INFO("JoystickControl::run collision avoidance disabled");
          }else{
            ROS_INFO("JoystickControl::run collision avoidance enabled");
          }
        }
      }
    }

    // create displayrobot model for debug in rviz
    std::unique_lock<std::mutex> lock2(joint_state_mutex_);
    temp_display.state.joint_state = joint_state_last_;
    lock2.unlock();
    temp_display.highlight_links.clear();
    moveit_msgs::ObjectColor temp_object;
    temp_object.color = color_current;
    // populated status message for joystick controller
    /*
    const moveit::core::JointModel* something = current_joint_model_->getCommonRoot();
    const moveit::core::LinkModel* child = something->getChildLinkModel();
    const moveit::core::LinkModel* parent = something->getParentLinkModel();
	  status_marker.header.frame_id = parent->getName();
	  status_marker.header.stamp = ros::Time();
	  //status_marker.text="child: " + child->getName()+"\nparent: " + parent->getName();
    */
	  status_marker.color.a = color_current.a;
	  status_marker.color.r = color_current.r;
	  status_marker.color.g = color_current.g;
	  status_marker.color.b = color_current.b;
	  status_marker.text="group: " + current_joint_model_->getName()+"\nmode: " + Mode_Names_[mode_];
	  status_marker_pub_.publish(status_marker);
	  
    std::vector<std::string> temp_links = move_groups_[joint_group_index]->getLinkNames();
    for (size_t i = 0; i < temp_links.size(); i++){
      temp_object.id = temp_links[i];
      temp_display.highlight_links.push_back(temp_object);
    }
			// take mode-dependent action
			switch (mode_) {
			case MODE_NAMED: {
				move_groups_[joint_group_index]->setNamedTarget(name);
				moveit::planning_interface::MoveGroupInterface::Plan plan;
				move_groups_[joint_group_index]->plan(plan);
				move_groups_[joint_group_index]->asyncExecute(plan);
				// the .asyncExecute() method is non-blocking so we can stop it with the .stop() method
				mode_ = switchMode(MODE_NONE);
				break;
			}
			case MODE_JOINT_VELOCITY: {
				jointByJointControlVelocity(current_state, temp_display, planning_scene);
				break;
			}
			case MODE_JOINT_POSITION: {
				jointByJointControlPosition(current_state, temp_display, planning_scene);
				break;
			}
			case MODE_ENDPOINT: {
				// quick add on to use joystick for end point control
				//TODO replaces the hard coded indexes with data from the joystick map yaml file
				if (!use_spacemouse) {
					space_prev_.linear.x = joy_wrap_msg_.getMsg().axes[4]*0.25;
					space_prev_.linear.y = joy_wrap_msg_.getMsg().axes[3]*0.25;
					space_prev_.linear.z = joy_wrap_msg_.getMsg().axes[1]*0.25;
					space_prev_.angular.x = -joy_wrap_msg_.getMsg().buttons[4] + joy_wrap_msg_.getMsg().buttons[5];
					space_prev_.angular.y = joy_wrap_msg_.getMsg().axes[7]*0.7;
					space_prev_.angular.z = joy_wrap_msg_.getMsg().axes[6]*0.7;
					space_msg_.setMsg(space_prev_);
				}
        //TODO replamoveit_msgs::ObjectColor pose_object;ced hardcoded names "dual_arms" with a more configurable sollution
				if (current_joint_model_->getName() == "dual_arms") {
					dualEndPointControl(current_state, temp_display, planning_scene, space_holder_);
				} else {
					endPointControl(current_state, temp_display, planning_scene);
				}

				break;
			}
			case MODE_ENDPOINT_EE: {
				// quick add on to use joystick for end point control
				//TODO replaces the hard coded indexes with data from the joystick map yaml file
				if (!use_spacemouse) {
					space_prev_.linear.x = joy_wrap_msg_.getMsg().axes[4]*0.25;
					space_prev_.linear.y = joy_wrap_msg_.getMsg().axes[3]*0.25;
					space_prev_.linear.z = joy_wrap_msg_.getMsg().axes[1]*0.25;
					space_prev_.angular.x = (-joy_wrap_msg_.getMsg().buttons[4] + joy_wrap_msg_.getMsg().buttons[5])*2;
					space_prev_.angular.y = joy_wrap_msg_.getMsg().axes[7]*0.9;
					space_prev_.angular.z = joy_wrap_msg_.getMsg().axes[6]*0.9;
					space_msg_.setMsg(space_prev_);
				}
        //TODO replaced hardcoded names "dual_arms" with a more configurable sollution
				if (current_joint_model_->getName() == "dual_arms") {
					dualEndPointControl(current_state, temp_display, planning_scene, space_holder_);
				} else {
					endPointControl_EE(current_state, temp_display, planning_scene);
				}

				break;
			}
			default:
				break;
			}  //switch (mode_)

			// create displayrobot model for debug in rviz
			display_pub_.publish(temp_display);
			rate.sleep();
		  //while (ros::ok())
	}
}

/*----------------------------------------------------------------------------
  switch mode
 *----------------------------------------------------------------------------*/
JoystickControl::Mode JoystickControl::switchMode(const Mode& new_mode)
{
  //ROS_INFO("JoystickControl::switchMode from %i to %i",mode_,new_mode);
  
  // get list of trajectory controllers and if they are running
  // assuming that all trajectory controllers get turned off and joint controllers turned on for MODE_JOINT_POSITION and MODE_ENDPOINT
  // asuuming that all trajectory controllers get turned on and joint controllers turned off for all other modes
  controller_manager_msgs::ListControllers list_srv;
  ros::service::waitForService("/controller_manager/list_controllers");
  ros::ServiceClient list_controller = nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers", true);
  list_controller.call(list_srv);
  ROS_INFO("found %d controllers in list",list_srv.response.controller.size());
  
  std::vector<std::string> trajectory_controllers;
  bool trajectory_controllers_running = true;
  std::vector<std::string> joint_position_controllers;
  bool joint_position_controllers_running = true;
  std::vector<std::string> joint_velocity_controllers;
  bool joint_velocity_controllers_running = true;
  for(int i=0; i<list_srv.response.controller.size(); ++i){
    //ROS_INFO("JoystickControl::Mode controller %s of type %s is %s",list_srv.response.controller[i].name.c_str(),list_srv.response.controller[i].type.c_str(), list_srv.response.controller[i].state.c_str());
    if(list_srv.response.controller[i].type=="position_controllers/JointTrajectoryController"){
      trajectory_controllers.push_back(list_srv.response.controller[i].name);
      //ROS_INFO("JoystickControl::Mode found trajectory controller %s",list_srv.response.controller[i].name.c_str());
      if(list_srv.response.controller[i].state != "running"){
        trajectory_controllers_running = false;
      }
    }
    if(list_srv.response.controller[i].type=="position_controllers/JointPositionController"){
      joint_position_controllers.push_back(list_srv.response.controller[i].name);
      //ROS_INFO("JoystickControl::Mode found trajectory controller %s",list_srv.response.controller[i].name.c_str());
      if(list_srv.response.controller[i].state != "running"){
        joint_position_controllers_running = false;
      }
    }
    if(list_srv.response.controller[i].type=="velocity_controllers/JointVelocityController"){
      joint_velocity_controllers.push_back(list_srv.response.controller[i].name);
      //ROS_INFO("JoystickControl::Mode found trajectory controller %s",list_srv.response.controller[i].name.c_str());
      if(list_srv.response.controller[i].state != "running"){
        joint_velocity_controllers_running = false;
      }
    }
  }
  
  //ROS_INFO("found: trajectory controllers %d, joint controllers %d",trajectory_controllers_running,joint_controllers_running);
  // create flag for changing controllers, assume no change needed and then check
  bool change_controllers = false;
  controller_manager_msgs::SwitchController srv;
  srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  //srv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
  
  switch(new_mode){
    case MODE_NONE:
    case MODE_RESET:
      // reset is place holder mode to use while waiting for connection to arm to reestablish
      // do nothing
      break;
    case MODE_NAMED:
      // named pose execution uses moveit and the joint trajectory controller
      // check which controllers are running and update
      if(!trajectory_controllers_running){
        for(int i=0; i<trajectory_controllers.size(); ++i){
          srv.request.start_controllers.push_back(trajectory_controllers[i]);
        }
        change_controllers = true;
      }
      if(joint_position_controllers_running){
        for(int i=0; i<joint_position_controllers.size(); ++i){
          srv.request.stop_controllers.push_back(joint_position_controllers[i]);
        }
        change_controllers = true;
      }
      if(joint_velocity_controllers_running){
        for(int i=0; i<joint_velocity_controllers.size(); ++i){
          srv.request.stop_controllers.push_back(joint_velocity_controllers[i]);
        }
        change_controllers = true;
      }
      break;
    case MODE_JOINT_VELOCITY:
      // joint velocity control uses the velocity controllers
      // check which controllers are running and update
      if(trajectory_controllers_running){
        for(int i=0; i<trajectory_controllers.size(); ++i){
          srv.request.stop_controllers.push_back(trajectory_controllers[i]);
        }
        change_controllers = true;
      }
      if(joint_position_controllers_running){
        for(int i=0; i<joint_position_controllers.size(); ++i){
          srv.request.stop_controllers.push_back(joint_position_controllers[i]);
        }
        change_controllers = true;
      }
      if(!joint_velocity_controllers_running){
        for(int i=0; i<joint_velocity_controllers.size(); ++i){
          srv.request.start_controllers.push_back(joint_velocity_controllers[i]);
        }
        change_controllers = true;
      }
      break;
    case MODE_JOINT_POSITION:
      // joint position control uses the position conttrollers
    case MODE_ENDPOINT:
      // end point control uses the position conttrollers
    case MODE_ENDPOINT_EE:
      // end point control in end effector frame uses the position conttrollers
      // get current end point pose
      endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose(); //TODO Only place outisde of ee_control functions is here. Could transform into parameter of ee_control functions
      
      // populate joint positions from joint states instead of through moveit to avoid issues with mimic joints
      joint_positions_.resize(joint_names_.size());
      joint_commands.resize(joint_names_.size());
      std::unique_lock<std::mutex> lock2(joint_state_mutex_);
      for (size_t i = 0; i < joint_names_.size(); i++){
        std::vector<std::string>::iterator it = std::find(joint_state_last_.name.begin(),joint_state_last_.name.end(),joint_names_[i]);
        if(it != joint_state_last_.name.end()){
          joint_positions_[i] = joint_state_last_.position[it-joint_state_last_.name.begin()];
        }else{
          ROS_WARN("joint %s not found in joint states",joint_names_[i].c_str());
        }
      }
      // initialize commanded position as current position
      joint_commands = joint_positions_;
      
      
      EE_positions.resize(EE_names_.size());
      EE_commands.resize(EE_names_.size());
      if(end_effector_map_.count(current_joint_model_->getName()) != 0){
        for (size_t i = 0; i < EE_names_.size(); i++){
          std::vector<std::string>::iterator it = std::find(joint_state_last_.name.begin(),joint_state_last_.name.end(),EE_names_[i]);
          if(it != joint_state_last_.name.end()){
            EE_positions[i] = joint_state_last_.position[it-joint_state_last_.name.begin()];
          }else{
            ROS_WARN("joint %s not found in joint states",EE_names_[i].c_str());
          }
        }
      }
      EE_commands=EE_positions;
      lock2.unlock();
      // check which controllers are running and update
      if(trajectory_controllers_running){
        for(int i=0; i<trajectory_controllers.size(); ++i){
          srv.request.stop_controllers.push_back(trajectory_controllers[i]);
        }
        change_controllers = true;
      }
      if(!joint_position_controllers_running){
        for(int i=0; i<joint_position_controllers.size(); ++i){
          srv.request.start_controllers.push_back(joint_position_controllers[i]);
        }
        change_controllers = true;
      }
      if(joint_velocity_controllers_running){
        for(int i=0; i<joint_velocity_controllers.size(); ++i){
          srv.request.stop_controllers.push_back(joint_velocity_controllers[i]);
        }
        change_controllers = true;
      }
      break;
  }
  
 
  if(change_controllers){
    ros::service::waitForService("/controller_manager/switch_controller");
    ros::ServiceClient switch_controller = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller", true);
    
    if (switch_controller.call(srv))
    {
      if (srv.response.ok)
      {
        std::string temp_string = "switching from mode " + Mode_Names_[mode_] + " to " + Mode_Names_[new_mode];
       
        /*
        // debug code for watching which controllers are running at end of mode change
        list_controller.call(list_srv);
        //ROS_INFO("found %d controllers in list",list_srv.response.controller.size());
        //trajectory_controllers.clear();
        trajectory_controllers_running = true;
        //joint_controllers.clear();
        joint_controllers_running = true;
        for(int i=0; i<list_srv.response.controller.size(); ++i){
          if(list_srv.response.controller[i].type=="position_controllers/JointTrajectoryController"){
            //trajectory_controllers.push_back(list_srv.response.controller[i].name);
            //ROS_INFO("JoystickControl::Mode found trajectory controller %s",list_srv.response.controller[i].name.c_str());
            if(list_srv.response.controller[i].state != "running"){
              trajectory_controllers_running = false;
            }
          }
          if(list_srv.response.controller[i].type=="position_controllers/JointPositionController"){
            //joint_controllers.push_back(list_srv.response.controller[i].name);
            //ROS_INFO("JoystickControl::Mode found trajectory controller %s",list_srv.response.controller[i].name.c_str());
            if(list_srv.response.controller[i].state != "running"){
              joint_controllers_running = false;
            }
          }
        }
        
        ROS_INFO("left: trajectory controllers %d, joint controllers %d",trajectory_controllers_running,joint_controllers_running);
        */
        
        ROS_INFO("%s", temp_string.c_str());
        return new_mode;
      }
      else
      {
        ROS_ERROR("service call failed");
      }
    }
    return mode_;
  }else{
    std::string temp_string = "switching from mode " + Mode_Names_[mode_] + " to " + Mode_Names_[new_mode] + "(no controller change)";
    ROS_INFO("%s", temp_string.c_str());
    return new_mode;
  
  }
  
}

/*----------------------------------------------------------------------------
  joy callback
 *----------------------------------------------------------------------------*/
void JoystickControl::joyCallback(const sensor_msgs::Joy& msg)
{

  //ROS_INFO("JoystickControl::joyCallback");
  
  std::unique_lock<std::mutex> lock(mutex_);
  joy_msg_ = msg;
  if(joy_msg_.buttons.size() != joy_wrap_msg_.getMsg().buttons.size()){
    joy_wrap_msg_.setMsg(joy_msg_);
  }
}

/*----------------------------------------------------------------------------
  space mouse callback
 *----------------------------------------------------------------------------*/
void JoystickControl::spaceCallback(const geometry_msgs::Twist& msg)
{

  std::unique_lock<std::mutex> lock(mutex_);
  space_holder_ = msg;

}
/*----------------------------------------------------------------------------
  joint state callback
 *----------------------------------------------------------------------------*/
void JoystickControl::jointstateCallback(const sensor_msgs::JointState& msg)
{
  std::unique_lock<std::mutex> lock(joint_state_mutex_);
  joint_state_last_ = msg;
  
}
/*----------------------------------------------------------------------------
  manipulator connection status callback
 *----------------------------------------------------------------------------*/
void JoystickControl::connectionCallback(const std_msgs::Bool& msg)
{
  manipulator_connected = msg.data;
  //ROS_INFO("JoystickControl::connectionCallback got %d and put it in %d",msg.data,manipulator_connected);
  
}

void JoystickControl::updateCollision(robot_state::RobotState &current_state) {
	// update positions of all the drives in the "current_state" RobotState so that collision avoidance will work properly
	
  //ROS_INFO("JoystickControl::updateCollision");
  
  std::lock_guard<std::mutex> lock2(joint_state_mutex_);
	for (std::size_t it = 0; it < joint_model_groups_.size(); ++it) {
    //ROS_INFO("\tjoint model group %s",joint_model_groups_[it]->getName().c_str());
		//moveit::core::RobotStatePtr robot_state = move_groups_[it]->getCurrentState();
		std::vector<double> temp_joint_values;
		/*
        ROS_INFO("\t\tjoint %s at %f",temp_joint_names[i].c_str(),temp_joint_values[i]);
		// this method creates problem with mimic joints, try populating position from the joint state message
		robot_state->copyJointGroupPositions(joint_model_groups_[it], temp_joint_values);
		current_state.setJointGroupPositions(joint_model_groups_[it], temp_joint_values);
		*/
		
	  std::vector<std::string> temp_joint_names = joint_model_groups_[it]->getActiveJointModelNames();
		temp_joint_values.resize(temp_joint_names.size());
    for (size_t i = 0; i < temp_joint_names.size(); i++){
      std::vector<std::string>::iterator j = std::find(joint_state_last_.name.begin(),joint_state_last_.name.end(),temp_joint_names[i]);
      if(j != joint_state_last_.name.end()){
        temp_joint_values[i] = joint_state_last_.position[j-joint_state_last_.name.begin()];
        //ROS_INFO("\t\tjoint %s at %f",temp_joint_names[i].c_str(),temp_joint_positions[i]);
      }else{
        ROS_WARN("joint %s not found in joint states",joint_names_[i].c_str());
      }
    }
		current_state.setJointGroupPositions(joint_model_groups_[it], temp_joint_values);
		
	}
	
	// TODO update end effectors, include mimic joints
	
	/*
	for (size_t i = 0; i < joint_state_last_.name.size(); i++){
	  ROS_INFO("joint_state %s at %f",joint_state_last_.name[i].c_str(),joint_state_last_.position[i]);
	}
	*/
}

bool JoystickControl::updateJointState(){
	if(joint_state_.repeat()){
		return false;
	}
	else{
		joint_state_.setMsg(joint_state_last_);
	}
}


// Currently this both operates on the globally set joint_commands_, but it also returns the value of joint_commands_
// Going forward we would like to decouple global state, which is why a return statement was added.
std::vector<double> JoystickControl::processControlInputInternal(const moveit::core::JointModelGroup* joint_model_group){ //TODO take in

	joystick_map_.parseJointModel(joint_model_group, joy_wrap_msg_.getMsg(), joint_commands);
	processControlInputInternalEE();
	// Ensure Joint Limits are checked and obeyed
	joint_model_group->enforcePositionBounds(joint_commands.data());
	return joint_commands;

}

//TODO make this take in values like non-EE version
std::vector<double> JoystickControl::processControlInputInternalEE(){

	float joy_input;
	size_t i;
	std::string ee_name;
	JoyMapEntry cur_joy_map;

    for (size_t i = 0; i < EE_names_.size(); i++, joy_input=0){
      ee_name = EE_names_[i];
      double& ee_command = EE_commands[i];
      cur_joy_map = joystick_map_[ee_name];

      if(cur_joy_map.input==AXES){
        // axis A is left trigger which opens.
              if(cur_joy_map.axisB>=0){
                static bool b_triggered = false;
                static bool a_triggered = false;
                if(a_triggered and b_triggered){
                  joy_input = (joy_wrap_msg_.getMsg().axes[cur_joy_map.axisA]-joy_wrap_msg_.getMsg().axes[cur_joy_map.axisB])*JOINT_SCALE/2;
                }
                else if((!b_triggered and joy_wrap_msg_.getMsg().axes[cur_joy_map.axisB] != 0.0 ) or (b_triggered and !a_triggered and joy_wrap_msg_.getMsg().axes[cur_joy_map.axisB] != 1.0)){
                  joy_input = (1-joy_wrap_msg_.getMsg().axes[cur_joy_map.axisB])*JOINT_SCALE/2;
                  b_triggered = true;
                }
                else if((!a_triggered or (a_triggered && !b_triggered)) and joy_wrap_msg_.getMsg().axes[cur_joy_map.axisA] != 0.0){
                  joy_input = (joy_wrap_msg_.getMsg().axes[cur_joy_map.axisA]-1)*JOINT_SCALE/2;
                  a_triggered = true;
                }
              }
      }else if(cur_joy_map.input==BUTTONS){
        if(cur_joy_map.axisB>=0){
          joy_input = (joy_wrap_msg_.getMsg().buttons[cur_joy_map.axisA]-joy_wrap_msg_.getMsg().buttons[cur_joy_map.axisB])*JOINT_SCALE;
        }else{
          joy_input = joy_wrap_msg_.getMsg().buttons[cur_joy_map.axisA]*JOINT_SCALE;
        }
      }else{
        //TODO move this warning outside the loop, probably into the init fuction
        ROS_WARN("joystick map for %s input %i, not recognized",ee_name.c_str(),joystick_map_[ee_name].input);
      }
      if(cur_joy_map.invert){
        joy_input = -joy_input;
      }
      ee_command += joy_input;

      // manually check and enforce position limits on a drive-by-drive basis

      // manually check and enforce position limits on a drive-by-drive basis
      if(ee_command > joint_bounds_[ee_name].max_position_){
        ee_command = joint_bounds_[ee_name].max_position_;
        ROS_WARN("%s at + position limit %f",ee_name.c_str(),joint_bounds_[ee_name].max_position_);
      }
      if(ee_command < joint_bounds_[ee_name].min_position_){
        ee_command = joint_bounds_[ee_name].min_position_;
        ROS_WARN("%s at - position limit %f",ee_name.c_str(),joint_bounds_[ee_name].min_position_);
      }

    }
    return EE_commands;

}

void JoystickControl::jointByJointControlVelocity(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene){

    //ROS_INFO("JoystickControl:jointByJointControlVelocity");

	
  std::vector<double> joint_velocities = joystick_map_.parseJointModel(current_joint_model_, joy_wrap_msg_.getMsg());
  for (size_t i = 0; i < joint_velocities.size(); i++){
    joint_velocities[i] *= LOOP_RATE;
  }
  
  publishVelocityCommands(joint_names_, joint_velocities);


}
void JoystickControl::jointByJointControlPosition(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene){

    //color_current = color_joint_position_; // Should probably replace calls like this with a set_color
    //ROS_INFO("JoystickControl::run MODE_JOINT_POSITION");

    joint_commands = processControlInputInternal(current_joint_model_);


    //TODO add check if commanded position and telemetry position and too far apart

    // check for self-collisions
    //collision_request.group_name = cur_joint_model_->getName();
    collision_result.clear();
    current_state.setJointGroupPositions(current_joint_model_,joint_commands);
    if(end_effector_map_.count(current_joint_model_->getName()) != 0){

      // need to check and populate mimic drives because moveit doesn't handle it automatically
      std::vector<double> temp_EE_commands;
      std::vector<std::string> temp_names = EE_group->getJointModelNames();
      for (size_t i = 0; i < temp_names.size(); i++){
        const moveit::core::JointModel* temp_joint;
        const moveit::core::JointModel* temp_jointB;
        std::vector<std::string>::iterator it;
        temp_joint = EE_group->getJointModel(temp_names[i]);
        temp_jointB = temp_joint->getMimic(); //Could be cause of issue, pointer is not checked here before it is dereferenced.
        if(temp_jointB != NULL){
          //ROS_INFO("\t%s mimics %s",temp_joint->getName().c_str(),temp_jointB->getName().c_str());
          it = std::find(EE_names_.begin(),EE_names_.end(),temp_jointB->getName());
          temp_EE_commands.push_back(EE_commands[it-EE_names_.begin()]*temp_jointB->getMimicFactor()+temp_jointB->getMimicOffset());
        }else{
          //ROS_INFO("\t%s is a regular joint",temp_joint->getName().c_str());
          it = std::find(EE_names_.begin(),EE_names_.end(),temp_joint->getName());
          temp_EE_commands.push_back(EE_commands[it-EE_names_.begin()]);
        }
      }
      current_state.setJointGroupPositions(EE_group,temp_EE_commands);

    }
    planning_scene.checkSelfCollision(collision_request,collision_result);

    if(collision_result.collision){
      ROS_WARN("collision found:");
      std::vector<std::string> colliding_links;
      planning_scene.getCollidingLinks(colliding_links,current_state);
      moveit_msgs::ObjectColor temp_object;
      temp_object.color = joystick_display_colors[MODE_RESET];
      for(int i=0; i<colliding_links.size(); ++i){
        ROS_WARN("\t%s",colliding_links[i].c_str());
        // check if this link is already in the highlight list
        bool link_found = false;
        for(int j=0; j<temp_display.highlight_links.size(); j++){
          if(temp_display.highlight_links[j].id == colliding_links[i]){
            temp_display.highlight_links[j].color = joystick_display_colors[MODE_RESET];
            link_found = true;
          }
        }
        if(!link_found){
          temp_object.id = colliding_links[i];
          temp_display.highlight_links.push_back(temp_object);
        }
      }
    }
    if(collision_result.collision and (!collision_enable)){
        // populate joint positions from joint states instead of through moveit to avoid issues with mimic joints

        std::unique_lock<std::mutex> lock2(joint_state_mutex_); //Swapped this to lock_guard
        for (size_t i = 0; i < joint_names_.size(); i++){
          std::vector<std::string>::iterator it = std::find(joint_state_last_.name.begin(),joint_state_last_.name.end(),joint_names_[i]);
          if(it != joint_state_last_.name.end()){
            joint_commands[i] = joint_state_last_.position[it-joint_state_last_.name.begin()];
          }else{
            ROS_WARN("joint %s not found in joint states",joint_names_[i].c_str());
          }
        }
        
        if(end_effector_map_.count(current_joint_model_->getName()) != 0){
          for (size_t i = 0; i < EE_names_.size(); i++){
            std::vector<std::string>::iterator it = std::find(joint_state_last_.name.begin(),joint_state_last_.name.end(),EE_names_[i]);
            if(it != joint_state_last_.name.end()){
              EE_commands[i] = joint_state_last_.position[it-joint_state_last_.name.begin()];
            }else{
              ROS_WARN("joint %s not found in joint states",EE_names_[i].c_str());
            }
          }
          // Set EE jointGroupPositions inside this if, as not all movesets have end effectors
          current_state.setJointGroupPositions(EE_group,EE_commands);
        }
        lock2.unlock();
        current_state.setJointGroupPositions(current_joint_model_,joint_commands);

    }else{
    	publishPositionCommands(joint_names_, joint_commands); // TODO evaluate if this should only be called in else case
        publishPositionCommands(EE_names_, EE_commands);
    }

}


void JoystickControl::publishPositionCommands(std::vector<std::string>& name_vector,  std::vector<double>& command_vector){
	for (int i = 0; i < name_vector.size(); i++) {
		//ROS_INFO("\t%s at %f moving to %f",joint_names_[i].c_str(),joint_positions[i],joint_commands[i]);
		std_msgs::Float64 command;
		command.data = command_vector[i];
		joint_position_pubs_[name_vector[i]].publish(command);
	}
}

void JoystickControl::publishVelocityCommands(std::vector<std::string>& name_vector,  std::vector<double>& command_vector){
	for (int i = 0; i < name_vector.size(); i++) {
		//ROS_INFO("\t%s at %f moving to %f",joint_names_[i].c_str(),joint_positions[i],joint_commands[i]);
		std_msgs::Float64 command;
		command.data = command_vector[i];
		joint_velocity_pubs_[name_vector[i]].publish(command);
	}
}
void JoystickControl::endPointControl(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene){
	//color_current = joystick_display_colors[MODE_ENDPOINT];
	        // TODO do we want to support joystick (xbox or ds4) based end point control?


	        // handle end effector control, if present

			processControlInputInternalEE();
/*
			if (space_msg_.repeat() && space_msg_.zero()) {
				//endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
				publishPositionCommands(joint_names_, joint_commands);
				publishPositionCommands(EE_names_, EE_commands);
				return;
				}
*/
	        // convert the pose to a transform
	        // apply the space mouse input on top of that transform <- the TF2 library makes this easy with quaternion multiplication
	        //convert back to pose and pass to moveit
	        
          geometry_msgs::PoseStamped endpoint_pose_last = endpoint_pose_;
          
          if (!(space_msg_.repeat() && space_msg_.zero())) {
	          //ROS_INFO("current endpoint pose in frame %s at [%2.3f,%2.3f,%2.3f]",endpoint_pose_.header.frame_id.c_str(),endpoint_pose_.pose.position.x,endpoint_pose_.pose.position.y,endpoint_pose_.pose.position.z);
	          tf2::Transform endpoint_tf;
	          tf2::fromMsg(endpoint_pose_.pose, endpoint_tf);
	          //tf2::Vector3 temp_origin = endpoint_tf.getOrigin();
	          //ROS_INFO("endpoint_tf before [%2.3f,%2.3f,%2.3f]",temp_origin.x(),temp_origin.y(),temp_origin.z());

	          //***************************************************************************
	          // to debug user input publish a transform showing an exagerated transform
	          static tf2_ros::TransformBroadcaster br;
	          tf2::Quaternion debug_q;
	          debug_q.setRPY(space_msg_.getMsg().angular.x*ANGULAR_SCALE/LOOP_RATE*3,
	                          space_msg_.getMsg().angular.y*ANGULAR_SCALE/LOOP_RATE*3,
	                          space_msg_.getMsg().angular.z*ANGULAR_SCALE/LOOP_RATE*3);
	          geometry_msgs::TransformStamped debug_tf;
	          debug_tf.header=endpoint_pose_.header;
	          //debug_tf.header.frame_id = move_groups_[joint_group_index]->getEndEffectorLink();
	          debug_tf.header.stamp=ros::Time::now();
	          debug_tf.child_frame_id="target_endpoint";
	          debug_tf.transform.translation.x=endpoint_pose_.pose.position.x + space_msg_.getMsg().linear.x*LINEAR_SCALE/LOOP_RATE*3;
	          debug_tf.transform.translation.y=endpoint_pose_.pose.position.y + space_msg_.getMsg().linear.y*LINEAR_SCALE/LOOP_RATE*3;
	          debug_tf.transform.translation.z=endpoint_pose_.pose.position.z + space_msg_.getMsg().linear.z*LINEAR_SCALE/LOOP_RATE*3;
	          //debug_tf.transform.translation.x=space_prev_.linear.x;
	          //debug_tf.transform.translation.y=space_prev_.linear.y;
	          //debug_tf.transform.translation.z=space_prev_.linear.z;
	          debug_q=debug_q*endpoint_tf.getRotation();
	          tf2::convert(debug_q,debug_tf.transform.rotation);
	          br.sendTransform(debug_tf);
	          //***************************************************************************

	          tf2::Quaternion q;
	          // get current location of transform origin and add linear component of joystick message
	          endpoint_tf.setOrigin(endpoint_tf.getOrigin()+tf2::Vector3(
	                        space_msg_.getMsg().linear.x*LINEAR_SCALE/LOOP_RATE,
	                        space_msg_.getMsg().linear.y*LINEAR_SCALE/LOOP_RATE,
	                        space_msg_.getMsg().linear.z*LINEAR_SCALE/LOOP_RATE));
	          // build quaternion from angular component of joystick message using roll-pitch-yaw (RPY) format
	          q.setRPY(space_msg_.getMsg().angular.x*ANGULAR_SCALE/LOOP_RATE,
	                    space_msg_.getMsg().angular.y*ANGULAR_SCALE/LOOP_RATE,
	                    space_msg_.getMsg().angular.z*ANGULAR_SCALE/LOOP_RATE);
	          // multiply quaternion from joystick by existing pose quaternion and set as new pose quaternion
	          endpoint_tf.setRotation(q*endpoint_tf.getRotation());
	          //temp_origin = endpoint_tf.getOrigin();
	          //ROS_INFO("endpoint_tf after  [%2.3f,%2.3f,%2.3f]",temp_origin.x(),temp_origin.y(),temp_origin.z());

	          // convert transfrom back into pose format to send to moveit
	          tf2::toMsg(endpoint_tf, endpoint_pose_.pose);
	        }
	        
	        //get current joint positions, for checking if IK solution is too far away
	        // TODO consider populating joint data from joint_states message, might be faster
	        moveit::core::RobotStatePtr robot_state = move_groups_[joint_group_index]->getCurrentState(wait_time);
	        bool found_ik;
	        if(robot_state == NULL){
	          ROS_WARN("JoystickControl::endPointControl robot state pointer is null");
	          found_ik = false;
	        }else{
	          //ROS_INFO("JoystickControl::endPointControl robot state appears valid");
	          robot_state->copyJointGroupPositions(current_joint_model_, joint_commands);
            found_ik = robot_state->setFromIK(current_joint_model_, endpoint_pose_.pose);
	        }
          //TODO:Using the approximate KinematicsQueryOptions 'return_approximate_solutions' method may allow us to control under-actuated systems better.  Other options include using the setApproimateJointValueTarget which uses the MoveitPlanningInterface rather than this robot_state->setFromIK. We could also try using IKFAST or TRACKIK rather than KDL, since KDL is really only for >6 DOF.  The easiest KDL hack is to place this line in your kinematics.yaml "position_only_ik: true"  The drawback is that we cannot control wrist roll.  Approximate solutions would be better for us.
          //bool found_ik = robot_state->setFromIK(joint_model_groups_[joint_model_index], endpoint_pose_.pose,1,3.0,is_valid,approx_variable);
          //ROS_INFO("MODE_ENDPOINT - IK returned %d",found_ik);

          //bool found_ik = robot_state->setFromDiffIK(joint_model_group_, twist, ENDPOINT_LINK, 1.0/LOOP_RATE);
          //const kinematics::KinematicsQueryOptions approx_variable;
          //approx_variable.return_approximate_solution();

	        


	        if (found_ik)
	        {
	          //ROS_INFO("target endpoint pose in frame %s at [%2.3f,%2.3f,%2.3f]",endpoint_pose_.header.frame_id.c_str(),endpoint_pose_.pose.position.x,endpoint_pose_.pose.position.y,endpoint_pose_.pose.position.z);
	          // get new joint values
	          std::vector<double> joint_values;
	          robot_state->copyJointGroupPositions(current_joint_model_, joint_values);
	          double scale_down_value = 1.0;
	          // check for jumps that are higher than max velocity.
	          // scale all joint values if one joint has too much divergence.
	          bool valid_ik = true;
	          num_joints_ = joint_names_.size();

	          for (size_t i = 0; i < num_joints_; i++)
	          {
	            //ROS_INFO("\t%s at %2.3f moving to %2.3f",joint_names_[i].c_str(),joint_commands[i],joint_values[i]);
	            //ROS_INFO("\t%s moving %2.3f",joint_names_[i].c_str(),joint_values[i] - joint_commands[i]);
	            scale_down_value = fmin(scale_down_value,(MAX_VEL/LOOP_RATE)/abs(joint_values[i] - joint_commands[i]));
	          }

	          if (scale_down_value  < 0.05)  // Give up on unreasonable end point commands that make the arm flail.
	          {
	            ROS_WARN("unreasonable scale_down_value = %f",scale_down_value);
	            valid_ik = false;

	            // go back to current pose
	            //endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
	            
	            // ignore transfromation from this control loop
	            endpoint_pose_ = endpoint_pose_last;
	            return;
	          }
	          if(scale_down_value<1.0){
	            //ROS_WARN("scale_down_value = %f",scale_down_value);
	            for (size_t i = 0; i < num_joints_; i++)
	            {
	          		joint_values[i] = (joint_commands[i]-joint_values[i])*scale_down_value + joint_values[i];
	            }
	          }

	            // check for self-collisions

	            //collision_request.group_name = cur_joint_model_->getName();
	            collision_result.clear();
	            current_state.setJointGroupPositions(current_joint_model_,joint_values);
	            if(end_effector_map_.count(current_joint_model_->getName()) != 0){
	              // need to check and populate mimic drives because moveit doesn't handle it automatically
	              std::vector<double> temp_EE_commands;
	              std::vector<std::string> temp_names = EE_group->getJointModelNames();
	              for (size_t i = 0; i < temp_names.size(); i++){
	                const moveit::core::JointModel* temp_joint; // If this could be null the next line could be trouble
	                const moveit::core::JointModel* temp_jointB;
	                std::vector<std::string>::iterator it;
	                temp_joint = EE_group->getJointModel(temp_names[i]);
	                temp_jointB = temp_joint->getMimic(); //Could be cause of issue, pointer is not checked here before it is dereferenced.
	                if(temp_jointB != NULL){
	                  //ROS_INFO("\t%s mimics %s",temp_joint->getName().c_str(),temp_jointB->getName().c_str());
	                  it = std::find(EE_names_.begin(),EE_names_.end(),temp_jointB->getName());
	                  temp_EE_commands.push_back(EE_commands[it-EE_names_.begin()]*temp_jointB->getMimicFactor()+temp_jointB->getMimicOffset());
	                }else{
	                  //ROS_INFO("\t%s is a regular joint",temp_joint->getName().c_str());
	                  it = std::find(EE_names_.begin(),EE_names_.end(),temp_joint->getName());
	                  temp_EE_commands.push_back(EE_commands[it-EE_names_.begin()]);
	                }
	              }
	              current_state.setJointGroupPositions(EE_group,temp_EE_commands);

	            }
	            planning_scene.checkSelfCollision(collision_request,collision_result);
              if(collision_result.collision){
                ROS_WARN("collision found:");
                std::vector<std::string> colliding_links;
                planning_scene.getCollidingLinks(colliding_links,current_state);
                moveit_msgs::ObjectColor temp_object;
                temp_object.color = joystick_display_colors[MODE_RESET];
                for(int i=0; i<colliding_links.size(); ++i){
                  ROS_WARN("\t%s",colliding_links[i].c_str());
                  // check if this link is already in the highlight list
                  bool link_found = false;
                  for(int j=0; j<temp_display.highlight_links.size(); j++){
                    if(temp_display.highlight_links[j].id == colliding_links[i]){
                      temp_display.highlight_links[j].color = joystick_display_colors[MODE_RESET];
                      link_found = true;
                    }
                  }
                  if(!link_found){
                    temp_object.id = colliding_links[i];
                    temp_display.highlight_links.push_back(temp_object);
                  }
                }
    
	              // go back to current pose
	              //endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
	              
	              // ignore transfromation from this control loop
	              endpoint_pose_ = endpoint_pose_last;
	            }else if(!current_state.satisfiesBounds(current_joint_model_)){
	              ROS_WARN("outside position limits");
	              // go back to current pose
	              //endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
	              
	              // ignore transfromation from this control loop
	              endpoint_pose_ = endpoint_pose_last;
	            }else{
	            	publishPositionCommands(joint_names_, joint_values);
	            	publishPositionCommands(EE_names_, EE_commands);
	            }
	          //}
	        }
	        else
	        {
	          ROS_WARN("no ik solution found");
	          // go back to current pose
	          //endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
	          
            // ignore transfromation from this control loop
            endpoint_pose_ = endpoint_pose_last;
	        }
}

void JoystickControl::endPointControl_EE(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene){
  //color_current = joystick_display_colors[MODE_ENDPOINT];
  // TODO do we want to support joystick (xbox or ds4) based end point control?


  // handle end effector control, if present

  processControlInputInternalEE();


  geometry_msgs::PoseStamped endpoint_pose_last = endpoint_pose_;

  if (!(space_msg_.repeat() && space_msg_.zero())) {
    //ROS_INFO("current endpoint pose in frame %s at [%2.3f,%2.3f,%2.3f]",endpoint_pose_.header.frame_id.c_str(),endpoint_pose_.pose.position.x,endpoint_pose_.pose.position.y,endpoint_pose_.pose.position.z);

    // convert current end point command to transfrom for moving in pincer coordinate frame
    geometry_msgs::TransformStamped current_endpoint_tf;
    //current_endpoint_tf.child_frame_id = ""; // still works if this is blank
    current_endpoint_tf.transform.translation.x = endpoint_pose_.pose.position.x;
    current_endpoint_tf.transform.translation.y = endpoint_pose_.pose.position.y;
    current_endpoint_tf.transform.translation.z = endpoint_pose_.pose.position.z;
    current_endpoint_tf.transform.rotation = endpoint_pose_.pose.orientation;

    // create a stamped pose representing the linear and rotation commands from the spacemouse
    geometry_msgs::PoseStamped pose_in;
    pose_in.pose.position.x = space_msg_.getMsg().linear.x*LINEAR_SCALE/LOOP_RATE;
    pose_in.pose.position.y = space_msg_.getMsg().linear.y*LINEAR_SCALE/LOOP_RATE;
    pose_in.pose.position.z = space_msg_.getMsg().linear.z*LINEAR_SCALE/LOOP_RATE;
    tf2::Quaternion q;
    // build quaternion from angular component of joystick message using roll-pitch-yaw (RPY) format
    q.setRPY(space_msg_.getMsg().angular.x*ANGULAR_SCALE/LOOP_RATE,
             space_msg_.getMsg().angular.y*ANGULAR_SCALE/LOOP_RATE,
             space_msg_.getMsg().angular.z*ANGULAR_SCALE/LOOP_RATE);
    // convert between different quaternion data types
    tf2::convert(q,pose_in.pose.orientation);
    // could set frame ID's but it still works if they're blank
    //pose_stamped.header.frame_id = move_groups_[joint_model_index]->getEndEffectorLink();
    pose_in.header.stamp = ros::Time::now();

    // create stamped pose to represent end point pose + space mouse commands
    geometry_msgs::PoseStamped pose_out;
    // use current end point pose command instead of data from drive telemetry because it runs smoother
    tf2::doTransform(pose_in,pose_out,current_endpoint_tf);

    // debug print statements (optional)
    //ROS_INFO("pose_in frame %s at [%2.3f,%2.3f,%2.3f]",pose_in.header.frame_id.c_str(),pose_in.pose.position.x,pose_in.pose.position.y,pose_in.pose.position.z);
    //ROS_INFO("pose_out frame %s at [%2.3f,%2.3f,%2.3f]",pose_out.header.frame_id.c_str(),pose_out.pose.position.x,pose_out.pose.position.y,pose_out.pose.position.z);



    //***************************************************************************
    // to debug user input publish a transform showing an exagerated transform
    static tf2_ros::TransformBroadcaster br;
    // create a stamped pose representing the linear and rotation commands from the spacemouse
    geometry_msgs::PoseStamped debug_pose_in;
    pose_in.pose.position.x = space_msg_.getMsg().linear.x*LINEAR_SCALE/LOOP_RATE*3;
    pose_in.pose.position.y = space_msg_.getMsg().linear.y*LINEAR_SCALE/LOOP_RATE*3;
    pose_in.pose.position.z = space_msg_.getMsg().linear.z*LINEAR_SCALE/LOOP_RATE*3;
    // build quaternion from angular component of joystick message using roll-pitch-yaw (RPY) format
    tf2::Quaternion debug_q;
    debug_q.setRPY(space_msg_.getMsg().angular.x*ANGULAR_SCALE/LOOP_RATE*3,
                    space_msg_.getMsg().angular.y*ANGULAR_SCALE/LOOP_RATE*3,
                    space_msg_.getMsg().angular.z*ANGULAR_SCALE/LOOP_RATE*3);
    // convert between different quaternion data types
    tf2::convert(debug_q,debug_pose_in.pose.orientation);
    // could set frame ID's but it still works if they're blank
    //pose_stamped.header.frame_id = move_groups_[joint_model_index]->getEndEffectorLink();
    debug_pose_in.header.stamp = ros::Time::now();

    // create stamped pose to represent end point pose + space mouse commands
    geometry_msgs::PoseStamped debug_pose_out;
    // use current end point pose command instead of data from drive telemetry because it runs smoother
    tf2::doTransform(debug_pose_in,debug_pose_out,current_endpoint_tf);
    
    geometry_msgs::TransformStamped debug_tf;
    debug_tf.header=endpoint_pose_.header;
    debug_tf.header.stamp=ros::Time::now();
    debug_tf.child_frame_id="target_endpoint";
    debug_tf.transform.translation.x = debug_pose_out.pose.position.x;
    debug_tf.transform.translation.y = debug_pose_out.pose.position.y;
    debug_tf.transform.translation.z = debug_pose_out.pose.position.z;
    tf2::convert(debug_pose_out.pose.orientation,debug_tf.transform.rotation);
    br.sendTransform(debug_tf);

    //***************************************************************************


    // copy the new pose into the command pose
    // TODO fix these ambiguously underscore suffixed variable names
    endpoint_pose_.pose=pose_out.pose;
  }

  //get current joint positions, for checking if IK solution is too far away
  // TODO consider populating joint data from joint_states message, might be faster
  moveit::core::RobotStatePtr robot_state = move_groups_[joint_group_index]->getCurrentState(wait_time);
  bool found_ik;
  if(robot_state == NULL){
    ROS_WARN("JoystickControl::endPointControl_EE robot state pointer is null");
    found_ik = false;
  }else{
    //ROS_INFO("JoystickControl::endPointControl_EE robot state appears valid");
    robot_state->copyJointGroupPositions(current_joint_model_, joint_commands);
    // get inverse kinematic solution
    found_ik = robot_state->setFromIK(current_joint_model_, endpoint_pose_.pose);
  }


  if (found_ik)
  {
	          //ROS_INFO("target endpoint pose in frame %s at [%2.3f,%2.3f,%2.3f]",endpoint_pose_.header.frame_id.c_str(),endpoint_pose_.pose.position.x,endpoint_pose_.pose.position.y,endpoint_pose_.pose.position.z);
	          // get new joint values
	          std::vector<double> joint_values;
	          robot_state->copyJointGroupPositions(current_joint_model_, joint_values);
	          double scale_down_value = 1.0;
	          // check for jumps that are higher than max velocity.
	          // scale all joint values if one joint has too much divergence.
	          bool valid_ik = true;
	          num_joints_ = joint_names_.size();

	          for (size_t i = 0; i < num_joints_; i++)
	          {
	            //ROS_INFO("\t%s at %2.3f moving to %2.3f",joint_names_[i].c_str(),joint_commands[i],joint_values[i]);
	            //ROS_INFO("\t%s moving %2.3f",joint_names_[i].c_str(),joint_values[i] - joint_commands[i]);
	            scale_down_value = fmin(scale_down_value,(MAX_VEL/LOOP_RATE)/abs(joint_values[i] - joint_commands[i]));
	          }

	          if (scale_down_value  < 0.05)  // Give up on unreasonable end point commands that make the arm flail.
	          {
	            ROS_WARN("unreasonable scale_down_value = %f",scale_down_value);
	            valid_ik = false;

	            // go back to current pose
	            //endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
	            
	            // ignore transfromation from this control loop
	            endpoint_pose_ = endpoint_pose_last;
	            return;
	          }
	          if(scale_down_value<1.0){
	            //ROS_WARN("scale_down_value = %f",scale_down_value);
	            for (size_t i = 0; i < num_joints_; i++)
	            {
	          		joint_values[i] = (joint_commands[i]-joint_values[i])*scale_down_value + joint_values[i];
	            }
	          }

	            // check for self-collisions

	            //collision_request.group_name = cur_joint_model_->getName();
	            collision_result.clear();
	            current_state.setJointGroupPositions(current_joint_model_,joint_values);
	            if(end_effector_map_.count(current_joint_model_->getName()) != 0){
	              // need to check and populate mimic drives because moveit doesn't handle it automatically
	              std::vector<double> temp_EE_commands;
	              std::vector<std::string> temp_names = EE_group->getJointModelNames();
	              for (size_t i = 0; i < temp_names.size(); i++){
	                const moveit::core::JointModel* temp_joint; // If this could be null the next line could be trouble
	                const moveit::core::JointModel* temp_jointB;
	                std::vector<std::string>::iterator it;
	                temp_joint = EE_group->getJointModel(temp_names[i]);
	                temp_jointB = temp_joint->getMimic(); //Could be cause of issue, pointer is not checked here before it is dereferenced.
	                if(temp_jointB != NULL){
	                  //ROS_INFO("\t%s mimics %s",temp_joint->getName().c_str(),temp_jointB->getName().c_str());
	                  it = std::find(EE_names_.begin(),EE_names_.end(),temp_jointB->getName());
	                  temp_EE_commands.push_back(EE_commands[it-EE_names_.begin()]*temp_jointB->getMimicFactor()+temp_jointB->getMimicOffset());
	                }else{
	                  //ROS_INFO("\t%s is a regular joint",temp_joint->getName().c_str());
	                  it = std::find(EE_names_.begin(),EE_names_.end(),temp_joint->getName());
	                  temp_EE_commands.push_back(EE_commands[it-EE_names_.begin()]);
	                }
	              }
	              current_state.setJointGroupPositions(EE_group,temp_EE_commands);

	            }
	            planning_scene.checkSelfCollision(collision_request,collision_result);
              if(collision_result.collision){
                ROS_WARN("collision found:");
                std::vector<std::string> colliding_links;
                planning_scene.getCollidingLinks(colliding_links,current_state);
                moveit_msgs::ObjectColor temp_object;
                temp_object.color = joystick_display_colors[MODE_RESET];
                for(int i=0; i<colliding_links.size(); ++i){
                  ROS_WARN("\t%s",colliding_links[i].c_str());
                  // check if this link is already in the highlight list
                  bool link_found = false;
                  for(int j=0; j<temp_display.highlight_links.size(); j++){
                    if(temp_display.highlight_links[j].id == colliding_links[i]){
                      temp_display.highlight_links[j].color = joystick_display_colors[MODE_RESET];
                      link_found = true;
                    }
                  }
                  if(!link_found){
                    temp_object.id = colliding_links[i];
                    temp_display.highlight_links.push_back(temp_object);
                  }
                }
    
	              // go back to current pose
	              //endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
	              
	              // ignore transfromation from this control loop
	              endpoint_pose_ = endpoint_pose_last;
	            }else if(!current_state.satisfiesBounds(current_joint_model_)){
	              ROS_WARN("outside position limits");
	              // go back to current pose
	              //endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
	              
	              // ignore transfromation from this control loop
	              endpoint_pose_ = endpoint_pose_last;
	            }else{
	            	publishPositionCommands(joint_names_, joint_values);
	            	publishPositionCommands(EE_names_, EE_commands);
	            }
	          //}
	        }
	        else
	        {
	          ROS_WARN("no ik solution found");
	          // go back to current pose
	          //endpoint_pose_ = move_groups_[joint_group_index]->getCurrentPose();
	          
            // ignore transfromation from this control loop
            endpoint_pose_ = endpoint_pose_last;
	        }
}
void JoystickControl::dualEndPointControl(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene,geometry_msgs::Twist input1)
{
	moveit::core::JointModelGroup* original_joint_model = current_joint_model_;

	static std::vector<std::pair<moveit::core::JointModelGroup*, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>> group_list;
	if (group_list.size() == 0) {
		for (moveit::core::JointModelGroup *group : kinematic_model->getJointModelGroups()) {
			if (group->isEndEffector()
					and group->getName().find("dual") == std::string::npos) {
				std::string parent_group_name =
						group->getEndEffectorParentGroup().first;
				moveit::core::JointModelGroup *parent_group =
						kinematic_model->getJointModelGroup(parent_group_name);
				std::shared_ptr<moveit::planning_interface::MoveGroupInterface> parent_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(parent_group_name);
				group_list.push_back(std::pair<moveit::core::JointModelGroup*, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>(parent_group, parent_move_group));
			}
		}
	}

	moveit::core::JointModelGroup* left_group; //currently arbitrary
	moveit::core::JointModelGroup* right_group; //currently arbitrary

	std::string left_group_name;
	std::string right_group_name;


	dualEndPointControlInternal(current_state, temp_display, planning_scene,input1, group_list, original_joint_model);


//	if(current_joint_model_->getSubgroupNames().size() == 2){
//		std::vector<const moveit::core::JointModelGroup*> sub_groups;
//		current_joint_model_->getSubgroups(sub_groups);
//		for(auto subgroup : sub_groups){
//			if(subgroup->getName() == "right_arm"){
//				std::cout<<"RightArmFOund!"<<std::endl;
//				right_group = subgroup;
//
//			}
//			if(subgroup->getName() == "left_arm"){
//							std::cout<<"LeftARmFound!"<<std::endl;
//							left_group = subgroup;
//						}
//		}
//
//	}
//	std::cout << "size is " << joint_model_groups_.size() << std::endl;
////    joint_group_index += (joint_group_index + 1) % (joint_model_groups_.size()-1);
//    current_joint_model_ = joint_model_groups_[joint_group_index];
//	dualEndPointControlInternal(current_state, temp_display,started_in_collision, temp_object, planning_scene,input1);
//	joint_group_index += (joint_group_index + 1) % (joint_model_groups_.size()-1);
//	current_joint_model_ = joint_model_groups_[joint_group_index];
//
//	dualEndPointControlInternal(current_state, temp_display,started_in_collision, temp_object, planning_scene,input1); // Still need to take in control groups

	current_joint_model_ = original_joint_model;

}
struct ee_state{
	bool ik_found = false;
	std::vector<std::string> joint_names;
	std::vector<std::string> ee_names;
	std::vector<double> joint_values;
	std::vector<double> ee_commands;
};

void JoystickControl::dualEndPointControlInternal(robot_state::RobotState& current_state, moveit_msgs::DisplayRobotState& temp_display, planning_scene::PlanningScene& planning_scene,geometry_msgs::Twist input1, std::vector<std::pair<moveit::core::JointModelGroup*, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>> working_groups, moveit::core::JointModelGroup* original_joint_model_group)
{

	//color_current = color_endpoint_;
	std::vector<ee_state> output;

	// handle end effector control, if present
//	std::cout << "num of move groups " << move_groups_.size() << std::endl;

	for(auto pair : working_groups){
		output.push_back(ee_state());
		ee_state& working_ee_state = output.back();
	auto working_joint_model_group = pair.first;
	auto working_move_group = pair.second;
	current_joint_model_ = working_joint_model_group;

	moveit::core::JointModelGroup* EE_group = kinematic_model->getJointModelGroup(end_effector_map_[working_joint_model_group->getName()]);
	            working_ee_state.ee_names = EE_group->getActiveJointModelNames();
//	static bool flip = false;
	geometry_msgs::PoseStamped endpoint_pose = working_move_group->getCurrentPose();

	EE_commands = processControlInputInternalEE(); // Can we get rid of EE_commands entirely?
	working_ee_state.ee_commands = processControlInputInternalEE();

	std::vector<std::string> joint_names = working_joint_model_group->getActiveJointModelNames();
	working_ee_state.joint_names = joint_names;

	if (space_msg_.repeat() && space_msg_.zero()) {
		//endpoint_pose = working_move_group->getCurrentPose();
//		publishPositionCommands(working_ee_state.joint_names, joint_commands);
		publishPositionCommands(working_ee_state.ee_names, working_ee_state.ee_commands);
		return;
	}


	// convert the pose to a transform
	// apply the space mouse input on top of that transform <- the TF2 library makes this easy with quaternion multiplication
	//convert back to pose and pass to moveit

	//ROS_INFO("current endpoint pose in frame %s at [%2.3f,%2.3f,%2.3f]",endpoint_pose.header.frame_id.c_str(),endpoint_pose.pose.position.x,endpoint_pose.pose.position.y,working_move_group_.pose.position.z);
	tf2::Transform endpoint_tf;
	tf2::fromMsg(endpoint_pose.pose, endpoint_tf);
	//tf2::Vector3 temp_origin = endpoint_tf.getOrigin();
	//ROS_INFO("endpoint_tf before [%2.3f,%2.3f,%2.3f]",temp_origin.x(),temp_origin.y(),temp_origin.z());

	//***************************************************************************
	// to debug user input publish a transform showing an exagerated transform
	static tf2_ros::TransformBroadcaster br;
	tf2::Quaternion debug_q;
	debug_q.setRPY(
			space_msg_.getMsg().angular.x * ANGULAR_SCALE / LOOP_RATE * 3,
			space_msg_.getMsg().angular.y * ANGULAR_SCALE / LOOP_RATE * 3,
			space_msg_.getMsg().angular.z * ANGULAR_SCALE / LOOP_RATE * 3);
	geometry_msgs::TransformStamped debug_tf;
	debug_tf.header = endpoint_pose.header;
	//debug_tf.header.frame_id = working_move_group->getEndEffectorLink();
	debug_tf.header.stamp = ros::Time::now();
	debug_tf.child_frame_id = "target_endpoint";
	debug_tf.transform.translation.x = endpoint_pose.pose.position.x
			+ space_msg_.getMsg().linear.x * LINEAR_SCALE / LOOP_RATE * 3;
	debug_tf.transform.translation.y = endpoint_pose.pose.position.y
			+ space_msg_.getMsg().linear.y * LINEAR_SCALE / LOOP_RATE * 3;
	debug_tf.transform.translation.z = endpoint_pose.pose.position.z
			+ space_msg_.getMsg().linear.z * LINEAR_SCALE / LOOP_RATE * 3;
	//debug_tf.transform.translation.x=space_msg_.getMsg().linear.x;
	//debug_tf.transform.translation.y=space_msg_.getMsg().linear.y;
	//debug_tf.transform.translation.z=space_msg_.getMsg().linear.z;
	debug_q = debug_q * endpoint_tf.getRotation();
	tf2::convert(debug_q, debug_tf.transform.rotation);
	br.sendTransform(debug_tf);
	//***************************************************************************

	tf2::Quaternion q;
	// get current location of transform origin and add linear component of joystick message
	endpoint_tf.setOrigin(
			endpoint_tf.getOrigin()
					+ tf2::Vector3(
							space_msg_.getMsg().linear.x * LINEAR_SCALE
									/ LOOP_RATE,
							space_msg_.getMsg().linear.y * LINEAR_SCALE
									/ LOOP_RATE,
							space_msg_.getMsg().linear.z * LINEAR_SCALE
									/ LOOP_RATE));
	// build quaternion from angular component of joystick message using roll-pitch-yaw (RPY) format
	q.setRPY(space_msg_.getMsg().angular.x * ANGULAR_SCALE / LOOP_RATE,
			space_msg_.getMsg().angular.y * ANGULAR_SCALE / LOOP_RATE,
			space_msg_.getMsg().angular.z * ANGULAR_SCALE / LOOP_RATE);
	// multiply quaternion from joystick by existing pose quaternion and set as new pose quaternion
	endpoint_tf.setRotation(q * endpoint_tf.getRotation());
	//temp_origin = endpoint_tf.getOrigin();
	//ROS_INFO("endpoint_tf after  [%2.3f,%2.3f,%2.3f]",temp_origin.x(),temp_origin.y(),temp_origin.z());

	// convert transfrom back into pose format to send to moveit
	tf2::toMsg(endpoint_tf, endpoint_pose.pose);

	//get current joint positions, for checking if IK solution is too far away
	// TODO consider populating joint data from joint_states message, might be faster
	moveit::core::RobotStatePtr robot_state =
			working_move_group->getCurrentState(wait_time);
			
	bool found_ik;
  if(robot_state == NULL){
    ROS_WARN("JoystickControl::dualEndPointControlInternal robot state pointer is null");
    found_ik = false;
  }else{
    //ROS_INFO("JoystickControl::dualEndPointControlInternal robot state appears valid");
    robot_state->copyJointGroupPositions(working_joint_model_group,
			joint_commands);
		found_ik = robot_state->setFromIK(working_joint_model_group,
			endpoint_pose.pose);
  }
  

	//bool found_ik = robot_state->setFromDiffIK(joint_model_group_, twist, ENDPOINT_LINK, 1.0/LOOP_RATE);
	//const kinematics::KinematicsQueryOptions approx_variable;
	//approx_variable.return_approximate_solution();

	working_ee_state.ik_found = found_ik;
	//TODO:Using the approximate KinematicsQueryOptions 'return_approximate_solutions' method may allow us to control under-actuated systems better.  Other options include using the setApproimateJointValueTarget which uses the MoveitPlanningInterface rather than this robot_state->setFromIK. We could also try using IKFAST or TRACKIK rather than KDL, since KDL is really only for >6 DOF.  The easiest KDL hack is to place this line in your kinematics.yaml "position_only_ik: true"  The drawback is that we cannot control wrist roll.  Approximate solutions would be better for us.
	//bool found_ik = robot_state->setFromIK(joint_model_groups_[joint_model_index], endpoint_pose.pose,1,3.0,is_valid,approx_variable);
	//ROS_INFO("MODE_ENDPOINT - IK returned %d",found_ik);

	if (found_ik) {
		//ROS_INFO("target endpoint pose in frame %s at [%2.3f,%2.3f,%2.3f]",endpoint_pose.header.frame_id.c_str(),endpoint_pose.pose.position.x,endpoint_pose.pose.position.y,endpoint_pose.pose.position.z);
		// get new joint values
		std::vector<double> joint_values;
		robot_state->copyJointGroupPositions(working_joint_model_group, working_ee_state.joint_values);
//		joint_state_last_.position
		double scale_down_value = 1.0;
		// check for jumps that are higher than max velocity.
		// scale all joint values if one joint has too much divergence.

		bool valid_ik = true;
		num_joints_ = working_ee_state.joint_names.size();

		for (size_t i = 0; i < num_joints_; i++) {
			//ROS_INFO("\t%s at %2.3f moving to %2.3f",working_ee_state.joint_names[i].c_str(),joint_commands[i],working_ee_state.joint_values[i]);
			//ROS_INFO("\t%s moving %2.3f",working_ee_state.joint_names[i].c_str(),working_ee_state.joint_values[i] - joint_commands[i]);
			scale_down_value = fmin(scale_down_value, (MAX_VEL / LOOP_RATE) / abs(working_ee_state.joint_values[i] - joint_commands[i]));
		}

		if (scale_down_value < 0.05) // Give up on unreasonable end point commands that make the arm flail.
				{
			ROS_WARN("unreasonable scale_down_value = %f", scale_down_value);
			valid_ik = false;

			// go back to current pose
			endpoint_pose = working_move_group->getCurrentPose();
			return;
		}
		if (scale_down_value < 1.0) {
			//ROS_WARN("scale_down_value = %f", scale_down_value);
			for (size_t i = 0; i < num_joints_; i++) {
				working_ee_state.joint_values[i] = (joint_commands[i] - working_ee_state.joint_values[i])
						* scale_down_value + working_ee_state.joint_values[i];
			}
		}

		// check for self-collisions

		//collision_request.group_name = cur_joint_model_->getName();
		collision_result.clear();
		current_state.setJointGroupPositions(working_joint_model_group,
				working_ee_state.joint_values);
		if (end_effector_map_.count(working_joint_model_group->getName())
				!= 0) {
			// need to check and populate mimic drives because moveit doesn't handle it automatically
			std::vector<double> temp_EE_commands;
			std::vector<std::string> temp_names =
					EE_group->getJointModelNames();
			for (size_t i = 0; i < temp_names.size(); i++) {
				const moveit::core::JointModel *temp_joint; // If this could be null the next line could be trouble
				const moveit::core::JointModel *temp_jointB;
				std::vector<std::string>::iterator it;
				temp_joint = EE_group->getJointModel(temp_names[i]);
				temp_jointB = temp_joint->getMimic(); //Could be cause of issue, pointer is not checked here before it is dereferenced.
				if (temp_jointB != NULL) {
					//ROS_INFO("\t%s mimics %s",temp_joint->getName().c_str(),temp_jointB->getName().c_str());
					it = std::find(working_ee_state.ee_names.begin(), working_ee_state.ee_names.end(),
							temp_jointB->getName());
					temp_EE_commands.push_back(
							working_ee_state.ee_commands[it - working_ee_state.ee_names.begin()]
									* temp_jointB->getMimicFactor()
									+ temp_jointB->getMimicOffset());
				} else {
					//ROS_INFO("\t%s is a regular joint",temp_joint->getName().c_str());
					it = std::find(working_ee_state.ee_names.begin(), working_ee_state.ee_names.end(),
							temp_joint->getName());
					temp_EE_commands.push_back(
							working_ee_state.ee_commands[it - working_ee_state.ee_names.begin()]);
				}
			}
			current_state.setJointGroupPositions(EE_group, temp_EE_commands);

		}
		planning_scene.checkSelfCollision(collision_request, collision_result);
    if(collision_result.collision){
      ROS_WARN("collision found:");
      std::vector<std::string> colliding_links;
      planning_scene.getCollidingLinks(colliding_links,current_state);
      moveit_msgs::ObjectColor temp_object;
      temp_object.color = joystick_display_colors[MODE_RESET];
      for(int i=0; i<colliding_links.size(); ++i){
        ROS_WARN("\t%s",colliding_links[i].c_str());
        // check if this link is already in the highlight list
        bool link_found = false;
        for(int j=0; j<temp_display.highlight_links.size(); j++){
          if(temp_display.highlight_links[j].id == colliding_links[i]){
            temp_display.highlight_links[j].color = joystick_display_colors[MODE_RESET];
            link_found = true;
          }
        }
        if(!link_found){
          temp_object.id = colliding_links[i];
          temp_display.highlight_links.push_back(temp_object);
        }
      }
			// go back to current pose
			endpoint_pose = working_move_group->getCurrentPose();
		} else if (!current_state.satisfiesBounds(working_joint_model_group)) {
			ROS_WARN("outside position limits");
			// go back to current pose
			endpoint_pose = working_move_group->getCurrentPose();
		} else { //Just here for debug, can remove
			ROS_INFO("%s success", working_joint_model_group->getName().c_str());
		}
		//}
	} else { //I think we can remove this else
		ROS_WARN("no ik solution found");
		// go back to current pose
		endpoint_pose = working_move_group->getCurrentPose();
	}
	}
	current_joint_model_ = original_joint_model_group;
	//If the number of solutions in output matches the number of groups we want to control
	if(output.size() == working_groups.size()){
//			for(ee_state working_state : output){
//				publishPositionCommands(working_state.joint_names, working_state.joint_values); // This worked, but not in endpoint
//				publishPositionCommands(working_state.ee_names, working_state.ee_commands); // This worked but not n
//
//		}
	    ee_state final_output;
		for(ee_state ee_state_working_state : output){
			// If one of the joints has in inverse kinematic failure, quit
			if(!ee_state_working_state.ik_found){
				return;
			}
			final_output.joint_names.insert(final_output.joint_names.end(), ee_state_working_state.joint_names.begin(), ee_state_working_state.joint_names.end());
			final_output.ee_names.insert(final_output.ee_names.end(), ee_state_working_state.ee_names.begin(), ee_state_working_state.ee_names.end());
			final_output.joint_values.insert(final_output.joint_values.end(), ee_state_working_state.joint_values.begin(), ee_state_working_state.joint_values.end());
			final_output.ee_commands.insert(final_output.ee_commands.end(), ee_state_working_state.ee_commands.begin(), ee_state_working_state.ee_commands.end());
		}
		publishPositionCommands(final_output.joint_names, final_output.joint_values); // This worked, but not in endpoint
		publishPositionCommands(final_output.ee_names, final_output.ee_commands); // This worked but not n
	}



}

//std::vector<double> JoystickControl::getJointStatePosition(){
//	std::scope_lock<std::mutex> lock(joint_state_mutex_);
//	return joint_state_last_.position;
//
//}

/*----------------------------------------------------------------------------
  main
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_control");

  
  ros::NodeHandle nh;
  
  // start spinner
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  JoystickControl joystick_control(nh);
  
//  if (joystick_control.init())
//  {
    joystick_control.run();
//  }
  
  return 0;
}
