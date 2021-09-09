#include <hdt_generic_joystick/generic_joystick.h>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//TODO: Are all 3 of these needed?
#include <ros/ros.h>
//#include <joint_limits_interface/joint_limits.h>
//#include <joint_limits_interface/joint_limits_urdf.h>
//#include <joint_limits_interface/joint_limits_rosparam.h>

#include <iostream>

/*----------------------------------------------------------------------------
  constructor
 *----------------------------------------------------------------------------*/
JoystickControl::JoystickControl(const ros::NodeHandle& nh) : nh_(nh)
{
}

/*----------------------------------------------------------------------------
  destructor
 *----------------------------------------------------------------------------*/
JoystickControl::~JoystickControl()
{
}

/*----------------------------------------------------------------------------
  init
 *----------------------------------------------------------------------------*/
bool JoystickControl::init()
{ 
  mode_ = MODE_NONE;
  
  joy_sub_ = nh_.subscribe("joy", 10, &JoystickControl::joyCallback, this);
  
  // why is this node publishing a joystick topic that isn't being used?
  //joy_pub_ = nh_.advertise<sensor_msgs::Joy>(JOY_TOPIC, 10);
  
  joy_msg_.axes.resize(NUM_AXES);
  joy_msg_.buttons.resize(NUM_BUTTONS);
  joy_prev_ = joy_msg_;
  
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(MOVE_GROUP);
  move_group_->setMaxVelocityScalingFactor(VEL_SCALE);
  move_group_->setEndEffectorLink(ENDPOINT_LINK);
  
  moveit::core::RobotModelConstPtr robot_model = move_group_->getRobotModel();
  joint_model_group_ = robot_model->getJointModelGroup(MOVE_GROUP);
  
  joint_names_ = move_group_->getActiveJoints();
  num_joints_ = joint_names_.size();
  joint_values_.resize(num_joints_);
  
  //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  //const moveit::core::JointBoundsVector& bounds_vector = kinematic_model->getActiveJointModelsBounds();
  const moveit::core::JointBoundsVector& bounds_vector = joint_model_group_->getActiveJointModelsBounds();

  //std::vector<moveit::core::VariableBounds> joint_bounds;
  //joint_bounds.resize(num_joints_);

  for(std::size_t i=0; i<bounds_vector.size();i++){
    //ROS_INFO("\tthis one is %d long", bounds_vector[i]->size());
    moveit::core::JointModel::Bounds temp_bounds = *bounds_vector[i];
    joint_bounds.push_back(temp_bounds[0]);
    /*
    for(std::size_t j=0; j<temp_bounds.size(); j++){
      //ROS_INFO("\t\tthis one is %d long", temp_bounds[j].size());
      ROS_INFO("\tposition limits %f %f", temp_bounds[j].max_position_,temp_bounds[j].min_position_);
    }
    */
    
  }

  if(num_joints_ != joint_bounds.size()){
    //something wierd has happened, joint info vectors are different lengths
    ROS_WARN("ActiveJoints and JointBounds vectors are different lengths");
  }else{
    for(std::size_t i=0; i<joint_bounds.size();i++){
      ROS_INFO("%s has position limits %f to %f",joint_names_[i].c_str(),joint_bounds[i].max_position_,joint_bounds[i].min_position_);
    }
  }

  // get joint names and position limits from end effector group
  const moveit::core::JointModelGroup* EE_model_group_ = robot_model->getJointModelGroup("hand");
  EE_names_ = EE_model_group_->getActiveJointModelNames();
  num_EE_ = EE_names_.size();
  std::vector<double> EE_values_;
  EE_values_.resize(num_EE_);
  const moveit::core::JointBoundsVector& EE_bounds_vector = EE_model_group_->getActiveJointModelsBounds();
  //std::vector<moveit::core::VariableBounds> EE_bounds;
  for(std::size_t i=0; i<EE_bounds_vector.size();i++){
    //ROS_INFO("\tthis one is %d long", bounds_vector[i]->size());
    moveit::core::JointModel::Bounds temp_bounds = *EE_bounds_vector[i];
    EE_bounds.push_back(temp_bounds[0]);
    
  }
  if(num_EE_ != EE_bounds.size()){
    //something wierd has happened, joint info vectors are different lengths
    ROS_WARN("ActiveJoints and JointBounds vectors are different lengths for EE");
  }else{
    for(std::size_t i=0; i<EE_bounds.size();i++){
      ROS_INFO("%s has position limits %f to %f",EE_names_[i].c_str(),EE_bounds[i].max_position_,EE_bounds[i].min_position_);
    }
  }
  // declaring joint_limit_interface variables  
  //boost::shared_ptr<urdf::ModelInterface> urdf;
  //joint_limits_interface::JointLimits limits;
  //joint_limits_interface::SoftJointLimits soft_limits;
  /*
  	// check for robot description
	urdf::Model model;
	if(!model.initParam("robot_description")) {
		ROS_ERROR("could not read robot description");
	}
  
  // iterate through joints
	typedef std::map< std::string, boost::shared_ptr<urdf::Joint> >::iterator it_type;
	for(it_type iterator = model.joints_.begin(); iterator != model.joints_.end(); iterator++) {
	boost::shared_ptr<urdf::Joint> joint = iterator->second;
*/
 

  for (auto& joint_name : joint_names_)
  {   
    // add joint to controller list
    std::string controller = "/hdt_arm/" + joint_name + "_position_controller";
    joint_controllers_.push_back(controller);
    
    
    // add joint to pub list
    std::string topic = controller + "/command";
    ros::Publisher pub = nh_.advertise<std_msgs::Float64>(topic, 10);
    joint_pubs_[joint_name] = pub;
    //ROS_INFO("creating publisher for %s", joint_name.c_str());
  }
  
  // pincer controller
  std::string controller = "/hdt_arm/pincer_joint_position_controller";
  std::string topic = controller + "/command";
  pincer_pub_ = nh_.advertise<std_msgs::Float64>(topic, 10);
  
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
  
  return true;
}

/*----------------------------------------------------------------------------
  run
 *----------------------------------------------------------------------------*/
void JoystickControl::run()
{
  std::string name;
  ros::Rate rate(LOOP_RATE);

  while (ros::ok())
  {
    // acquire mutex
    std::unique_lock<std::mutex> lock(mutex_);
    
    // check for button transitions
    if ((joy_msg_.buttons[NAMED_BUTTON] == 0) && (joy_prev_.buttons[NAMED_BUTTON] == 1))
    {
      // set named mode and name
      mode_ = switchMode(MODE_NAMED);
      name = "home";
    }
    else if ((joy_msg_.buttons[JOINT_BUTTON] == 0) && (joy_prev_.buttons[JOINT_BUTTON] == 1))
    {
      // toggle joint mode
      switch (mode_)
      {
        case MODE_NONE:
          mode_ = switchMode(MODE_JOINT);
          break;
        case MODE_JOINT:
          mode_ = switchMode(MODE_NONE);
          break;
        default:
          break;
      }
    }
    else if ((joy_msg_.buttons[ENDPOINT_BUTTON] == 0) && (joy_prev_.buttons[ENDPOINT_BUTTON] == 1))
    {
      // toggle joint mode
      switch (mode_)
      {
        case MODE_NONE:
          mode_ = switchMode(MODE_ENDPOINT);
          break;
        case MODE_JOINT:
          mode_ = switchMode(MODE_NONE);
          break;
        default:
          break;
      }
    }
    
    // update joy prev
    joy_prev_ = joy_msg_;
    
    lock.unlock();
    
    // take mode-dependent action
    switch (mode_)
    {
      case MODE_NAMED: {
        move_group_->setNamedTarget(name);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_->plan(plan);
        move_group_->execute(plan);
        mode_ = switchMode(MODE_NONE);
        break;
      }
      case MODE_JOINT: {
              
        // increment joint values
        std::vector<double> joint_values = joint_values_;
        joint_values_[0] = joint_values[0] + joy_prev_.axes[0]*JOINT_SCALE;
        joint_values_[1] = joint_values[1] + joy_prev_.axes[1]*JOINT_SCALE;
        joint_values_[2] = joint_values[2] + joy_prev_.axes[3]*JOINT_SCALE;
        joint_values_[3] = joint_values[3] + joy_prev_.axes[2]*JOINT_SCALE;
        joint_values_[4] = joint_values[4] + joy_prev_.axes[6]*JOINT_SCALE;
        joint_values_[5] = joint_values[5] + joy_prev_.axes[5]*JOINT_SCALE;
        for(int i=0; i<joint_bounds.size();i++){
          if(joint_values_[i] > joint_bounds[i].min_position_ and joint_values_[i] < joint_bounds[i].max_position_){
            joint_values[i] = joint_values_[i];
          }

        }

        // set joint values
        setValues(joint_values);
        break;
      }
      case MODE_ENDPOINT: {
        // generate endpoint twist
        //geometry_msgs::Twist twist;
        //twist.linear.x = -joy_prev_.axes[3]*LINEAR_SCALE;
        //twist.linear.y = joy_prev_.axes[4]*LINEAR_SCALE;
        //twist.linear.z = joy_prev_.axes[2]*LINEAR_SCALE;
        //twist.angular.x = -joy_prev_.axes[5]*ANGULAR_SCALE;
        //twist.angular.y = joy_prev_.axes[0]*ANGULAR_SCALE;
        //twist.angular.z = joy_prev_.axes[1]*ANGULAR_SCALE;
        
        // get endpoint tf
        tf2::Transform endpoint_tf;
        tf2::fromMsg(endpoint_pose_.pose, endpoint_tf);
        
        // get joy tfjoint_pubs_
	//TODO make joystick to coordinate axis mapping configurable with ROS parameters
        tf2::Transform joy_tf;
        joy_tf.setOrigin(tf2::Vector3(joy_prev_.axes[3]*LINEAR_SCALE/LOOP_RATE, -joy_prev_.axes[2]*LINEAR_SCALE/LOOP_RATE, joy_prev_.axes[6]*LINEAR_SCALE/LOOP_RATE));
        joy_tf.setRotation(tf2::Quaternion(joy_prev_.axes[1]*ANGULAR_SCALE/LOOP_RATE, joy_prev_.axes[5]*ANGULAR_SCALE/LOOP_RATE, -joy_prev_.axes[0]*ANGULAR_SCALE/LOOP_RATE));
        
        // calculate new endpoint pose
        geometry_msgs::PoseStamped endpoint_pose;
        endpoint_pose.header.stamp = ros::Time::now();
        endpoint_pose.header.frame_id = endpoint_pose_.header.frame_id;	// this creates motion in end point coordinate frame
        tf2::toMsg(endpoint_tf*joy_tf, endpoint_pose.pose);  // rotation motion in end point coordinate frame not base frame
	
        // solve ik
        moveit::core::RobotStatePtr robot_state = move_group_->getCurrentState();
        //bool found_ik = robot_state->setFromDiffIK(joint_model_group_, twist, ENDPOINT_LINK, 1.0/LOOP_RATE);
        bool found_ik = robot_state->setFromIK(joint_model_group_, endpoint_pose.pose, ENDPOINT_LINK);
        
        if (found_ik)
        {
          // get new joint values
          std::vector<double> joint_values;
          robot_state->copyJointGroupPositions(joint_model_group_, joint_values);
          double scale_down_value = 1.0;
          // check for jumps that are higher than max velocity.
          // scale all joint values if one joint has too much divergence.
          bool valid_ik = true;
          for (size_t i = 0; i < num_joints_; i++)
          {   
            if ((joint_values[i] - joint_values_[i]) > MAX_VEL/LOOP_RATE)
            {
              //ROS_WARN("%s jumped from %1.3f to %1.3f", joint_names_[i].c_str(), joint_values_[i], joint_values[i]);
              if (scale_down_value > (MAX_VEL/LOOP_RATE)/(joint_values[i] - joint_values_[i]))
              {
              	scale_down_value = (MAX_VEL/LOOP_RATE)/(joint_values[i] - joint_values_[i]);
              	//ROS_WARN("Scale down = %1.3f", scale_down_value);
              	if (scale_down_value  < 0.05)  // Give up on unreasonable end point commands that make the arm flail.
              	{
              		valid_ik = false;
              	}
              }
            }
            if ((joint_values_[i] - joint_values[i]) > MAX_VEL/LOOP_RATE)
            {
              //ROS_ERROR("%s jumped from %1.3f to %1.3f", joint_names_[i].c_str(), joint_values_[i], joint_values[i]);
              if (scale_down_value > (MAX_VEL/LOOP_RATE)/(joint_values_[i] - joint_values[i]))
              {
              	scale_down_value = (MAX_VEL/LOOP_RATE)/(joint_values_[i] - joint_values[i]);
              	//ROS_ERROR("Scale down = %1.3f", scale_down_value);
                if (scale_down_value  < 0.05)  // Give up on unreasonable end point commands that make the arm flail.
              	{
              		valid_ik = false;
              	}
              }
            }
          }
          
          // if valid, set joint values
          if (valid_ik)
          {
          	for (size_t i = 0; i < num_joints_; i++)
          	{   
          		if(joint_values[i] > joint_values_[i])
          		{
          		joint_values[i] = (joint_values[i]-joint_values_[i])*scale_down_value + joint_values[i];
          		}
          		else
          		{
          		joint_values[i] = (joint_values_[i]-joint_values[i])*scale_down_value + joint_values[i];
          		}
          	}
            
            setValues(joint_values);
            endpoint_pose_ = endpoint_pose;
          }
        }
        else
        {
          ROS_WARN("no ik solution found");
        }
      }
      default:
        break;
    }
    
    // update pincer value
    pincer_value_ += joy_prev_.axes[4]*JOINT_SCALE;
    pincer_value_=fmin(pincer_value_,EE_bounds[0].max_position_);
    pincer_value_=fmax(pincer_value_,EE_bounds[0].min_position_);

    // send updated command
    std_msgs::Float64 command;
    command.data = pincer_value_;
    pincer_pub_.publish(command);
    
    rate.sleep();
  }
}

/*----------------------------------------------------------------------------
  switch mode
 *----------------------------------------------------------------------------*/
JoystickControl::Mode JoystickControl::switchMode(const Mode& new_mode)
{
  controller_manager_msgs::SwitchController srv;
  srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  
  // if switching to joint or endpoint, enable joint controller
  if ((new_mode == MODE_JOINT) || (new_mode == MODE_ENDPOINT))
  {
    // get current values
    joint_values_ = move_group_->getCurrentJointValues();
    endpoint_pose_ = move_group_->getCurrentPose();
    
    srv.request.start_controllers.insert(std::end(srv.request.start_controllers), std::begin(joint_controllers_), std::end(joint_controllers_));
    srv.request.stop_controllers.push_back(TRAJ_CONTROLLER);
  }
  // if switching from joint or endpoint, disable joint controller
  else if ((mode_ == MODE_JOINT) || (mode_ == MODE_ENDPOINT))
  {
    srv.request.start_controllers.push_back(TRAJ_CONTROLLER);
    srv.request.stop_controllers.insert(std::end(srv.request.stop_controllers), std::begin(joint_controllers_), std::end(joint_controllers_));
  }
  
  ros::service::waitForService("/controller_manager/switch_controller");
  ros::ServiceClient switch_controller = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller", true);
  
  if (switch_controller.call(srv))
  {
    if (srv.response.ok)
    {
      ROS_INFO("switching from mode %d to %d", mode_, new_mode);
      return new_mode;
    }
    else
    {
      ROS_ERROR("service call failed");
    }
  }
  
  return mode_;
}

/*----------------------------------------------------------------------------
  joy callback
 *----------------------------------------------------------------------------*/
void JoystickControl::joyCallback(const sensor_msgs::Joy& msg)
{
  std::unique_lock<std::mutex> lock(mutex_);
  
  joy_msg_.axes[0] = msg.axes[0];
  joy_msg_.axes[1] = msg.axes[1];
  joy_msg_.axes[2] = -msg.axes[3];
  joy_msg_.axes[3] = msg.axes[4];
  if (msg.axes[2] - msg.axes[5] > 0.05) {joy_msg_.axes[4] = (msg.axes[2] - msg.axes[5])/2.0;} // Deadband for triggers
  if (msg.axes[2] - msg.axes[5] < 0.05) {joy_msg_.axes[4] = (msg.axes[2] - msg.axes[5])/2.0;} // Deadband for triggers
  //joy_msg_.axes[4] = (msg.axes[2] - msg.axes[5])/2.0;
  joy_msg_.axes[5] = static_cast<float>(msg.buttons[5] - msg.buttons[4]);
  joy_msg_.axes[6] = msg.axes[7];
  joy_msg_.axes[7] = msg.axes[6];
  
  joy_msg_.buttons[0] = msg.buttons[0];
  joy_msg_.buttons[1] = msg.buttons[1];
  joy_msg_.buttons[2] = msg.buttons[2];
  
  // what is this for?
  //joy_pub_.publish(joy_msg_);
}

/*----------------------------------------------------------------------------
  set values
 *----------------------------------------------------------------------------*/
void JoystickControl::setValues(const std::vector<double>& joint_values)
{
  // copy joint values
  joint_values_ = joint_values;
  
  // iterate through active joints
  for (size_t i = 0; i < num_joints_; i++)
  {   
    // send updated command
    std_msgs::Float64 command;
    command.data = joint_values_[i];
    joint_pubs_[joint_names_[i]].publish(command);
  }
}

/*----------------------------------------------------------------------------
  main
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_control");

  ros::NodeHandle nh;
  
  // start spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  JoystickControl joystick_control(nh);
  if (joystick_control.init())
  {
    joystick_control.run();
  }
  
  return 0;
}
