#include <hdt_adroit_control/adroit_interface.h>

#include <controller_manager/controller_manager.h>

#include <sstream>
#include <std_msgs/Bool.h>

static const int SAMPLE_RATE = 10.0;

/*----------------------------------------------------------------------------
  main function
 *----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "adroit_control");
  
  // async spinner for controller manager(??)
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  // get private params
  ros::NodeHandle private_nh("~");
  std::string robot_name;
  bool fake_execution;
  
  if (!private_nh.getParam("robot_name", robot_name))
  {
    ROS_ERROR("Robot name not provided, exiting...");
    return 0;
  }
  private_nh.param<bool>("fake_execution", fake_execution, false);
  
  // hardware interface
  ros::NodeHandle nh;
  AdroitInterface robot(robot_name, fake_execution);
  if (robot.init(nh) == false)
    return 0;
  
  // controller manager
  controller_manager::ControllerManager manager(&robot);
 
  // look for sample rate
  std::stringstream loop_hz_param;
  loop_hz_param << robot_name << "/hardware_interface/loop_hz";
  double loop_hz;
  if (!nh.getParam(loop_hz_param.str(), loop_hz))
  {
    // otherwise set to default
    loop_hz= SAMPLE_RATE;
  }
  
  // update loop
  ros::Rate rate(loop_hz);
  ros::Time last = ros::Time::now();
  
  ros::Publisher robot_connected_pub = nh.advertise<std_msgs::Bool>("manipulator_connected", 10);
  std_msgs::Bool robot_connected;
  robot_connected.data=false;
  
  int connect_counter=0;
  while (ros::ok())
  {
    // get time
    ros::Time now = ros::Time::now();
    ros::Duration period = now - last;
    //ROS_INFO_STREAM("AdroitControl:main period " << period);
    
    //TODO consider replacing if statements with switch-case and treat as a state machine
    
    // get current state
    if(robot.read())
    {
      connect_counter++;
      if(connect_counter>=5){
        if(!robot_connected.data){
          ROS_INFO("connected to manipulator arm");
          robot.reset();
        }
        connect_counter=5;
        robot_connected.data=true;
      }
    }else{
      connect_counter--;
      if(connect_counter<=0){
        if(robot_connected.data){
          ROS_WARN("connection to manipulator arm lost");
        }
        connect_counter=0;
        robot_connected.data=false;
      }
    }
    if(robot_connected.data){
      // update controller manager, without reseting controllers
      manager.update(now, period,false);
      // write to hw interface
      robot.write();
    }else{
      // update controller manager, reseting controllers
      manager.update(now, period,true);
    }
    robot_connected_pub.publish(robot_connected);
    // update time and sleep
    last = now;
    rate.sleep();
  }
  
  // wait for shutdown
  ros::waitForShutdown();
    
  return 0;
}
