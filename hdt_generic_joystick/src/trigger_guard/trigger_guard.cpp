#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <message_filters/subscriber.h>
#include <utility>

ros::Subscriber joy_sub;
ros::Publisher pub;

std::vector<XmlRpc::XmlRpcValue> guarded_triggers_list;
std::map<int, bool> guard_map;
const int trigger_axesA = 5;
const int trigger_axesB = 2;
bool a_triggered = false;
bool b_triggered = false;
std::pair<const int, bool> pair1 = std::pair<const int, bool>(trigger_axesA, a_triggered);
std::pair<const int, bool> pair2 = std::pair<const int, bool>(trigger_axesB, b_triggered);
bool warn_once = true;
bool triggered = false;


/*----------------------------------------------------------------------------
  joy callback
 *----------------------------------------------------------------------------*/
void joyCallback(const sensor_msgs::Joy& msg)
{

  sensor_msgs::Joy output(msg);
  if(!a_triggered || !b_triggered){
	  if(warn_once){
	  ROS_WARN("PRESS BOTH TRIGGERS TO ENABLE CONTROL");
	  warn_once = false;
	  }
	  if(msg.axes[trigger_axesA] != 0){
		a_triggered = true;
	  }
	  else if(!a_triggered){
		  output.axes[trigger_axesA] = 1;
	  }
	  if(msg.axes[trigger_axesB] != 0){
	  	b_triggered = true;
	  }
	  else if(!b_triggered){
		  output.axes[trigger_axesB] = 1;
	  }
  }
  else{
  pub.publish(output);
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "trigger_guard");
  ros::NodeHandle nh;;
  for(auto& entry : guard_map){
	  entry.second = false;
  }
  
  pub = nh.advertise<sensor_msgs::Joy>("guarded_joy", 1);
  message_filters::Subscriber<sensor_msgs::Joy> sub(nh, "joy", 1);
  sub.registerCallback(joyCallback);


  ros::spin();

  return 0;
}
