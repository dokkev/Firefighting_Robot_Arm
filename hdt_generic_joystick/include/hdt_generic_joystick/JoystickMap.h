/*
 * JoyStickMap.h
 *
 *  Created on: May 8, 2020
 *      Author: hdt-global
 */

#ifndef INCLUDE_HDT_GENERIC_JOYSTICK_JOYSTICKMAP_H_
#define INCLUDE_HDT_GENERIC_JOYSTICK_JOYSTICKMAP_H_

#include <map>
#include <string>
#include <hdt_generic_joystick/MsgWrapper.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <hdt_generic_joystick/adroit_constants.h>
#include <moveit/robot_model/joint_model_group.h>
#include <vector>

enum input_type{
	AXES,
	BUTTONS
};

	struct JoyMapEntry{
	input_type input;
	    int axisA;
	    int axisB;
	    bool invert;

	double proccessJoyMapEntry(sensor_msgs::Joy msg) {
		float joy_input;
		switch (input) {
		case AXES:

			if (axisB >= 0) {
				joy_input = (msg.axes[axisA] - msg.axes[axisB]) * JOINT_SCALE
						/ 2;
			} else {
				joy_input = msg.axes[axisA] * JOINT_SCALE;
			}

			break;
		case BUTTONS:

			if (axisB >= 0) {
				joy_input = (msg.buttons[axisA] - msg.buttons[axisB])
						* JOINT_SCALE;
			} else {
				joy_input = msg.buttons[axisA] * JOINT_SCALE;
			}

		}

		if (invert) {
			joy_input = -joy_input;
		}
		return joy_input;
	}
};

class JoystickMap{



	using joymapentries_t = std::map<std::string, JoyMapEntry> ;
	joymapentries_t joystick_map_;

public:
	JoystickMap();
	JoystickMap( std::map<std::string, JoyMapEntry > inner_map);
	JoystickMap(const ros::NodeHandle& nh);

	double proccessJoyMessage(const std::string& jointName, const sensor_msgs::Joy& msg);
	std::vector<double> parseJointModel(const std::vector<const moveit::core::JointModel*>& models, const sensor_msgs::Joy& msg);
	void parseJointModel(const std::vector<const moveit::core::JointModel*>& models, const sensor_msgs::Joy& msg, std::vector<double>& input);
	std::vector<double> parseJointModel(const moveit::core::JointModelGroup* model_group, const sensor_msgs::Joy& msg);
	void parseJointModel(const moveit::core::JointModelGroup* model_group, const sensor_msgs::Joy& msg, std::vector<double>& input);
	std::vector<int> getJoystickValueByJointName(const std::string& joint_name, const sensor_msgs::Joy& msg);
	JoyMapEntry& operator[](std::string key)
	{
	    return joystick_map_[key];
	}

	using const_iterator = joymapentries_t::const_iterator;

	  const_iterator begin() const { return joystick_map_.begin(); }
	  const_iterator end() const { return joystick_map_.end(); }
	  const_iterator cbegin() const { return joystick_map_.cbegin(); }
	  const_iterator cend() const { return joystick_map_.cend(); }

};
//  std::map<std::string, int> button_map;




#endif /* INCLUDE_HDT_GENERIC_JOYSTICK_JOYSTICKMAP_H_ */
