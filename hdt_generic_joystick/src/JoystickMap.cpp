/*
 * JoystickMap.cpp
 *
 *  Created on: May 8, 2020
 *      Author: hdt-global
 */

#include <hdt_generic_joystick/JoystickMap.h>
#include <hdt_generic_joystick/adroit_constants.h>
#include <ros/ros.h>
#include <exception>
#include <vector>



JoystickMap::JoystickMap(){
		joystick_map_ = std::map<std::string, JoyMapEntry>();
	}
	JoystickMap::JoystickMap(std::map<std::string, JoyMapEntry > inner_map){
		joystick_map_ = inner_map;
	}


JoystickMap::JoystickMap(const ros::NodeHandle &nh) {
	std::string prefix = "/joystick_map/";
	std::vector<std::string> keys;
	nh.getParamNames(keys);
	std::string name_string;
	std::string temp_string;
	for (std::string key : keys) {
		if (key.find(prefix) != std::string::npos) {
			name_string = key.substr(prefix.length(), key.length());
			name_string = name_string.substr(0, name_string.find("/"));
			std::cout << "name_string is: " << name_string << std::endl;
			std::string param;
			int axisA_value;
			int axisB_value;
			bool invert_value;
			nh.getParam(prefix + name_string + "/input", param);
			JoyMapEntry nextEntry{
			   (param == "axes") ? AXES : BUTTONS,
			    nh.getParam(prefix+name_string+"/axisA",axisA_value)  ?  axisA_value : throw std::runtime_error("no axisA value in config file!"),
			    nh.getParam(prefix+name_string+"/axisB",axisB_value)  ?  axisB_value : -1 ,
				nh.getParam(prefix+name_string+"/invert",invert_value) ? invert_value : false};
			joystick_map_.emplace(name_string, nextEntry); //Emplace will not override
		}
	}
}

	double JoystickMap::proccessJoyMessage(const std::string& joint_name, const sensor_msgs::Joy& msg){
		return joystick_map_.at(joint_name).proccessJoyMapEntry(msg);
	}
	
	// return joystick values matching joint map
	std::vector<double> JoystickMap::parseJointModel(const std::vector<const moveit::core::JointModel*>& models, const sensor_msgs::Joy& msg){
		size_t i = 0;
		std::vector<double> parsed_output(models.size());
		for(const moveit::core::JointModel* joint_model : models){
			parsed_output[i] = proccessJoyMessage(joint_model->getName(), msg);
			i++;
		}
		return parsed_output;
	}
	
	// increment position commands in input array with joystick values
	void JoystickMap::parseJointModel(const std::vector<const moveit::core::JointModel*>& models, const sensor_msgs::Joy& msg, std::vector<double>& input){
		size_t i = 0;
		std::vector<double> parsed_output(models.size());
		for(const moveit::core::JointModel* joint_model : models){
			input[i] += proccessJoyMessage(joint_model->getName(), msg);
			i++;
		}
	}
	std::vector<double> JoystickMap::parseJointModel(const moveit::core::JointModelGroup* model_group, const sensor_msgs::Joy& msg){
		return parseJointModel(model_group->getActiveJointModels(), msg);
	}

	void JoystickMap::parseJointModel(const moveit::core::JointModelGroup* model_group, const sensor_msgs::Joy& msg, std::vector<double>& input){
			parseJointModel(model_group->getActiveJointModels(), msg, input);
		}

	/**
	 * Returns the values in a given joint message for a given specified joint name
	 */
	std::vector<int> JoystickMap::getJoystickValueByJointName(const std::string& joint_name, const sensor_msgs::Joy& msg){
		JoyMapEntry& working_map_entry = joystick_map_.at(joint_name);
		if(working_map_entry.axisB >= 0){
			return std::vector<int>{msg.axes[working_map_entry.axisA], msg.axes[working_map_entry.axisB]};
		}
		return std::vector<int>{msg.axes[working_map_entry.axisA]};
	}




