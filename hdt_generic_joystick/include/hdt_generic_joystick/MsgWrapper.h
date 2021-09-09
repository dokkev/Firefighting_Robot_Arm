/*
 * MsgWrapper.h
 *
 *  Created on: May 8, 2020
 *      Author: Jared Deane
 */

#ifndef INCLUDE_MSGWRAPPER_H_
#define INCLUDE_MSGWRAPPER_H_

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

/**
 * Template class for wrapping around a message. Keeps track of the previous value of the message when a new one is set.
 */
template<class T>
class MsgWrapper {
protected:
	T msg_, prev_;
public:
	MsgWrapper() {
	}
	virtual ~MsgWrapper() {
	}
	MsgWrapper(T msg) {
		msg_ = prev_ = msg;
	}
	/**
	 * Sets the msg_ value to newMsg and store the previous msg_ value in prev_
	 */
	void setMsg(T newMsg) {
		prev_ = msg_;
		msg_ = newMsg;
	}
	T getMsg() {
		return msg_;
	}

	T getPrevMsg() {
		return prev_;
	}
	/**
	 * Returns true if current message and previous message contain the same content
	 */
	virtual bool repeat()=0;
};

class TwistWrapper: virtual public MsgWrapper<geometry_msgs::Twist> {
public:
	TwistWrapper() {
	}
	TwistWrapper(geometry_msgs::Twist twist_msg) :
			MsgWrapper(twist_msg) {
	}
	bool repeat() {
		return (msg_.angular.x == prev_.angular.x
				and msg_.angular.y == prev_.angular.y
				and msg_.angular.z == prev_.angular.z
				and msg_.linear.x == prev_.linear.x
				and msg_.linear.y == prev_.linear.y
				and msg_.linear.z == prev_.linear.z);
	}
	/**
	 * Returns true if all the angular and linear values are all zero
	 */
	bool zero() {
		return (msg_.angular.x == 0.0 and msg_.angular.y == 0.0
				and msg_.angular.z == 0.0 and msg_.linear.x == 0.0
				and msg_.linear.y == 0.0 and msg_.linear.z == 0.0);
	}

};

class JoyWrapper: virtual public MsgWrapper<sensor_msgs::Joy> {
public:
	JoyWrapper() {
	}
	JoyWrapper(sensor_msgs::Joy joy_msg) :
			MsgWrapper(joy_msg) {
	}
	bool repeat() override{
		return (msg_.axes == prev_.axes && msg_.buttons == prev_.buttons);
	}
};

class JointStateWrapper: public MsgWrapper<sensor_msgs::JointState> {
public:
	bool repeat() override{
		bool is_repeat = true;
		for (auto msg_effort_iter = msg_.effort.begin(), msg_velocity_iter =
				msg_.velocity.begin(), msg_position_iter =
				msg_.position.begin(), prev_effort_iter = prev_.effort.begin(),
				prev_velocity_iter = prev_.velocity.begin(),
				prev_position_iter = prev_.position.begin();
				msg_effort_iter != msg_.effort.end();
				++msg_effort_iter, ++msg_velocity_iter, ++msg_position_iter, ++prev_effort_iter, ++prev_velocity_iter, ++prev_position_iter) {
			is_repeat &=(msg_effort_iter == prev_effort_iter
					and msg_velocity_iter == prev_velocity_iter
					and msg_position_iter == prev_position_iter);
		}
		return is_repeat;
	}
};

#endif /* INCLUDE_MSGWRAPPER_H_ */
