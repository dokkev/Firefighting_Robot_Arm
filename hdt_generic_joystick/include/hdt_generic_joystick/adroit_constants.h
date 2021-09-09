/*
 * adroit_constants.h
 *
 *  Created on: May 8, 2020
 *      Author: hdt-global
 */

#ifndef INCLUDE_HDT_GENERIC_JOYSTICK_ADROIT_CONSTANTS_H_
#define INCLUDE_HDT_GENERIC_JOYSTICK_ADROIT_CONSTANTS_H_

#define NUM_AXES        8

#define NAMED_BUTTON    0
#define JOINT_BUTTON    1
#define ENDPOINT_BUTTON 2
#define NUM_BUTTONS     3

#define MAX_VEL         0.3 // Only used for Endpoint at present.  Best to keep low!  Real max is for each actuator is 0.7854
#define VEL_SCALE       0.5 //just for joint-by-joint control.
#define LOOP_RATE       20.0
#define JOINT_SCALE     VEL_SCALE/LOOP_RATE // MAX_VEL*VEL_SCALE/LOOP_RATE
#define LINEAR_SCALE    0.3
#define ANGULAR_SCALE   MAX_VEL



#endif /* INCLUDE_HDT_GENERIC_JOYSTICK_ADROIT_CONSTANTS_H_ */
