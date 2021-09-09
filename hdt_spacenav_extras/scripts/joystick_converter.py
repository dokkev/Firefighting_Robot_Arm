#!/usr/bin/env python
import rospy
import threading
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

joint_state = JointState()

def joystick_callback(msg):
  #print(msg)
  global pub_joint_state
  global joint_state
  joint_state.velocity[0]=0.0
  if msg.buttons[12]:
    joint_state.velocity[0]=0.5
  if msg.buttons[13]:
    joint_state.velocity[0]=0.1
  if msg.buttons[14]:
    joint_state.velocity[0]=-0.1
  if msg.buttons[15]:
    joint_state.velocity[0]=-0.5
    
  pub_joint_state.publish(joint_state)
   
if __name__ == '__main__':
  rospy.init_node('spacenav_reader', anonymous=True)
  sub_joy = rospy.Subscriber('spacenav_joy',Joy,joystick_callback)
  global pub_joint_state
  pub_joint_state = rospy.Publisher('joint_joy',JointState,queue_size=10)
  global joint_state
  joint_state = JointState()
  joint_state.name=["leftpincer"]
  joint_state.position=[0]*len(joint_state.name)
  joint_state.velocity=[0]*len(joint_state.name)
  joint_state.effort=[0]*len(joint_state.name)


  while not rospy.is_shutdown():
    pass
'''
  pub_twist = rospy.Publisher('spacenav_twist',Twist,queue_size=10)
  spnav_open()
  msg = Joy()
  #msg.buttons=[0]*num_buttons
  msg.buttons=[0]*num_buttons_pro
  msg.axes=[0.0]*num_axes
  msg_twist = Twist()  
  while not rospy.is_shutdown():
    # check if an event is avalible, instead of blocking spnav_wait_event()
    event = spnav_poll_event()
    if event != None:
      # handle joystick events
      if event.ev_type == 1:
        #print("joystick")
        #print(event.rotation)  #-y,z,x
        #print(event.translation)  #-y,z,x
        
        # populate joystick axes as linear x, y ,x, angular x, y, z
        msg.axes[0]=event.translation[2]
        msg.axes[1]=-event.translation[0]
        msg.axes[2]=event.translation[1]
        msg.axes[3]=event.rotation[2]
        msg.axes[4]=-event.rotation[0]
        msg.axes[5]=event.rotation[1]
        pub_joy.publish(msg)
        
        # populate twist message
        msg_twist.linear.x = event.translation[2]
        msg_twist.linear.y = -event.translation[0]
        msg_twist.linear.z = event.translation[1]
        msg_twist.angular.x = event.rotation[2]
        msg_twist.angular.y = -event.rotation[0]
        msg_twist.angular.z = event.rotation[1]
        pub_twist.publish(msg_twist)
        
      # handle button events
      if event.ev_type == 2:
        print("button {} is {}".format(event.bnum,event.press))
        #print(event.bnum) # button id
        #print(event.press) # true for press, false for release
        
        # resize button array for the spavenav pro
        if event.bnum >= len(msg.buttons):
          while num_buttons_pro >= len(msg.buttons):
            msg.buttons.append(0)
        if event.press:
          msg.buttons[event.bnum]=1
        else:
          msg.buttons[event.bnum]=0
        pub_joy.publish(msg)
  spnav_close()
'''
