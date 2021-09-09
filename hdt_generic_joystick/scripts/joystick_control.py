#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

from controller_manager_msgs.srv import LoadController, SwitchController

from threading import Lock

class JoystickControl:
  MODE_NONE = 0
  MODE_NAMED = 1
  MODE_JOINT = 2
  NUM_MODES = 3

  NAMED_BUTTON = 0
  JOINT_BUTTON = 1
  NUM_BUTTONS = 2

  JOINT_AXES = {}
  JOINT_AXES['drive1_joint'] = 2
  JOINT_AXES['drive2_joint'] = 3
  JOINT_AXES['drive3_joint'] = 1
  JOINT_AXES['drive4_joint'] = 5
  JOINT_AXES['drive5_joint'] = 0
  JOINT_AXES['drive6_joint'] = 4
  JOINT_AXES['pincer_joint'] = 6
  NUM_AXES = 8
  
  MAX_VEL = 0.7854
  VEL_SCALE = 0.4
  LOOP_RATE = 20.0
  JOY_SCALE = MAX_VEL*VEL_SCALE/LOOP_RATE
  JOY_TOPIC = '/hdt_joystick/joy'
  
  TRAJ_CONTROLLER = '/hdt_arm/arm_controller'

  def __init__(self):
    self.mode = self.MODE_NONE

    self.joy = Joy()
    self.mutex = Lock()
    
    self.joy.axes = [0.0]*self.NUM_AXES
    self.joy.buttons = [0]*self.NUM_BUTTONS
    self.joy_prev = copy.deepcopy(self.joy)
    rospy.Subscriber('/joy', Joy, self.callback)
    self.joy_pub = rospy.Publisher(self.JOY_TOPIC, Joy, queue_size=1)
    
    self.group = moveit_commander.MoveGroupCommander("arm")
    self.group.set_max_velocity_scaling_factor(self.VEL_SCALE)
    
    self.frame_id = self.group.get_planning_frame()
    self.joint_names = self.group.get_active_joints()
    #self.joint_names.append("pincer_joint") # an alternative way to include pincer
    self.joint_values = [0.0]*len(self.joint_names)
    self.pincer_value = 0.0
    
    self.joint_controllers = []
    self.joint_pubs = {}
    for joint_name in self.joint_names:
      # add to controller list
      controller = '/hdt_arm/' + joint_name + '_position_controller'
      self.joint_controllers.append(controller)
      
      # add to pub list
      topic = controller + '/command'
      pub = rospy.Publisher(topic, Float64, queue_size=1)
      self.joint_pubs[joint_name] = pub
    
    # add pincer control
    # add to controller list
    controller = '/hdt_arm/pincer_joint_position_controller'
    self.joint_controllers.append(controller)
    # add to pub list
    topic = controller + '/command'
    pub = rospy.Publisher(topic, Float64, queue_size=1)
    self.joint_pubs['pincer_joint'] = pub
    
    # load joint controllers
    rospy.wait_for_service('/controller_manager/load_controller')
    for controller in self.joint_controllers:
      load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
      try:
        res = load_controller(controller)
        
        if not res.ok:
          rospy.logerr("could not load controller %s" % controller)
          return
      except rospy.ServiceException, e:
        rospy.logerr("service call failed: %s" % e)
        return

  def switch(self, new):
    # check for valid state 
    if new < self.MODE_NONE or new >= self.NUM_MODES:
      rospy.logwarn("invalid mode: %d" % new)
      return self.mode
    
    start_controllers = []
    stop_controllers = []
    strictness = SwitchController._request_class.STRICT

    # if switching to joint, enable joint controller
    if new == self.MODE_JOINT:
      # get current state
      self.joint_values = self.group.get_current_joint_values()
    
      start_controllers.extend(self.joint_controllers)
      stop_controllers.append(self.TRAJ_CONTROLLER)
    # if switching from joint, disable joint controller
    elif self.mode == self.MODE_JOINT:
      start_controllers.append(self.TRAJ_CONTROLLER)
      stop_controllers.extend(self.joint_controllers)
    
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    try:
      res = switch_controller(start_controllers, stop_controllers, strictness)

      if res.ok:
        rospy.loginfo("switching from mode %d to %d" % (self.mode, new))
        return new
    except rospy.ServiceException, e:
      rospy.logerr("service call failed: %s" % e)
    
    return self.mode

  def callback(self, msg):    
    # acquire mutex
    self.mutex.acquire()
    try:
      # copy msg to joy
      #self.joy = copy.deepcopy(msg)
      self.joy.axes[0] = msg.axes[0]
      self.joy.axes[1] = msg.axes[1]
      self.joy.axes[2] = msg.axes[3]
      self.joy.axes[3] = msg.axes[4]
      self.joy.axes[4] = (msg.axes[2] - msg.axes[5])/2.0
      self.joy.axes[5] = msg.buttons[5] - msg.buttons[4]
      self.joy.axes[6] = msg.axes[7]
      #self.joy.axes[7] = msg.axes[7]
      self.joy.buttons[0] = msg.buttons[0]
      self.joy.buttons[1] = msg.buttons[1]
    finally:
      self.mutex.release()

  def run(self):    
    r = rospy.Rate(self.LOOP_RATE)
    while not rospy.is_shutdown():
      # acquire mutex
      self.mutex.acquire()
      try:
        # check for button transitions
        if self.joy.buttons[self.NAMED_BUTTON] == 0 and self.joy_prev.buttons[self.NAMED_BUTTON] == 1:
          # set named mode and name
          self.mode = self.switch(self.MODE_NAMED)
          name = "home"
        elif self.joy.buttons[self.JOINT_BUTTON] == 0 and self.joy_prev.buttons[self.JOINT_BUTTON] == 1:
          # toggle joint mode
          if self.mode == self.MODE_NONE:
            self.mode = self.switch(self.MODE_JOINT)
          elif self.mode == self.MODE_JOINT:
            self.mode = self.switch(self.MODE_NONE)
         
        # update joy_prev
        self.joy_prev = copy.deepcopy(self.joy)
      finally:
        self.mutex.release()
      
      # take mode-dependent action
      if self.mode == self.MODE_NAMED:
        self.group.set_named_target(name)
        plan = self.group.plan()
        self.group.go(wait=True)
        self.mode = self.switch(self.MODE_NONE)
      elif self.mode == self.MODE_JOINT:      
        # iterate through active joints
        for i in range(len(self.joint_names)):
          # increment joint values
          joint_name = self.joint_names[i]
          axis = self.JOINT_AXES[joint_name]
          self.joint_values[i] += self.joy_prev.axes[axis]*self.JOY_SCALE
          
          # send updated joint command
          joint_pub = self.joint_pubs[joint_name]
          command = Float64()
          command.data = self.joint_values[i]
          joint_pub.publish(command)
         
        # Pincer controller  
        axis = self.JOINT_AXES['pincer_joint']
        self.pincer_value += self.joy_prev.axes[axis]*self.JOY_SCALE
        # send updated joint command
        joint_pub = self.joint_pubs['pincer_joint']
        command = Float64()
        command.data = self.pincer_value
        joint_pub.publish(command)
        
      
      # publish joy msg
      self.joy_pub.publish(self.joy)
      
      r.sleep()
    
if __name__=='__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('joystick_control', anonymous=True)
  try:
    joystick_control = JoystickControl()
    joystick_control.run()
  except rospy.ROSInterruptException:
    pass
