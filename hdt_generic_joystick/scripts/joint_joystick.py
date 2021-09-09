#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

import wx

# duration between call backs for control loop (joystick to cmd conversion and publishing)
period = 1.0/10; 

pos_per_cycle = 0.01

global t_start
#state=0
state_null=0
state_joystick=1
state_pose=2
state_cycle=3


cycle_count=0
pos_num=0


class MainWindow(wx.Frame):
	def __init__(self,parent,title):
		wx.Frame.__init__(self,parent,title=title)
		self.Bind(wx.EVT_CLOSE, self.OnQuit)

		self.joint_names=[]
		self.joint_pos_telem=[]
		self.joint_pos_cmd={}
		self.poses=[]
		self.joystick_data_last=Joy()
		self.trigger_motion=[False,False]
		self.state=state_null
		rospy.init_node('joint_joystick', anonymous=True)
		rospy.on_shutdown(self.OnQuit)

		# read control set parameters
		if rospy.has_param('~control_set'):
			self.control_sets=rospy.get_param('~control_set')
		else:
			rospy.logerr("no control_set specified in parameters")
			self.OnQuit()
		rospy.loginfo("using control sets:")
		for control_set in self.control_sets:
			self.control_sets[control_set]=self.control_sets[control_set].split(',')
			rospy.loginfo("{} contains to {}".format(control_set,self.control_sets[control_set]))
		self.control_set_index=0

		# read joystick mapping from parameter
		if rospy.has_param('joystick_map'):
			self.joystick_map=rospy.get_param('joystick_map')
		else:
			rospy.logerr("no joystick map specified in parameters")
			self.OnQuit()
		rospy.loginfo("using joystick map:")
		self.publishers={}
		for joint in self.joystick_map:
			rospy.loginfo("{} maps to {}".format(joint,self.joystick_map[joint]))
			#TODO make the first part of this topic name a parameter
			self.publishers[joint] = rospy.Publisher('/hdt_arm/{}_position_controller/command'.format(joint), Float64, queue_size=10)
		
		# read button mapping from parameter
		if rospy.has_param('button_map'):
			self.button_map=rospy.get_param('button_map')
		else:
			rospy.logerr("no button map specified in parameters")
			self.OnQuit()		
		rospy.loginfo("using button map:")
		for action in self.button_map:
			rospy.loginfo("{} maps to {}".format(action,self.button_map[action]))
		
		# create subscriber for input device
		sub_joystick = rospy.Subscriber('joy', Joy, self.callback_joystick)

		# subscribe to joint state message
		#TODO make this topic name a parameter
		sub_telem = rospy.Subscriber('/joint_states',JointState, self.callback_telem)

		# create timer for publishing topics
        	self.publish_timer=wx.Timer(self,wx.ID_ANY)
        	self.Bind(wx.EVT_TIMER,self.PublishCtrl,self.publish_timer)
		# set rate for publishing control commands in miliseconds
        	self.publish_timer.Start(int(period*1000))

		self.mirror_mode=False

		# create master panel    
		panel = wx.Panel(self) 
		panel_sizer=wx.BoxSizer(wx.HORIZONTAL)

		# get background image of xbox controller
		#TODO fix this absolute path
		#image_file = '/home/hdt-global/catkin_ws/src/crsi_production/hdt_adroit_python/scripts/xbox_controller.jpg'
		#xbox_image = wx.Image(image_file,wx.BITMAP_TYPE_ANY)#.ConvertToBitmap()
		#self.imageCtrl = wx.StaticBitmap(panel,wx.ID_ANY,wx.BitmapFromImage(xbox_image))

		# A Statusbar in the bottom of the window
		self.CreateStatusBar()
		self.SetStatusText("status bar text")
		
		# create panel and sizer for buttons
		button_panel_left=wx.Panel(panel)
		button_sizer_left=wx.BoxSizer(wx.VERTICAL)
		button_panel_right=wx.Panel(panel)
		button_sizer_right=wx.BoxSizer(wx.VERTICAL)
		
		# create buttons
		self.clear_pose_button=wx.Button(button_panel_right, label="clear list of poses",style=wx.ALIGN_CENTRE)
		self.clear_pose_button.Bind(wx.EVT_BUTTON,self.ClearPoseButton)
		#self.power_button.SetBackgroundColour([200,200,200])
		button_sizer_right.Add(self.clear_pose_button,1,wx.EXPAND,10)
		
		self.add_pose_button=wx.Button(button_panel_right, label="add current pose to cycle list",style=wx.ALIGN_CENTRE)
		self.add_pose_button.Bind(wx.EVT_BUTTON,self.AddPoseButton)
		button_sizer_right.Add(self.add_pose_button,1,wx.EXPAND,10)
		
		self.scale_slider=wx.Slider(button_panel_left,value=100,minValue=0,maxValue=100,style=wx.SL_HORIZONTAL|wx.SL_LABELS)		
		self.scale_slider.Bind(wx.EVT_SCROLL, self.ScaleSlide)
		self.scale_slider_label=wx.StaticText(button_panel_left,label ="scale joystick = 100%")
		button_sizer_left.Add(self.scale_slider_label,1,wx.CENTER,20)
		button_sizer_left.Add(self.scale_slider,1,wx.EXPAND)
		self.scale_value=1

		# assemble panels and sizers
		button_panel_left.SetSizer(button_sizer_left)
		button_panel_right.SetSizer(button_sizer_right)
		panel_sizer.Add(button_panel_left,1,wx.EXPAND,10)
		# add image of xbox controller for extra feature selection
		#panel_sizer.Add(self.imageCtrl, 0, wx.ALL, 5)
		panel_sizer.Add(button_panel_right,1,wx.EXPAND,10)
		panel.SetSizer(panel_sizer)
		#self.Frame.SetSize((bmpl.GetWidth(),bmpl.GetHeight()))		
		self.Show(True)

	def ParameterRead(self, Drive, Index):
		#rospy.wait_for_service('/hdt_adroit_coms/read_drive_param')
		pass

	def ClearPoseButton(self, event):
		self.poses=[]
		if self.state==state_cycle:
			state=state_null
			print "returning to null state"

	def AddPoseButton(self, event):
		print "adding current pose to cycle list"
		self.poses.append({})
		for namespace in self.joint_names:
			self.poses[-1][namespace]=list(self.joint_pos_telem[namespace])

		for i in self.poses:
			print i

	def PublishCtrl(self,event):
		#global joint_pos_cmd
		global pos_num
		global cycle_count

		if(self.state==state_null):
			# do nothing right now
			pass
		elif(self.state==state_joystick):
			# loop through control sets
			#print("building joystick command")	
			control_set_name=self.control_sets.keys()[self.control_set_index]	
			for name in self.control_sets[control_set_name]:
				if name in self.joystick_map:
					joint_info=self.joystick_map[name]
					#print("\tfor drive: {} is type {}".format(name,type(joint_info)))
					if 'axisB' in joint_info:
						self.build_command(name,joint_info['input'],[joint_info['axisA'],joint_info['axisB']],joint_info['invert'])
					else:
						self.build_command(name,joint_info['input'],[joint_info['axisA']],joint_info['invert'])
			
			if self.mirror_mode:
				self.mirror_mode=False
				# don't have a definition for which joints to mirror yet
				pass
			if self.mirror_mode:
				for temp_index in range(len(self.joint_names[follow_set_name])):	
					self.joint_pos_cmd[follow_set_name][temp_index] = -self.joint_pos_telem[control_set_name][temp_index]
					self.joint_vel_cmd[follow_set_name][temp_index] = velocity_int*self.scale_value*1.0
				# assume gripper is last element in list, don't invert position command
				self.joint_pos_cmd[follow_set_name][-1] = self.joint_pos_telem[control_set_name][-1]
					

		elif(self.state==state_cycle):
			#print("calcualting distance to target pose")
			distance=0
			for namespace in self.namespaces:
				for i in range(0,len(self.joint_pos_cmd[namespace])):
					if not(math.isnan(self.poses[pos_num][namespace][i])):
						self.joint_pos_cmd[namespace][i]=self.poses[pos_num][namespace][i]
						distance+=pow(self.joint_pos_telem[namespace][i]-self.poses[pos_num][namespace][i],2)
					else:
						self.joint_pos_cmd[namespace][i]=self.joint_pos_telem[namespace][i]
				self.joint_vel_cmd[namespace]=[velocity_max*self.scale_value]*len(self.joint_pos_telem[namespace])

			if distance<0.01:
				pos_num=(pos_num+1) % len(self.poses)
				if pos_num==0:
					cycle_count=cycle_count+1
				s="on pose " +repr(pos_num+1) +"/" + repr(len(self.poses)) + " of cycle "+repr(cycle_count)
				print s
				cycle_msg=Int16()
				cycle_msg.data=cycle_count
				self.pub_count.publish(cycle_msg)
		

		elif(self.state==state_pose):
			# going to position set in callback_joystick
			pass
		if(self.state!=state_null):
			# loop through publishers
			for joint in self.joystick_map:
				msg = Float64()
				msg.data = self.joint_pos_cmd[joint]
				#print("cmd {} to {} radians".format(joint,msg.data))
				self.publishers[joint].publish(msg)

	def build_command(self,joint_name,input_type,channel,invert):
		# read through list of joint names
		for namespace in self.joint_names:
			if joint_name in self.joint_names:
				#temp_index=self.joint_names.index(joint_name)
				if input_type == "axes":
					if len(channel)==1:
						joystick_value = self.joystick_data.axes[channel[0]]
					if len(channel)==2:
						joystick_value = self.joystick_data.axes[channel[0]]-self.joystick_data.axes[channel[1]]
				if input_type == "buttons":
					if len(channel)==1:
						joystick_value = self.joystick_data.buttons[channel[0]]
					if len(channel)==2:
						joystick_value = self.joystick_data.buttons[channel[0]]-self.joystick_data.buttons[channel[1]]
				if invert:
					joystick_value = -joystick_value	
				# use +-2*pi as possition command, control drive with velocity command	
				if joystick_value==0:
					#self.joint_pos_cmd[joint_name] = self.joint_pos_telem[self.joint_names.index(joint_name)]
					pass
				else:
					self.joint_pos_cmd[joint_name] = self.joint_pos_cmd[joint_name]+pos_per_cycle*self.scale_value*joystick_value

				
	def callback_telem(self,telem_data):
		self.joint_names=telem_data.name
		self.joint_pos_telem=telem_data.position

	def callback_joystick(self,joystick_message):
		#global joint_pos_cmd
		#global joint_pos_telem
		#global joystick_data
		#global state
		global velocity_max		
	 	self.joystick_data = joystick_message

		if joystick_message.axes[2]!=0:
			# axis 2 is left, at rest value is 1 but shows up as 0 until pressed
			self.trigger_motion[0]=True
		if joystick_message.axes[5]!=0:
			# axis 5 is right, at rest value is 1 but shows up as 0 until pressed
			self.trigger_motion[1]=True
		if not self.trigger_motion[0]:
			temp=list(self.joystick_data.axes)
			temp[2]=1
			self.joystick_data.axes=temp
		if not self.trigger_motion[1]:
			temp=list(self.joystick_data.axes)
			temp[5]=1
			self.joystick_data.axes=temp

		# assume buttons for now, though button map can include axis (also "invert" tag)
		if self.joystick_data.buttons!=self.joystick_data_last.buttons:
			# handle button to stop sending commands to actuators
			temp_index = self.button_map["stop_control"]['axisA']
			if(self.joystick_data.buttons[temp_index]):
				print "stop control"
				self.state=state_null
				for joint in self.joint_pos_cmd:
					self.joint_pos_cmd[joint]=self.joint_pos_telem[self.joint_names.index(joint)]
					
				# make sure mirror mode is off, to avoid surprises
				self.mirror_mode=False
			# handle button to toggle mirror mode
			temp_index = self.button_map["mirror_mode"]['axisA']
			if(self.joystick_data.buttons[temp_index]):
				self.mirror_mode = not(self.mirror_mode)
				if self.mirror_mode:
					print "mirror mode ON"
				else:
					
					print "mirror mode OFF"
			# handle button to enable joystick control
			temp_index = self.button_map["joystick_control"]['axisA']
			if(self.joystick_data.buttons[temp_index]):
				#make sure telemetry positions for all drives have shown up since we're using positon control
				joints_found=True
				for joint in self.joystick_map:
					if joint in self.joint_names:
						self.joint_pos_cmd[joint] = self.joint_pos_telem[self.joint_names.index(joint)]
						pass
					else:
						joints_found=False
						rospy.logerr("missing telemetry for {}".format(joint))
				
				if joints_found:
					print "joystick control"
					self.state=state_joystick
					control_set_name=self.control_sets.keys()[self.control_set_index]
					print "controlling drives {}".format(self.control_sets[control_set_name])
				else:
					rospy.logerr("have not recieved telemetry from all joints yet")

			# handle button to enable cycling control
			temp_index = self.button_map["cycle_control"]['axisA']
			if(self.joystick_data.buttons[temp_index]):
				print(self.poses)
				if len(self.poses)>0:
					print "cycling control - starting"
					self.state = state_cycle
					for joint in self.joystick_map:
						self.joint_pos_cmd[joint] = self.joint_pos_telem[self.joint_names.index(joint)]
				else:
					print "cycling control - no poses in cycle list"

			# handle button to toggle commanded velocity
			temp_index = self.button_map["change_speed"]['axisA']
			if(self.joystick_data.buttons[temp_index]):
				print "switch speed"
				if self.scale_value<1:
					self.scale_value = 1
				else:
					self.scale_value = 0.2
				print "\tvelocity scale = {0:3.0f}%".format(self.scale_value*100)

			# handle button to send all drives to zero position
			temp_index = self.button_map["zero_position"]['axisA']
			if(self.joystick_data.buttons[temp_index]):
				print "go to zero position"
				self.state = state_pose
				for joint in self.joystick_map:
					self.joint_pos_cmd[joint] = 0.0
				
			# handle button to change drive set being controller
			temp_index = self.button_map["change_drive_set"]['axisA']
			if(self.joystick_data.buttons[temp_index]):
				print "change drive set"
				self.control_set_index = self.control_set_index+ 1
				if self.control_set_index >= len(self.control_sets):
					self.control_set_index = 0
				control_set_name=self.control_sets.keys()[self.control_set_index]
				print("set {} contains drives {}".format(control_set_name,self.control_sets[control_set_name]))
				# clear velocity commands for all drives to avoid issues with drift
				for joint in self.joystick_map:
					self.joint_pos_cmd[joint] = self.joint_pos_telem[self.joint_names.index(joint)]
				
		self.joystick_data_last=self.joystick_data

	def ScaleSlide(self,event):
		# slider to adjust scaling on joystick
		self.scale_value = float(event.GetEventObject().GetValue())/100
		self.scale_slider_label.SetLabel("joystick scale = {0:3.0f}%".format(self.scale_value))

	def OnQuit(self):
		print "shutting down"
		self.Destroy()

if __name__ == '__main__':

	app = wx.App()
	frame = MainWindow(None,"extra controls for joystick")
	app.MainLoop()

