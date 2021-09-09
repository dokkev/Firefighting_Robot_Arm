#!/usr/bin/env python
import rospy
from can_msgs.msg import Frame


def received_callback(msg):
  msg.id
  print("recieved this")
  print(msg)
    
   
if __name__ == '__main__':
  rospy.init_node('drive_controller', anonymous=True)
  
  sub_joy = rospy.Subscriber('received_messages',Frame,received_callback)
  pub_joint_state = rospy.Publisher('sent_messages',Frame,queue_size=10)
  


  while not rospy.is_shutdown():
    pass
