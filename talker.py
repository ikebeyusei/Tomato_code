#!/usr/bin/env python
## coding: UTF-8

import rospy2 as rospy
from std_msgs.msg import String

def callback(message):
    rospy.loginfo("get message! "+ message.data) 

rospy.init_node('listener')
sub = rospy.Subscriber('chatter', String, callback) 
rospy.spin()
