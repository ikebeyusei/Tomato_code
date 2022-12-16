#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from sensor_msgs.msg import Joy

modeCounter = 1
wasButtonPush = False
detection_flag = False
target_pos_flag = False

axis_mode = 0
arm_mode = 0
gripper_mode = 0
vehicle_distance = 0

class JoyTwist(object):
	def __init__(self):
		self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

	def joy_callback(self, joy_msg):
		global modeCounter
		global wasButtonPush
		if joy_msg.buttons[16] == 1:
			if wasButtonPush == 0:
				modeCounter = modeCounter + 1
				wasButtonPush = True
		else:
			wasButtonPush = False

		if modeCounter == 3:
			modeCounter = 1

class Stand_by_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'doing'])

    def execute(self, userdata):
        if modeCounter == 2:
            time.sleep(1)
            return 'done'
        else:
            return 'doing'

class Search_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','exit'])

    def execute(self, userdata):
        if modeCounter == 2:
            time.sleep(1)
            return 'done'
        else:
            return 'exit' 

class Harvest_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done' , 'exit'])

    def execute(self, userdata):
        if modeCounter == 2:
            time.sleep(1)
            return 'done'
        else:
            return 'exit' 

class Transport_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])

    def execute(self, userdata):
        if modeCounter == 2:
            time.sleep(1)
            return 'done'
        else:
            return 'exit' 

class Detection_tomato(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['true', 'false', 'exit'])

    def execute(self,userdata):
        if modeCounter == 1:
            return 'exit'
        else:
            global detection_flag
            detection_flag = False
            time.sleep(1)
            detection_flag = True  # tomato_is_found
            if detection_flag == True:
                return 'true'
            else:
                return 'false'

class Target_pos_tomato(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['true','false', 'exit'])

    def execute(self,userdata):
        if modeCounter == 1:
            return 'exit'
        else:
            global target_pos_flag
            target_pos_flag = False
            time.sleep(1)
            target_pos_flag = True
            if target_pos_flag == True:
                return 'true'
            else:
                return 'false'

class Axis_mode(smach.State):
    def __init__(self):
        global axis_mode
        axis_mode = 0
        smach.State.__init__(self,outcomes=['done','exit'])

    def execute(self,userdata):
        if modeCounter == 1:
            return 'exit'
        else:
            time.sleep(1)
            axis_mode = 1
            time.sleep(1)
            axis_mode = 2
            return 'done'

class Gripper_mode(smach.State):
    def __init__(self):
        global gripper_mode
        gripper_mode = 0
        smach.State.__init__(self,outcomes=['done','exit'])

    def execute(self,userdata):
        if modeCounter == 1:
            return 'exit'
        else:
            time.sleep(1)
            gripper_mode = 1
            return 'done'

class Arm_mode(smach.State):
    def __init__(self):
        global arm_mode
        arm_mode = 0
        smach.State.__init__(self,outcomes=['done','exit'])

    def execute(self,userdata):
        if modeCounter == 1:
            return 'exit'
        else:
            time.sleep(1)
            arm_mode = 1
            return 'done'

def main():
    sm = smach.StateMachine(outcomes=[''])

    with sm:
        init_sub = smach.StateMachine(outcomes=['mode_finish'])
        with init_sub:
            smach.StateMachine.add('Stand_By',Stand_by_mode(), transitions={'done':'mode_finish' , 'doing':'Stand_By'})

        smach.StateMachine.add('STAND_BY_MODE',init_sub, transitions={'mode_finish':'SEARCH_MODE'})

        search_sub = smach.StateMachine(outcomes=['mode_finish','exit'])
        with search_sub:
            smach.StateMachine.add('Search',Search_mode(),transitions={'done':'detection_flag','exit':'exit'})
            smach.StateMachine.add('detection_flag', Detection_tomato(),transitions={'true':'target_pos_flag','false':'detection_flag','exit':'exit'})
            smach.StateMachine.add('target_pos_flag', Target_pos_tomato(),transitions={'true':'mode_finish','false':'target_pos_flag','exit':'exit'})
        smach.StateMachine.add('SEARCH_MODE',search_sub, transitions={'mode_finish':'HARVEST_MODE','exit':'STAND_BY_MODE'})

        transport_sub = smach.StateMachine(outcomes=['mode_finish','exit'])
        with transport_sub:
                smach.StateMachine.add('Transport',Transport_mode(), transitions={'done':'mode_finish','exit':'exit'})
        smach.StateMachine.add('TRANSPORT_MODE',transport_sub, transitions={'mode_finish':'SEARCH_MODE','exit':'STAND_BY_MODE'})

        harvest_sub = smach.StateMachine(outcomes=['mode_finish','exit'])
        with harvest_sub:
            smach.StateMachine.add('Harvest',Harvest_mode(),transitions={'done':'Axis_mode' ,'exit':'exit'})
            smach.StateMachine.add('Axis_mode',Axis_mode(), transitions={'done':'Arm_mode', 'exit':'exit'})
            smach.StateMachine.add('Arm_mode', Arm_mode(), transitions={'done':'Gripper_mode', 'exit':'exit'})
            smach.StateMachine.add('Gripper_mode',Gripper_mode(),transitions={'done':'mode_finish', 'exit':'exit'})
        smach.StateMachine.add('HARVEST_MODE',harvest_sub, transitions={'mode_finish':'TRANSPORT_MODE','exit':'STAND_BY_MODE'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('action_selection_smach')
    joy_twist = JoyTwist()
    main()
    rospy.spin()
