#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from sensor_msgs.msg import Joy

modeCounter = 1
wasButtonPush = False

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
        smach.State.__init__(self, outcomes=['done','doing'])

    def execute(self, userdata):
        if modeCounter == 2:
            time.sleep(1)
            return 'done'
	else:
	    return 'doing'

class Harvest_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'doing'])

    def execute(self, userdata):
        if modeCounter == 2:
            time.sleep(1)
            return 'done'
        else:
            return 'doing'

class Transport_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'doing'])

    def execute(self, userdata):
        if modeCounter == 2:
            time.sleep(1)
            return 'done'
        else:
            return 'doing'

def main():
    sm = smach.StateMachine(outcomes=['exit'])

    with sm:
        init_sub = smach.StateMachine(outcomes=['mode_finish'])
        with init_sub:
            smach.StateMachine.add('Stand_By',Stand_by_mode(), transitions={'done':'mode_finish' , 'doing':'Stand_By'})

        smach.StateMachine.add('STAND_BY_MODE',init_sub, transitions={'mode_finish':'SEARCH_MODE'})

        search_sub = smach.StateMachine(outcomes=['mode_finish'])

        with search_sub:
            smach.StateMachine.add('Search',Search_mode(),transitions={'done':'TRANSPORT_MODE','doing':'Search'})
            transport_sub = smach.StateMachine(outcomes=['mode_finish'])
            with transport_sub:
                smach.StateMachine.add('Transport',Transport_mode(), transitions={'done':'mode_finish', 'doing':'Transport'})
            smach.StateMachine.add('TRANSPORT_MODE',transport_sub, transitions={'mode_finish':'mode_finish'})
        smach.StateMachine.add('SEARCH_MODE',search_sub, transitions={'mode_finish':'HARVEST_MODE'})

        harvest_sub = smach.StateMachine(outcomes=['mode_finish'])
        with harvest_sub:
            smach.StateMachine.add('Harvest',Harvest_mode(),transitions={'done':'mode_finish' , 'doing':'Harvest'})

        smach.StateMachine.add('HARVEST_MODE',harvest_sub, transitions={'mode_finish':'SEARCH_MODE'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('action_selection_smach')
    joy_twist = JoyTwist()
    main()
    rospy.spin()
