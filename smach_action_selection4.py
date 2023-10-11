#!/usr/bin/env/python

import rospy
import smach
import smach_ros
import time
from sensor_msgs.msg import Joy

global madeCounter
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

        if modeCounter == 8:
            modeCounter = 1


class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['move_forward','step_back','turn_left','turn_right','move_to_the_right','move_to_the_left','move_finish'])

    def execute(self,userdata):
        global robot_mode
        robot_mode = 0
        if modeCounter == 1:
            return 'move_forward'
        elif modeCounter == 2:
            return 'step_back'
        elif modeCounter == 3:
            return 'turn_left'
        elif modeCounter == 4:
            return 'turn_right'
        elif modeCounter == 5:
            return 'move_to_the_right'
        elif modeCounter == 6:
            return 'move_to_the_left'
        else:
            return 'move_finish '
        
        
class Move_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop','exit'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'stop'

class Step_back(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop','exit'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'stop'

class Turn_left(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop','exit'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'stop'
        
class Turn_right(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop','exit'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'stop'
    
class Move_to_the_right(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop','exit'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'stop'
    
class Move_to_the_left(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop','exit'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'stop'

class Mode_finish(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['mode_finish','exit'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'mode_finish'

#arm!!!!
class Arm_init(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['done'])
    
    def execute(self,userdata):
        return 'done'

class Arm_open(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['done'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'done'

class Arm_close(smach.State):
    def __init__(self):
            smach.State.__init__(self,outcomes=['done'])

    def execute(self,userdata):
        time.sleep(1.0)
        return 'done'

class Line_tracing(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['detection','not_detected'])

    def execute(self,userdata):
        global detection_line
        detection_line = True
        time.sleep(1.0)
        detection_line = False
        if detection_line == True:
            return 'detection'
        else:
            return 'not_detected'

    
def main():
    sm = smach.StateMachine(outcomes=[''])

    with sm:
        init_sub = smach.StateMachine(outcomes=['Not','doing'])
        with init_sub:
            smach.StateMachine.add('Line_Tracing',Line_tracing(),transitions={'detection':'Line_Tracing','not_detected':'Not'})
        smach.StateMachine.add('LINE_TRACING',init_sub,transitions={'Not':'MOVE_MODE','doing':'LINE_TRACING'})

        move_sub = smach.StateMachine(outcomes = ['mode_finish','exit'])
        with move_sub:
            smach.StateMachine.add('Stop!',Stop(),transitions={'move_forward':'Move_Forward','step_back':'Step_Back','turn_left':'Turn_Left','turn_right':'Turn_Right','move_to_the_right':'Move_To_The_Right','move_to_the_left':'Move_To_The_Left','move_finish':'mode_finish'})
            smach.StateMachine.add('Move_Forward',Move_forward(),transitions={'stop':'Stop!','exit':'exit'})
            smach.StateMachine.add('Step_Back',Step_back(),transitions={'stop':'Stop!','exit':'exit'})
            smach.StateMachine.add('Turn_Left',Turn_left(),transitions={'stop':'Stop!','exit':'exit'})
            smach.StateMachine.add('Turn_Right',Turn_right(),transitions={'stop':'Stop!','exit':'exit'})
            smach.StateMachine.add('Move_To_The_Right',Move_to_the_right(),transitions={'stop':'Stop!','exit':'exit'})
            smach.StateMachine.add('Move_To_The_Left',Move_to_the_left(),transitions={'stop':'Stop!','exit':'exit'})
        smach.StateMachine.add('MOVE_MODE',move_sub, transitions={'mode_finish':'ARM_MODE','exit':'MOVE_MODE'})

        arm_sub = smach.StateMachine(outcomes = {'mode_finish'})
        with arm_sub:
            smach.StateMachine.add('Arm_Init',Arm_init(),transitions={'done':'Arm_Open'})
            smach.StateMachine.add('Arm_Open',Arm_open(),transitions={'done':'Arm_Close'})
            smach.StateMachine.add('Arm_Close',Arm_close(),transitions={'done':'mode_finish'})
        smach.StateMachine.add('ARM_MODE',arm_sub,transitions={'mode_finish':'MOVE_MODE'})
        
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('action_selection_smach')
    joy_twist = JoyTwist()
    main()
    rospy.spin()
