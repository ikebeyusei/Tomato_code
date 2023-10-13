#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from sensor_msgs.msg import Joy

modeCounter = 0
wasButtonPush = False
controlCounter = 0
wasButtonPush2 = False
global str_moveMode
str_moveMode = 'stop'


class JoyTwist(object):
	def __init__(self):
		self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

	def joy_callback(self, joy_msg):
         global modeCounter
         global wasButtonPush
         global controlCounter
         global wasButtonPush2
         if joy_msg.buttons[16] == 1:
            if wasButtonPush == 0:
                modeCounter = modeCounter + 1
                wasButtonPush = True
            else:
                wasButtonPush = False

         if joy_msg.buttons[3] == 1:
            if controlCounter == 0:
                controlCounter = controlCounter + 1
                wasButtonPush2 = True
            else:
                wasButtonPush2 = False

        if modeCounter == 8:
            modeCounter = 1

class Stand_by_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'doing'])

    def execute(self, userdata):
        if modeCounter == 1:
            return 'done'
        else:
            return 'doing'

class Stand_by_finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['select_push_1','select_push_2'])

    def execute(self, userdata):
        global controlCounter
        if controlCounter == 1:
            time.sleep(1)
            return 'select_push_1'
        else:
            return 'select_push_2' 

class Stop_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[
            'done' ,
            'doing',
            'right',
            'front',
            'behind',
            'left',
            'exit'
        ])

    def execute(self, userdata):
        if str_moveMode == 'stop':
            return 'doing'
        elif str_moveMode == 'front':
            return 'front'
        elif str_moveMode == 'back':
            return 'behind'
        elif str_moveMode == 'right':
            return 'right'
        elif str_moveMode == 'left':
            return 'left'
        elif str_moveMode == 'arm_init':
            return 'done'
        elif controlCounter == 2:
            return 'exit'

class Move_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','doing','exit'])

    def execute(self, userdata):
        if str_moveMode == 'front':
            return 'doing'
        elif str_moveMode == 'stop':
            return 'done'
        elif controlCounter == 2:
            return 'exit' 

class Right_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','doing','exit'])

    def execute(self, userdata):
        if str_moveMode == 'right':
            return 'doing'
        elif str_moveMode == 'stop':
            return 'done'
        elif controlCounter == 2:
            return 'exit' 

class Back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','doing','exit'])

    def execute(self, userdata):
        if str_moveMode == 'back':
            return 'doing'
        elif str_moveMode == 'stop':
            return 'done'
        elif controlCounter == 2:
            return 'exit' 

class Left_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','doing','exit'])

    def execute(self, userdata):
        if str_moveMode == 'left':
            return 'doing'
        elif str_moveMode == 'stop':
            return 'done'
        elif controlCounter == 2:
            return 'exit' 

class Arm_init(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['arm_open','arm_close','doing','exit'])

    def execute(self,userdata):
        if str_moveMode == 'arm_init':
            return 'doing'
        elif str_moveMode == 'arm_open':
            return 'arm_open'
        elif str_moveMode == 'arm_close':
            return 'arm_close'
        elif controlCounter == 2:
            return 'exit'

class Arm_open(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['done','doing','exit','arm_init'])

    def execute(self,userdata):
        if str_moveMode == 'arm_init':
            return 'arm_init'
        elif str_moveMode == 'arm_open':
            return 'doing'
        elif str_moveMode == 'arm_close':
            return 'done'
        elif controlCounter == 2:
            return 'exit'

class Arm_close(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['done','doing','exit','arm_init'])

    def execute(self,userdata):
        if str_moveMode == 'arm_init':
            return 'arm_init'
        elif str_moveMode == 'arm_open':
            return 'done'
        elif str_moveMode == 'arm_close':
            return 'doing'
        elif controlCounter == 2:
            return 'exit'

class Manual_stop_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[
            'done' ,
            'doing',
            'right',
            'front',
            'behind',
            'left',
            'exit'
        ])

    def execute(self, userdata):
        if str_moveMode == 'stop':
            return 'doing'
        elif str_moveMode == 'front':
            return 'front'
        elif str_moveMode == 'back':
            return 'behind'
        elif str_moveMode == 'right':
            return 'right'
        elif str_moveMode == 'left':
            return 'left'
        elif str_moveMode == 'arm_init':
            return 'done'
        elif controlCounter == 1:
            return 'exit'

class Manual_move_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','doing','exit'])

    def execute(self, userdata):
        if str_moveMode == 'front':
            return 'doing'
        elif str_moveMode == 'stop':
            return 'done'
        elif controlCounter == 1:
            return 'exit' 

class Manual_right_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','doing','exit'])

    def execute(self, userdata):
        if str_moveMode == 'right':
            return 'doing'
        elif str_moveMode == 'stop':
            return 'done'
        elif controlCounter == 1:
            return 'exit' 

class Manual_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','doing','exit'])

    def execute(self, userdata):
        if str_moveMode == 'back':
            return 'doing'
        elif str_moveMode == 'stop':
            return 'done'
        elif controlCounter == 1:
            return 'exit' 

class Manual_left_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','doing','exit'])

    def execute(self, userdata):
        if str_moveMode == 'left':
            return 'doing'
        elif str_moveMode == 'stop':
            return 'done'
        elif controlCounter == 1:
            return 'exit' 

def main():
    sm = smach.StateMachine(outcomes=[''])

    with sm:
        init_sub = smach.StateMachine(outcomes=['1','2'])
        with init_sub:
            smach.StateMachine.add('Stand_By_Mode',Stand_by_mode(), transitions={'done':'Stand_By_Finish' , 'doing':'Stand_By_Mode'})
            smach.StateMachine.add('Stand_By_Finish',Stand_by_finish(), transitions={'select_push_1':'1' , 'select_push_2':'2'})
        smach.StateMachine.add('STAND_BY_MODE',init_sub, transitions={'1':'MOVE_MODE','2':'MANUAL_CONTROL_MODE'})

        move_mode_sub_sub = smach.StateMachine(outcomes=['mode_finish','exit'])
        with move_mode_sub_sub:
            smach.StateMachine.add('Stop_Mode',Stop_mode(),transitions={'done':'mode_finish','exit':'exit','doing':'Stop_Mode','right':'Right_Move','front':'Move_Forward','behind':'BACK','left':'Left_Move'})
            smach.StateMachine.add('Move_Forward', Move_forward(),transitions={'done':'Stop_Mode','exit':'exit','doing':'Move_Forward'})
            smach.StateMachine.add('Right_Move',Right_move(),transitions={'done':'Stop_Mode','exit':'exit','doing':'Right_Move'})
            smach.StateMachine.add('BACK', Back(),transitions={'done':'Stop_Mode','exit':'exit','doing':'BACK'})
            smach.StateMachine.add('Left_Move', Left_move(),transitions={'done':'Stop_Mode','exit':'exit','doing':'Left_Move'})
        smach.StateMachine.add('MOVE_MODE',move_mode_sub_sub, transitions={'mode_finish':'ARM_MODE','exit':'STAND_BY_MODE'})

        arm_sub = smach.StateMachine(outcomes=['exit'])
        with arm_sub:
            smach.StateMachine.add('Arm_Init',Arm_init(), transitions={'arm_open':'Arm_Open','arm_close':'Arm_Close','exit':'exit','doing':'Arm_Init'})
            smach.StateMachine.add('Arm_Open',Arm_open(), transitions={'arm_init':'Arm_Init','exit':'exit','doing':'Arm_Open','done':'Arm_Init'})
            smach.StateMachine.add('Arm_Close',Arm_close(), transitions={'arm_init':'Arm_Init','exit':'exit','doing':'Arm_Close','done':'Arm_Init'})
        smach.StateMachine.add('ARM_MODE',arm_sub, transitions={'exit':'STAND_BY_MODE'})

        manual_move_mode_sub_sub = smach.StateMachine(outcomes=['exit'])
        with manual_move_mode_sub_sub:
            smach.StateMachine.add('Stop_Mode2',Manual_stop_mode(),transitions={'done':'mode_finish','exit':'exit','doing':'Stop_Mode2','right':'Right_Move2','front':'Move_Forward2','behind':'BACK2','left':'Left_Move2'})
            smach.StateMachine.add('Move_Forward2', Manual_move_forward(),transitions={'done':'Stop_Mode2','exit':'exit','doing':'Move_Forward2'})
            smach.StateMachine.add('Right_Move2',Manual_right_move(),transitions={'done':'Stop_Mode2','exit':'exit','doing':'Right_Move2'})
            smach.StateMachine.add('BACK2', Manual_back(),transitions={'done':'Stop_Mode2','exit':'exit','doing':'BACK2'})
            smach.StateMachine.add('Left_Move2', Manual_left_move(),transitions={'done':'Stop_Mode2','exit':'exit','doing':'Left_Move2'})
        smach.StateMachine.add('MANUAL_CONTROL_MODE',manual_move_mode_sub_sub, transitions={'exit':'STAND_BY_MODE'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('action_selection_smach')
    joy_twist = JoyTwist()
    main()
    rospy.spin()
