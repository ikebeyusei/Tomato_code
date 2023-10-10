#!/usr/bin/env/python

import rospy
import smach
import smach_ros
import time
from sensor_msgs.msg import Joy

madeCounter = 1
wasButtonPush = False

class JoyTwist(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy',Joy,self.joy_callback,queue_size=1)

    def joy_callback(self,joy_msgs):
        global modeCounter
        global wasButtonPush
        #avoiding Duplicate Reads
        #joy_msgs[16] ==> PS_button
        if joy_msgs[16] == 1:
            if wasButtonPush == 0:
                modeCounter = modeCounter + 1
                wasButtonPush = True
        else:
            wasButtonPush = False

        if modeCounter == 51:
            modeCounter = 1

#move_mode
class Stop(smach.State):
    #process everything through "Stop"
    def __init__(self):
        smach.State.__init__(self,outcomes=['move_forward','step_back','turn_left','turn_right','move_to_the_right','move_to_the_left','doing'])

    def execute(self,userdata):
        global robot_mode
        robot_mode = 0
        #when modeCounter is 2, R2 tries to keep stopping
        if modeCounter == 2:
            return 'doing'
        elif modeCounter == 3:
            return 'move_forward'
        elif modeCounter == 4:
            return 'step_back'
        elif modeCounter == 5:
            return 'turn_left'
        elif modeCounter == 6:
            return 'turn_right'
        elif modeCounter == 7:
            return 'move_to_the_right'
        elif modeCounter == 8:
            return 'move_to_the_left'
        
class Move_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop'])

    def execute(self,userdata):
        time.sleep(1.0)#I'm going to change this
        return 'stop'#after 1 second, put it into Stop_mode

class Step_back(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop'])

    def execute(self,userdata):
        time.sleep(1.0)#I'm going to change this
        return 'stop'#after 1 second, put it into Stop_mode

class Turn_left(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop'])

    def execute(self,userdata):
        time.sleep(1.0)#I'm going to change this
        return 'stop'#after 1 second, put it into Stop_mode
        
class Turn_right(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop'])

    def execute(self,userdata):
        time.sleep(1.0)#I'm going to change this
        return 'stop'#after 1 second, put it into Stop_mode
    
class Move_to_the_right(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop'])

    def execute(self,userdata):
        time.sleep(1.0)#I'm going to change this
        return 'stop'#after 1 second, put it into Stop_mode
    
class Move_to_the_left(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['stop'])

    def execute(self,userdata):
        time.sleep(1.0)#I'm going to change this
        return 'stop'#after 1 second, put it into Stop_mode
    
#camera!!!!!
class Image_recognition(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['done','doing'])

    def execute(self,userdata):
        global detection_flag
        detection_flag = False
        time.sleep(2.0)
        detection_flag = True #true if R2 find the ball
        if detection_flag == True:
            return 'done'
        else:
            return 'doing'

class Distance_calculation(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['doing','done'])

    def execute(self,userdata):
        global enough_near
        enough_near = False #true when close enough to the ball
        time.sleep(2.0)
        enough_near = True
        if enough_near == True:
            return 'done'
        else:
            return 'doing'
        
#arm!!!!
class Arm_init(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['done'])
    
    def execute(self,userdata):
        #write the process to set the arm to the initial position
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
            return 'not_detection'

    
def main():
    sm = smach.StateMachine(outcomes=[''])

    with sm:
        init_sub = smach.StateMachine(outcomes=['Not','doing'])
        with init_sub:
            smach.StateMachine.add('Line_Tracing',Line_tracing(),transitions={'detection':'Line_Tracing','not_detected':'doing'})
        smach.StateMachine.add('LINE_TRACING',init_sub,transitions={'Not':'MOVE_MODE','doing':'LINE_TRACING'})

        move_sub = smach.StateMachine(outcomes = ['mode_finish'])
        with move_sub:
            smach.StateMachine.add('Stop!',Stop(),transitions={'doing':'Stop!','move_forward':'Move_Forward','step_back':'Step_Back','turn_left':'Turn_Left','turn_right':'Turn_Right','move_to_the_right':'Move_To_The_Right','move_to_the_left':'Move_To_The_Left'})
            smach.StateMachine.add('Move_Forward',Move_forward(),transitions={'stop':'Stop!'})
            smach.StateMachine.add('Step_Back',Step_back(),transitions={'stop':'Stop!'})
            smach.StateMachine.add('Turn_Left',Turn_left(),transitions={'stop':'Stop!'})
            smach.StateMachine.add('Turn_Right',Turn_right(),transitions={'stop':'Stop!'})
            smach.StateMachine.add('Move_To_The_Right',Move_to_the_right(),transitions={'stop':'Stop!'})
            smach.StateMachine.add('Move_To_The_Left',Move_to_the_left(),transitions={'stop':'Stop!'})
        smach.StateMachine.add('MOVE_MODE',move_sub,transitions={'mode_finish':'ARM_MODE'})

        arm_sub = smach.StateMachine(outcomes = {'mode_finish'})
        with arm_sub:
            smach.StateMachine.add('Arm_Init',Arm_init(),transitions={'done':'Arm_Open'})
            smach.StateMachine.add('Arm_Opne',Arm_open(),transitions={'done':'Arm_Close'})
            smach.StateMachine.add('Arm_Close',Arm_close(),transitions={'done':'MOVE_MODE'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('action_selection_smach')
    joy_twist = JoyTwist()
    main()
    rospy.spin()
