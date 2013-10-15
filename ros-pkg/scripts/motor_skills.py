#!/usr/bin/env python
#
# Copyright 2013 Arn-O. See the LICENSE file at the top-level directory of this
# distribution and at
# https://github.com/Arn-O/kuk-A-droid/blob/master/LICENSE.

'''
This program is a component of the kuk-A-droid project.
The motor skills system is in charge of transforming the affective states into
actuator commands.
'''

from kuk_a_droid.msg import *
from sensor_msgs.msg import JointState

import rospy

# global constants

NODE_NAME = 'motor_skills'
TOPIC_JOINT_STATES = '~joint_states'
DEFAULT_FREQUENCY = 25
JOINT_ARRAY = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'finger_left_joint', 'finger_right_joint']

def main():
    #TODO: create a parameter on the parameter server for the frequence
    pub_freq = DEFAULT_FREQUENCY
    rospy.init_node(NODE_NAME)
    pub_js = rospy.Publisher(TOPIC_JOINT_STATES, JointState)
    speed_rot = 0.1
    joint_4_pos = -1.7
    clockwise = 1
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_ARRAY
        msg.position = [0.00, 0.00, 0.00, joint_4_pos, 0.00, 0.00, 0.00]
        pub_js.publish(msg)
        joint_4_pos = joint_4_pos + (speed_rot * (1.0 / pub_freq) * clockwise)
        if joint_4_pos < -1.7 or joint_4_pos > 1.7:
            clockwise = clockwise * -1
        rospy.sleep(1.0 / pub_freq)
    print 'Bye!'

if __name__ == '__main__':
    main()
