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
JOINT_NAME_ARRAY = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'finger_left_joint', 'finger_right_joint']

# global variales
    
#TODO: create a parameter on the parameter server for the frequence
pub_freq = DEFAULT_FREQUENCY
# ROS variables
rospy.init_node(NODE_NAME)
pub_js = rospy.Publisher(TOPIC_JOINT_STATES, JointState)
joints_pos = [0.0] * 7 


def publish_position():
    '''joint position publisher'''    
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = JOINT_NAME_ARRAY
    msg.position = joints_pos
    pub_js.publish(msg)

def prim_wrist(tar_pos, speed):
    '''primitive for the wrist (arm_5_joint)'''
    cur_pos = joints_pos[4]
    if cur_pos == tar_pos:
        return True
    else:
        rot_speed = (speed * 0.2) + 0.1
        next_pos = cur_pos + ((tar_pos - cur_pos) * rot_speed / pub_freq)
        joints_pos[4] = next_pos
    return False

def main():
    counter = 0
    while not rospy.is_shutdown():
        print counter
        if counter > 100:
            completion = prim_wrist(1.0, 3)
        publish_position()
        counter = counter + 1
        rospy.sleep(1.0 / pub_freq)
    print 'Bye!'

if __name__ == '__main__':
    main()
