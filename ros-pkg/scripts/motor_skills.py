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
# ROS constants
NODE_NAME = 'motor_skills'
TOPIC_JOINT_STATES = '~joint_states'
DEFAULT_FREQUENCY = 25
JOINT_NAME_ARRAY = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'finger_left_joint', 'finger_right_joint']

# affective constants
CALM_POS = (-0.62, 1.44, 1.11)
INTEREST_POS = (-0.51, 0.71, 0.86)

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

def prim_fingers(tar_pos, aperture, speed):
    '''primitive for the fingers'''
    '''positive position = right'''
    cur_pos_l = joints_pos[5]
    cur_pos_r = joints_pos[6]
    tar_pos_l = (tar_pos * -1) + (aperture / 2)
    tar_pos_r = tar_pos + (aperture / 2)

    if cur_pos_l == tar_pos_l and cur_pos_r == tar_pos_r:
        return True
    else:
        lin_speed = (speed * 0.2) + 0.1
        next_pos_l = cur_pos_l + ((tar_pos_l - cur_pos_l)
                * lin_speed / pub_freq)
        next_pos_r = cur_pos_r + ((tar_pos_r - cur_pos_r)
                * lin_speed / pub_freq)
        joints_pos[5] = next_pos_l
        joints_pos[6] = next_pos_r
    return False

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

def prim_arm(tar_pos1, tar_pos2, tar_pos3, speed):
    '''primitive of the arm'''
    cur_pos1 = joints_pos[1]
    cur_pos2 = joints_pos[2]
    cur_pos3 = joints_pos[3]
    if cur_pos1 == tar_pos1 and cur_pos2 == tar_pos2 and cur_pos3 == tar_pos3:
        return True
    else:
        rot_speed = (speed * 0.2) + 0.1
        next_pos1 = cur_pos1 + ((tar_pos1 - cur_pos1) * rot_speed / pub_freq)
        joints_pos[1] = next_pos1
        next_pos2 = cur_pos2 + ((tar_pos2 - cur_pos2) * rot_speed / pub_freq)
        joints_pos[2] = next_pos2
        next_pos3 = cur_pos3 + ((tar_pos3 - cur_pos3) * rot_speed / pub_freq)
        joints_pos[3] = next_pos3
    return False

def main():
    start_time = rospy.Time.now().secs
    while not rospy.is_shutdown():
        loop_time = rospy.Time.now().secs
        duration = loop_time - start_time
        is_finger_done = prim_fingers(0.000, 0.010, 3)
        is_wrist_done = prim_wrist(0.00, 3)
        arm_pos = list(CALM_POS)
        arm_pos.append(1)
        print arm_pos
        is_arm_done = prim_arm(*arm_pos)
        if duration > 15:
            arm_pos = list(INTEREST_POS)
            arm_pos.append(3)
            is_arm_done = prim_arm(*arm_pos)
        publish_position()
        rospy.sleep(1.0 / pub_freq)
    print 'Bye!'

if __name__ == '__main__':
    main()
