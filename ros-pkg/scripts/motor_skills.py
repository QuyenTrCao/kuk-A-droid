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
# reference position of the arm
ANGER_POS = (0.62, 0.07, 0.36)
FRUSTRATION_POS = (0.08, 0.02, -0.36)
FEAR_POS = (-0.99, 0.12, 1.79)
DISTRESS_POS = (0.39, 1.16, 0.78)
DISGUST_POS = (-0.08, 2.38, 0.03)
SORROW_POS = (1.57, 0.10, 1.79)
SURPRISE_POS = (-0.43, -0.23, 0.67)
INTEREST_POS = (-0.51, 0.71, 0.86)
CALM_POS = (-0.62, 1.44, 1.11)
BOREDOM_POS = (0.43, 1.53, 1.11)
JOY_POS = (-0.96, 0.31, 0.86)

# global variales
    
# TODO: create a parameter on the parameter server for the frequence
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

# robot primitive for controlling group of joints
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
        rot_speed = (speed * 0.22) + 0.12
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
        if abs(tar_pos1 - cur_pos1) > (rot_speed / pub_freq):
            clockwise = (tar_pos1 - cur_pos1) / abs(tar_pos1 - cur_pos1)
            next_pos1 = cur_pos1 + (clockwise * rot_speed / pub_freq)
        else:
            next_pos1 = tar_pos1
        joints_pos[1] = next_pos1
        if abs(tar_pos2 - cur_pos2) > (rot_speed / pub_freq):
            clockwise = (tar_pos2 - cur_pos2) / abs(tar_pos2 - cur_pos2)
            next_pos2 = cur_pos2 + (clockwise * rot_speed / pub_freq)
        else:
            next_pos2 = tar_pos2
        joints_pos[2] = next_pos2
        if abs(tar_pos3 - cur_pos3) > (rot_speed / pub_freq):
            clockwise = (tar_pos3 - cur_pos3) / abs(tar_pos3 - cur_pos3)
            next_pos3 = cur_pos3 + (clockwise * rot_speed / pub_freq)
        else:
            next_pos3 = tar_pos3
        joints_pos[3] = next_pos3
    return False

# emotion skills
def anger():
    '''demonstrate anger'''
    arousal = 4
    # fingers
    is_finger_done = prim_fingers(0.000, 0.022, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.00, arousal)
    
    # arm
    arm_pos = list(ANGER_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def frustration():
    '''demonstrate frustration'''
    arousal = 3
    # fingers
    is_finger_done = prim_fingers(0.000, 0.010, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(1.50, arousal)
    
    # arm
    arm_pos = list(FRUSTRATION_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def fear():
    '''demonstrate fear'''
    arousal = 4
    # fingers
    is_finger_done = prim_fingers(0.000, 0.000, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.00, arousal)
    
    # arm
    arm_pos = list(FEAR_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def distress():
    '''demonstrate distress'''
    arousal = 4
    # fingers
    is_finger_done = prim_fingers(0.000, 0.000, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.00, arousal)
    
    # arm
    arm_pos = list(DISTRESS_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def disgust():
    '''demonstrate disgust'''
    arousal = 2
    # fingers
    is_finger_done = prim_fingers(0.000, 0.000, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.00, arousal)
    
    # arm
    arm_pos = list(DISGUST_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def sorrow():
    '''demonstrate sorrow'''
    arousal = 0
    # fingers
    is_finger_done = prim_fingers(0.000, 0.000, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.00, arousal)
    
    # arm
    arm_pos = list(SORROW_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def surprise():
    '''demonstrate surprise'''
    arousal = 4
    # fingers
    is_finger_done = prim_fingers(0.000, 0.000, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.00, arousal)
    
    # arm
    arm_pos = list(SURPRISE_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def interest():
    '''demonstrate interest'''
    arousal = 3
    # fingers
    is_finger_done = prim_fingers(0.000, 0.020, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.00, arousal)
    
    # arm
    arm_pos = list(INTEREST_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def calm():
    '''demonstrate calm'''
    arousal = 1
    # fingers
    is_finger_done = prim_fingers(0.000, 0.010, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.40, arousal)
    
    # arm
    arm_pos = list(CALM_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def boredom():
    '''demonstrate boredom'''
    arousal = 1
    # fingers
    is_finger_done = prim_fingers(0.000, 0.010, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.40, arousal)
    
    # arm
    arm_pos = list(BOREDOM_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def joy():
    '''demonstrate joy'''
    arousal = 3
    # fingers
    is_finger_done = prim_fingers(0.000, 0.010, arousal)
    
    # wrist
    is_wrist_done = prim_wrist(0.40, arousal)
    
    # arm
    arm_pos = list(JOY_POS)
    arm_pos.append(arousal)
    is_arm_done = prim_arm(*arm_pos)

def main():
    seq = {0:calm, 1:anger, 2:frustration, 3:fear, 4:distress, 5:disgust, 
            6:sorrow, 7:surprise, 8:interest, 9:calm, 10:boredom, 11:joy}
    start_time = rospy.Time.now().secs
    cur_emotion = None
    while not rospy.is_shutdown():
        loop_time = rospy.Time.now().secs
        duration = loop_time - start_time
        index = (duration / 7)
        if seq[index].__name__ != cur_emotion:
            cur_emotion = seq[index].__name__
            rospy.loginfo("Emotion state changes to %s", cur_emotion)
        seq[index]()
        publish_position()
        # TODO: use the frequence synchronisation
        rospy.sleep(1.0 / pub_freq)
    print 'Bye!'

if __name__ == '__main__':
    main()
