#!/usr/bin/env python
#
# Copyright 2013 Arn-O. See the LICENSE file at the top-level directory of this
# distribution and at
# https://github.com/Arn-O/kuk-A-droid/blob/master/LICENSE.

'''
This program is a component of the kuk-A-droid project.
It will publish to ROS topics the valuation of the emotions and the behaviors.
The values can come from the onboarded device (aware mode) or from the teleop
program (zombie mode).
'''

from kuk_a_droid.msg import *

import rospy
#import roslib; roslib.load_manifest('kuk_a_droid')
import random
import argparse

# global constants
# ROS parameters
NODE_NAME = 'nervous_system'
PUBLISHER_NAME = 'nervous_states'
DEFAULT_FREQUENCY = 25

# affective parameter
# note: emotions and behaviors are listed in the following orders
# emotions = anger frustration fear distress disgust sorrow surprise interest calm boredom joy
# behaviors = seek-people seek-toy sleep
EMOTIONS_DICT = {'anger':0, 'frustration':1, 'fear':2, 'distress':3,
        'disgust':4, 'sorrow':5, 'surprise':6, 'interest':7, 'calm':8,
        'boredom':9, 'joy':10}
BEHAVIORS_DICT = {'seek-people':0, 'seek-toy':1, 'sleep':2}

DEFAULT_EMOTION = 'calm'
DEFAULT_BEHAVIOR = 'sleep'

def stop_node():
    '''Clean stuff here.'''

def get_nerv_msg(emotions, behaviors):
    '''Convert emotions and behaviors lists to msg.'''
    emot_param = Emotions(*emotions)
    behav_param = Behaviors(*behaviors)
    nerv_msg = NervStates(emot_param, behav_param)
    return nerv_msg

def add_random(val, max_var):
    '''Add a random value in limits.'''
    val = val +  random.randint(-max_var, max_var)
    if val < 0: val = 0
    if val > 1000: val = 1000
    return val

def add_noise(states, max_var):
    '''Add noise to affective states.'''
    states = [add_random(state, max_var) for state in states]
    return states

def nervous_publisher(freq, ref_emotions, ref_behaviors):
    '''Publisher loop'''
    pub = rospy.Publisher(PUBLISHER_NAME, NervStates)
    while not rospy.is_shutdown():
        emotions = add_noise(ref_emotions, 50)
        msg = get_nerv_msg(emotions, ref_behaviors)
        pub.publish(msg)
        rospy.sleep(1.0 / freq)

def main():
    '''Where everything starts.'''
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--frequency",
            type=int,
            default=DEFAULT_FREQUENCY,
            help='nervous publisher frequency')
    parser.add_argument("-e", "--emotion",
            choices=EMOTIONS_DICT.keys(),
            default=DEFAULT_EMOTION,
            help='emotion')
    parser.add_argument("-a", "--activation",
            type=int,
            default=950,
            help='level of activation of the emotion (max = 1000)')

    args = parser.parse_args()
    pub_freq = args.frequency

    emotions = [0] * 11
    behaviors = [0] * 3
    emotions[EMOTIONS_DICT.get(args.emotion)] = args.activation
        
    rospy.init_node(NODE_NAME)
    rospy.on_shutdown(stop_node)
    
    try:
        nervous_publisher(pub_freq, emotions, behaviors)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
