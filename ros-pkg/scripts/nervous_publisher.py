#!/usr/bin/env python
#
# Copyright 2013 _ArnO_. See the LICENSE file at the top-level directory of
# this distribution and at
# https://github.com/Arn-O/kuk-A-droid/blob/master/LICENSE.

'''
This program is a component of the kuk-A-droid project.
It will publish to ROS topics the valuation of the emotions and the behaviors.
The values can come from the onboarded device (aware mode) or from the teleop
program (zombie mode).
'''

import roslib; roslib.load_manifest('kuk_a_droid')

from kuk_a_droid.srv import *
from kuk_a_droid.msg import *
import rospy

# global constants
# ROS parameters
DEFAULT_START_MODE = 'aware'
NODE_NAME = 'nervous_system'
PUBLISHER_NAME = 'nervous_states'
PUB_FREQUENCY = 25

# affective parameters
NERVOUS_MODES = ['aware', 'zombie']
# note: emotions and behaviors are listed in the following orders
# emotions = anger frustration fear distress disgust sorrow surprise interest calm boredom joy
# behaviors = seek-people seek-toy sleep
EMOTIONS_INIT = [0, 0, 0, 0, 0, 0, 0, 0, 1500, 0, 0] #Calm
BEHAVIORS_INIT = [0, 0, 1800] #Sleep

def stop_node():
    '''Clean stuff here.'''
    rospy.delete_param('~mode')
    rospy.logwarn('Nervous publisher has stopped - Bye!')

def handle_set_nervous_mode(req):
    '''Handle for nervous_mode setter (aware/zombie).'''
    # TODO warning if already in requested mode
    if req.nervous_mode in NERVOUS_MODES:
        rospy.set_param('~mode', req.nervous_mode)
        rospy.loginfo('Nervous mode set to %s', req.nervous_mode)
    else:
        rospy.logerr('Nervous mode not allowed. Possible values are: aware/zombie')
    return SetNervousModeResponse()

def get_nerv_msg(emotions, behaviors):
    '''Convert emotions and behaviors lists to msg'''
    emot_param = Emotions(*emotions)
    behav_param = Behaviors(*behaviors)
    nerv_msg = NervStates(emot_param, behav_param)
    return nerv_msg

def nervous_publisher():
    '''Publisher loop'''
    pub = rospy.Publisher(PUBLISHER_NAME, NervStates)
    emotions = EMOTIONS_INIT
    behaviors = BEHAVIORS_INIT
    while not rospy.is_shutdown():
        msg = get_nerv_msg(emotions, behaviors)
        pub.publish(msg)
        rospy.sleep(1.0 / PUB_FREQUENCY)

def main():
    '''Where everything starts.'''
    rospy.init_node(NODE_NAME)
    rospy.on_shutdown(stop_node)
    rospy.set_param('~mode', DEFAULT_START_MODE)
    s = rospy.Service('~set_nervous_mode', SetNervousMode,
            handle_set_nervous_mode)
    rospy.loginfo('Nervous system started in %s mode, this can be changed using services', DEFAULT_START_MODE)
    
    try:
        nervous_publisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
