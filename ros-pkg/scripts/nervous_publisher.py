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
import rospy

# global constants
DEFAULT_START_MODE = 'aware'
NODE_NAME = 'nervous_system'

NERVOUS_MODES = ['aware', 'zombie']

def stop_node():
    '''Clean stuff here.'''
    rospy.delete_param('~mode')
    print 'Bye!'

def handle_set_nervous_mode(req):
    '''Handle for nervous_mode setter (aware/zombie).'''
    # TODO warning if already in requested mode
    if req.nervous_mode in NERVOUS_MODES:
        rospy.set_param('~mode', req.nervous_mode)
        rospy.loginfo('Nervous mode set to %s', req.nervous_mode)
    else:
        rospy.logerr('Nervous mode not allowed. Possible values are: aware/zombie')
    return SetNervousModeResponse()

def main():
    '''Where everything starts.'''
    rospy.init_node(NODE_NAME)
    rospy.set_param('~mode', DEFAULT_START_MODE)
    rospy.loginfo('Nervous system started in %s mode, this can be changed using services', DEFAULT_START_MODE)
    rospy.on_shutdown(stop_node)
    s = rospy.Service('~set_nervous_mode', SetNervousMode,
            handle_set_nervous_mode)
    rospy.spin()

if __name__ == '__main__':
    main()
