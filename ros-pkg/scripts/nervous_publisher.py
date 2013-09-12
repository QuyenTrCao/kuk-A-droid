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

from kuk_a_droid.srv.SetNervousMode  import SetNervousMode
import rospy

# global constants
DEFAULT_START_MODE = 'aware'

def stop_node():
    '''Clean stuff here.'''
    rospy.delete_param('~mode')
    print 'Bye!'

def handle_set_nervous_mode():
    '''Handle for nervous_mode setter (aware/zombie).'''
    print 'Change mode'
    return SetNervousModeResponse()

def main():
    '''Where everything starts.'''
    rospy.init_node('nervous_system')
    rospy.set_param('~mode', DEFAULT_START_MODE)
    rospy.on_shutdown(stop_node)
    s = rospy.Service('set_nervous_mode', SetNervousMode,
            handle_set_nervous_mode)
    rospy.spin()

if __name__ == '__main__':
    main()
