#!/usr/bin/env python
#
# Copyright 2013 _ArnO_. See the LICENSE file at the top-level directory of
# this distribution and at XXXXXXX.

'''
xxx
'''

from kuk_a_droid.srv.SetNervousMode  import SetNervousMode
import rospy

# global constants
DEFAULT_START_MODE = 'aware'

def stop_node():
    rospy.delete_param('~mode')
    print 'Bye!'

def handle_set_nervous_mode():
    print 'Change mode'
    return SetNervousModeResponse()

def main():
    rospy.init_node('nervous_system')
    rospy.set_param('~mode', DEFAULT_START_MODE)
    rospy.on_shutdown(stop_node)
    s = rospy.Service('set_nervous_mode', SetNervousMode,
            handle_set_nervous_mode)
    rospy.spin()

if __name__ == '__main__':
    main()
