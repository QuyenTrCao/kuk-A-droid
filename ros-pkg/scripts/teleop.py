#!/usr/bin/env python
#
# Copyright 2013 Arn-O. See the LICENSE file at the top-level directory of this
# distribution and at
# https://github.com/Arn-O/kuk-A-droid/blob/master/LICENSE.

'''
### Complete stub ###
Test program from designing the affective respond to emotions and behaviors
states. This program publishes the affective states. The states can be changed
using a cmd interface.
'''

from kuk_a_droid.msg import *
import rospy

# global constants
# ROS parameters
NODE_NAME = 'nervous_system'
PUBLISHER_NAME = 'nervous_states'
DEFAULT_FREQUENCY = 25

# note: emotions and behaviors are listed in the following orders
# emotions = anger frustration fear distress disgust sorrow surprise interest calm boredom joy
# behaviors = seek-people seek-toy sleep
EMOTIONS_DICT = {'anger':0, 'frustration':1, 'fear':2, 'distress':3,
                'disgust':4, 'sorrow':5, 'surprise':6, 'interest':7, 'calm':8,
                        'boredom':9, 'joy':10}
BEHAVIORS_DICT = {'seek-people':0, 'seek-toy':1, 'sleep':2}

class AffectiveYouBot(cmd.Cmd):
    
    '''Subclass of the cmd class'''

    def preloop(self):
        '''Override and used for class variable'''
        rospy.init_node(NODE_NAME)
        rospy.on_shutdown(stop_node)


    def do_emotion(self, line):
        '''
        Set the emotion to a given state and value.
        Usage: emotion fear
               emotion fear 950
        '''
        (rankfl, pubfl) = self.joints_info['fl']
        (rankfr, pubfr) = self.joints_info['fr']
        pub_arm_pose(pubfl, 0.0125)
        pub_arm_pose(pubfr, 0.0125)


    def do_EOF(self, line):
        '''Override end of file'''
        print "Bye!"
        return True

def main():
    cy = AffectiveYouBot()
    cy.cmdloop()

if __name__ == '__main__':
    main()
