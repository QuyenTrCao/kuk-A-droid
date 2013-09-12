#!/usr/bin/env python
#
# Copyright 2013 _ArnO_. See the LICENSE file at the top-level directory of
# this distribution and at
# https://github.com/Arn-O/kuk-A-droid/blob/master/LICENSE.

'''
xxx
'''

import rospy

# global constants

class CommandYouBot(cmd.Cmd):
    
    '''Subclass of the cmd class'''

    def preloop(self):
        '''Override and used for class variable'''
        
    def do_gripper_open(self, line):
        '''
        Open the gripper to the max position.
        Usage: gripper_open
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
    cy = CommandYouBot()
    cy.cmdloop()

if __name__ == '__main__':
    main()
