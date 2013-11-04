#!/usr/bin/env python
#
# Copyright 2013 Arn-O. See the LICENSE file at the top-level directory of this
# distribution and at
# https://github.com/Arn-O/kuk-A-droid/blob/master/LICENSE.

'''
This program is a component of the kuk-A-droid project.
Its purpose is to edit motion sequence (create/modify/save) and export
a sequence script from call by the motor skills.
'''

import cmd
import logging
import argparse

# global constants
DEF_FREQ = 60
DEF_NB_JOINTS = 1

def get_parameters():
    ''' Manage the execution parameters'''        
    # get input parameters
    parser = argparse.ArgumentParser()
    parser.add_argument(
            "-v",
            "--verbosity",
            action="store_true",
            help='maximum output verbosity'
        )
    args = parser.parse_args()
    
    # manage verbosity level
    if args.verbosity:
        logging.basicConfig(level=logging.DEBUG)
    logging.info('Sequence editor started in maximum verbosity mode')

def is_frame0(seq):
    '''Check if the frame 0 has been defined'''
    logging.debug('Call function is_frame0()')
    if not 0 in seq['keyframes']:
        logging.warning('Frame0 not defined')
    else:
        logging.debug('Frame0 is defined')

def parse_kframe_add(line):
    '''Parse the input for the add keyframe function'''
    logging.debug('Call function parse_kframe_add()')
    # TODO: add some checks here
    return int(line)

# TODO: manage the is_frame0() check in a postcmd method
class SequenceEditor(cmd.Cmd):

    '''Subclass of the cmd class'''

    def preloop(self):
        '''Override and used for class variable'''
        logging.debug('Call function preloop()')
        # initialize variables
        self.frequency = DEF_FREQ
        self.nb_joints = DEF_NB_JOINTS
        self.sequence = {'frequency': self.frequency, 'keyframes': {}}
        print("'Crtl+D' or EOF to quit")
        is_frame0(self.sequence)
        
    def do_disp_param(self, line):
        '''Display a list of the current parameters'''
        logging.debug('Call function do_disp_param()')
        print('Frequency: %i' % self.frequency)
        is_frame0(self.sequence)

    def do_kframe_disp(self, line):
        '''Insert a new keyframe'''
        logging.debug('Call function do_kframe_disp()')
        print self.sequence
        is_frame0(self.sequence)
        
    def do_kframe_add(self, line):
        '''Insert a new keyframe'''
        logging.debug('Call function do_kframe_add()')
        frame_nb  = parse_kframe_add(line)
        vals = []
        for i in range(self.nb_joints):
            val = float(input(
                    'Enter value for joint %i / %i: '  %
                    ((i + 1), self.nb_joints)
                ))
            vals.append(val)
        self.sequence['keyframes'][frame_nb] = vals
        is_frame0(self.sequence)

    def do_seq_disp(self, line):
        '''Display the sequence of motion with interpolation'''
        logging.debug('Call function seq_disp()')
        if len(self.sequence['keyframes']) == 0:
            logging.info('No frames to display')
            return
        last_frame = max(self.sequence['keyframes'])
        logging.debug('Last frame number %i:', last_frame)
        for i in range(last_frame + 1):
            print('Frame: %i / %i ' % (i, last_frame)),
            print('- time:')
        is_frame0(self.sequence)
        
    def do_EOF(self, line):
        '''Override end of file'''
        logging.debug('Call function do_EOF()')
        logging.info('User has requested to stop the program')
        print "Bye!"
        return True

def main():
    '''Where everything starts.'''
    
    # initialize
    get_parameters()

    # start the cmd loop
    se = SequenceEditor()
    se.prompt = "(SequenceEditor) "
    se.cmdloop()
          
if __name__ == '__main__':
    main()
