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

class SequenceEditor(cmd.Cmd):

    '''Subclass of the cmd class'''

    def preloop(self):
        '''Override and used for class variable'''
        logging.debug('Call function preloop()')
        # initialize variables
        self.frequency = DEF_FREQ
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
        frame  = parse_kframe_add(line)
        j1 = float(input('Enter value for j1:'))
        self.sequence['keyframes'][frame] = j1
        is_frame0(self.sequence)

    def do_EOF(self, line):
        '''Override end of file'''
        logging.debug('Call function do_EOF()')
        logging.info('User has requested to stop the program')
        stop = True
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
