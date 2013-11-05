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

import pickle
import cmd
import logging
import argparse

# global constants
DEF_FREQ = 60
DEF_NB_JOINTS = 1
DEF_FNAME = 'keyframes.txt'

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

def is_frame0(kf):
    '''Check if the frame 0 has been defined'''
    logging.debug('Call function is_frame0()')
    if not 0 in kf['frames']:
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
        self.fname = DEF_FNAME
        self.keyframes = {'frequency': self.frequency, 'frames': {}}
        print("'Crtl+D' or EOF to quit")
        is_frame0(self.keyframes)
        
    def do_param_disp(self, line):
        '''Display a list of the current parameters'''
        logging.debug('Call function do_param_disp()')
        print('Frequency: %i' % self.frequency)
        print('Number of joint(s): %i' % self.nb_joints)
        print('File name: %s' % self.fname)
        is_frame0(self.keyframes)

    def do_kframe_disp(self, line):
        '''Insert a new keyframe'''
        logging.debug('Call function do_kframe_disp()')
        print self.keyframes
        is_frame0(self.keyframes)
        
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
        self.keyframes['frames'][frame_nb] = vals
        is_frame0(self.keyframes)

    def do_kframe_save(self, line):
        '''Save the current key frames'''
        logging.debug('Call function do_kframe_save()')
        f = open(self.fname, 'w')
        pickle.dump(self.keyframes, f)
        f.close()
        is_frame0(self.keyframes)
        
    def do_kframe_open(self, line):
        '''Open a key frames file'''
        logging.debug('Call function do_kframe_open()')
        f = open(self.fname, 'r')
        self.keyframes = pickle.load(f) 
        f.close()
        logging.debug('Key frames loaded: %s', self.keyframes)
        print('Run kframe_disp to check the content')
        is_frame0(self.keyframes)
                
    def do_seq_disp(self, line):
        '''Display the sequence of motion with interpolation'''
        logging.debug('Call function seq_disp()')
        if len(self.keyframes['frames']) < 2:
            logging.info('At least 2 keyframes required')
            return
        last_frame = max(self.keyframes['frames'])
        logging.debug('Last frame number %i:', last_frame)
        for i in range(last_frame + 1):
            print('Frame: %i / %i ' % (i, last_frame)),
            print('- time:'),
            print('- type: '),
            if i in keyframes['frames']:
                print "K"
            else:
                print "I"
        is_frame0(self.keyframes)
        
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
