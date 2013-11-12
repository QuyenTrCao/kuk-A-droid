#!/usr/bin/env python
#
# Copyright 2013 Arn-O. See the LICENSE file at the top-level directory of this
# distribution and at
# https://github.com/Arn-O/kuk-A-droid/blob/master/LICENSE.

'''
This program is a component of the kuk-A-droid project.
Its purpose is to edit motion sequence (create/modify/save) based on key frames
and interpolation points. An export can be generated and used by the motor
skills.
'''

import numpy as np
import matplotlib.pyplot as plt
import json
import pickle
import cmd
import logging
import argparse

# global constants
DEF_FREQ = 60
DEF_NB_JOINTS = 1
DEF_WNAME = 'new_work'


def get_parameters():
    '''Manage the execution parameters'''        
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

def get_fnums(kf):
    '''Return an array with the keyframes numbers'''  
    logging.debug('Call function get_fnums()')
    fnums = [fnum for (fnum, jposes) in kf]
    logging.debug('Frames numbers: %s', fnums)   
    return fnums

def get_jposes(kf):
    '''Return an array with the arrays poses'''     
    logging.debug('Call function get_jposes()')
    jposes = [jposes for (fnum, jposes) in kf]
    return jposes    
 
def get_coord(frame, i):
    '''Return point coordinates from frames'''
    logging.debug('Call function get_coord()')
    (x, poses) = frame
    y = poses[i]
    return (x, y)
    
def sort_kframes(kf):
    '''Sort the keyframes in place'''
    logging.debug('Call function sort_kframes()')
    logging.debug('Before: %s', kf)
    kf.sort(key=lambda (fnum, jposes): fnum)    
    logging.debug('After: %s', kf)   

def is_frame0(kf):
    '''Check if the frame 0 has been defined'''
    logging.debug('Call function is_frame0()')
    fnums = get_fnums(kf)     
    if not 0 in fnums:
        logging.warning('Frame0 not defined, you should create one')
    else:
        logging.debug('Frame0 is defined')

# parse functions
def parse_kframe_add(line):
    '''Parse the input for the add keyframe function'''
    logging.debug('Call function parse_kframe_add()')   
    # TODO: add some checks here (no value for example or existing frame)
    return int(line)

def parse_kframe_open(line):
    '''Parse the open function'''
    logging.debug('Call function parse_kframe_open()')
    args = str.split(line)
    # TODO: add file checks
    fname = args [0] + '_kf.txt'
    return fname

# set parameters functions
def set_nb_joints(obj, val):
    '''Set number of joints'''
    logging.debug('Call function set_nb_joints()')
    obj.param['nb_joints'] = int(val)

def set_freq(obj, val):
    '''Set the frequency'''
    logging.debug('Call function set_freq()')
    obj.param['freq'] = int(val) 

def set_work_name(obj, val):
    '''Set the work name'''
    logging.debug('Call function set_work_name()')
    obj.param['wname'] = val 

def parse_param_set(line):
    '''Parse the input for the set parameter function'''
    logging.debug('Call function parse_param_set()')
    args = str.split(line)
    param_to_fcts = {
        'nb_joints': set_nb_joints,
        'freq': set_freq,
        'work_name': set_work_name
    }
    if args[0] in param_to_fcts:
        fct = param_to_fcts[args[0]]
        val = args[1]
        return (fct, val)
    else:
        logging.warning('Parameter not found')
        print('Unknown parameter: %s' % args[0])
        print('Available parameters: %s ' % param_to_fcts.keys())
        return None, None

def can_gen_seq(kf):
    '''Check if the sequence can be generated'''
    logging.debug('Call function can_gen_seq()')
    fnums = get_fnums(kf)
    if len(fnums) < 2:
        logging.warning('Not enough keyframes, at least two required')
        return False
    else:
        return True
    
def bezier_curve(p0, p1, p2, p3):
    '''Return the interpolated values using a cubic Bezier curve'''
    logging.debug('Call function bezier_curve()')      
    # Get each coordinates individually
    logging.debug('Control points: %s, %s, %s, %s', p0, p1, p2, p3)
    (x0, y0) = p0
    (x1, y1) = p1
    (x2, y2) = p2
    (x3, y3) = p3
    # calculate terms of the interpolation
    t = np.linspace(0, 1, 1000)
    # TODO: define the best linspace step
    # Check if this could be done directly on the tuples
    # Not a big deal, since it should be converted to array anyway
    xterm0 = (1 - t) * (1 - t) * (1 - t) * x0
    yterm0 = (1 - t) * (1 - t) * (1 - t) * y0
    xterm1 = 3 * (1 - t) * (1 - t) * t * x1
    yterm1 = 3 * (1 - t) * (1 - t) * t * y1
    xterm2 = 3 * (1 - t) * t * t * x2
    yterm2 = 3 * (1 - t) * t * t * y2
    xterm3 = t * t * t * x3
    yterm3 = t * t * t * y3
    xterm = xterm0 + xterm1 + xterm2 + xterm3
    yterm = yterm0 + yterm1 + yterm2 + yterm3
    # generate the arrays to return
    bzi = np.array([])
    bzp = np.array([])
    for i in range((x0 + 1), x3):
        rank = (np.abs(xterm - i)).argmin()
        bzi = np.append(bzi, i)
        bzp = np.append(bzp, yterm[rank])
    logging.debug('Bezier curve i: %s', bzi)
    logging.debug('Bezier curve p: %s', bzp)
    return bzi, bzp

def get_seq(kf):
    '''Generate the full sequence'''
    # Step 1: create the kf np arrays
    kfi = np.array(get_fnums(kf))
    kfp = np.array(get_jposes(kf))
    logging.debug('Keyframes array i: %s', kfi)
    logging.debug('Keyframes array p: %s', kfp)
    # Step2: create the if np arrays
    ifi = np.array([])
    ifp = np.array([])
    nb_joints = len(kfp[0])
    logging.info('Number of joints: %i', nb_joints) 
    for (kfs, kfe) in zip(kf[:-1], kf[1:]):
        for j in range(nb_joints):
            # TODO: ugly code, use an enumate on the joints name
            print kfs
            p0 = (x0, y0) = get_coord(kfs, j)
            p3 = (x3, y3) = get_coord(kfe, j)
            p1 = (x1, y1) = (((x3 - x0) * 0.25 + x0), y0)
            p2 = (x2, y2) = (((x3 - x0) * 0.75 + x0), y3)
            # TODO: extract interpolated i, should be done 1 time
            (bfi, bfp) = bezier_curve(p0, p1, p2, p3) 
            ifi = np.append(ifi, bfi)
            ifp = np.append(ifp, bfp)
    logging.debug('Interpolated frames array i: %s', ifi)
    logging.debug('Interpolated frames array p: %s', ifp)
    # Step3: merge key frames and interpolated frames
    sfi = np.array([])
    sfp = np.array([])
    sft = np.array([])
    for i in range(0, (np.amax(kfi) + 1)):
        sfi = np.append(sfi, i)
        if i in kfi:
            sfp = np.append(sfp, kfp[np.where(kfi==i)])
            sft = np.append(sft, 'k')
        if i in ifi:
            sfp = np.append(sfp, ifp[np.where(ifi==i)])
            sft = np.append(sft, 'i')
    logging.debug('Sequence frames array i: %s', sfi)
    logging.debug('Sequence frames array p: %s', sfp)
    logging.debug('Sequence frames array t: %s', sft)
    return sfi, sfp, sft

#def trans_it(pos_values):
#    '''Format pos values into frames dict'''
#    iframes = {'frequency': DEF_FREQ, 'frames': {}}
#    # TODO: should use the frequency from the parameters    
#    for pos_value in pos_values:
#        (i, p) = pos_value
#        vals = []
#        vals.append(p)
#        iframes['frames'][i] = vals
#    return iframes
    
# TODO: manage the is_frame0() check in a postcmd method
class SequenceEditor(cmd.Cmd):

    '''Subclass of the cmd class'''

    def preloop(self):
        '''Override and used for class variable'''
        logging.debug('Call function preloop()')
        # initialize variables
        self.param = {
                'freq': DEF_FREQ,
                'nb_joints': DEF_NB_JOINTS,
                'wname': DEF_WNAME,
                'jnames': ['grip1'],
            }
        self.kf = []        
        print("'Crtl+D' or EOF to quit")
        is_frame0(self.kf)
        
    def do_param_disp(self, line):
        '''Display a list of the current parameters'''
        logging.debug('Call function do_param_disp()')
        print('Frequency: %i' % self.param['freq'])
        print('Number of joint(s): %i' % self.param['nb_joints'])
        print('Work name: %s' % self.param['wname'])
        is_frame0(self.kf)

    def do_param_set(self, line):
        '''Set the number of joints'''
        (action, val) = parse_param_set(line)
        if action:
            action(self, val)
        is_frame0(self.kf)

    def do_kframe_disp(self, line):
        '''Display the keyframes'''
        logging.debug('Call function do_kframe_disp()')
        print self.kf
        is_frame0(self.kf)
        
    def do_kframe_add(self, line):
        '''Insert a new keyframe'''
        logging.debug('Call function do_kframe_add()')
        fnum  = parse_kframe_add(line)
        jposes = []
        for i in range(self.param['nb_joints']):
            jpose = float(input(
                    'Enter value for joint %i / %i: '  %
                    ((i + 1), self.param['nb_joints'])
                ))
            jposes.append(jpose)
        kf_new = (fnum, jposes)
        self.kf.append(kf_new)
        sort_kframes(self.kf)
        is_frame0(self.kf)

    def do_kframe_save(self, line):
        '''Save the current key frames'''
        logging.debug('Call function do_kframe_save()')
        fname = self.param['wname'] + '_kf.txt' 
        data = {'param': self.param, 'kf': self.kf}
        f = open(fname, 'w')
        pickle.dump(data, f)
        f.close()
        is_frame0(self.kf)
        
    def do_kframe_open(self, line):
        '''Open a key frames file'''
        # TODO: load parameter from the dump (see above)
        logging.debug('Call function do_kframe_open()')
        fname = parse_kframe_open(line)
        if fname:
            f = open(fname, 'r')
            data = pickle.load(f) 
            f.close()
            self.param = data['param']
            self.kf = data['kf']
            sort_kframes(self.kf)
            logging.debug('Key frames loaded: %s', self.kf)
            print('Use kframe_disp to check the content')
        is_frame0(self.kf)
                
    def do_seq_disp(self, line):
        '''Display the sequence of motion with interpolation'''
        logging.debug('Call function seq_disp()')
        if not can_gen_seq(self.kf): return
        (sfi, sfp, sft) = get_seq(self.kf)
        print sfi
        print sfp
        print sft
        # TODO: improve display

    def do_seq_plot(self, line):
        '''Plot the full sequence including the interpolated frames'''
        logging.debug('Call function seq_plot()')
        if not can_gen_seq(self.kf): return
        (sfi, sfp, sft) = get_seq(self.kf)
        plt.plot(sfi, sfp)
        plt.show(block=False)
        
    def do_seq_export(self, line):
        '''Export the full sequence including the interpolated frames'''
        logging.debug('Call function seq_export()')
        if not can_gen_seq(self.kf): return
        (sfi, sfp, sft) = get_seq(self.kf)
        export = {self.jnames[0]: {} } 
        for fi in sfi:
            export[self.jnames[0]][int(fi)] = round(sfp[fi],3)
        f = open('sequence.txt', 'w')
        f.write(json.dumps(export))
        f.close()

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
