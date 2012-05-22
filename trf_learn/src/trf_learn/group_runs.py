#!/usr/bin/python
import roslib; roslib.load_manifest('trf_learn')
import sys
import hrl_lib.util as ut
import trf_learn.recognize_3d as r3d
import time
import glob
import shutil
import os.path as pt
import os


def group_case(storage_path, pickle_name):
    p = ut.load_pickle(pickle_name)
    high_res_base_name = pt.split(p['image'])[1]
    master_dir = pt.join(storage_path, pt.splitext(high_res_base_name)[0])

    try:
        print 'making', master_dir
        os.mkdir(master_dir)
    except OSError, e:
        print e

    #make directory 
    tpickle   = time_from_name(name_no_ext(pickle_name), "_execute")
    tlatest   = tpickle + 20

    #copy the 1) pickle, 2) jpg, and 3) original high resolution image so we can reconstruct what's going on

    #copy pickle
    try:
        shutil.copyfile(pickle_name, pt.join(master_dir, pt.split(pickle_name)[1]))
    except IOError, e:
        print '                   ',
        print 'something wrong with copying file', pickle_name

    #Copy jpg
    #Look for the execute pick
    execute_picks = glob.glob(pt.split(pickle_name)[0] + '/*execute_pick.jpg')
    exec_pick_names = []
    for f in execute_picks:
        filename = name_no_ext(f)
        tf = time_from_name(filename, '_execute_pick')
        if (tf >= tpickle) and (tf <= tlatest):
            exec_pick_names.append(f)
            try:
                shutil.copyfile(f, pt.join(master_dir, pt.split(f)[1]))
            except IOError, e:
                print '                   something wrong with copying file', f

    #Copy highres
    highres_name = pt.join(pt.split(pickle_name)[0], high_res_base_name)
    try:
        shutil.copyfile(highres_name, pt.join(master_dir, high_res_base_name))
    except IOError, e:
        print '                   ',
        print 'something wrong with copying file', highres_name

if __name__ == '__main__':
    import sys
    import optparse
    storage_path = sys.argv[1]
    if len(sys.argv) > 1:
        for i in range(2, len(sys.argv)):
            group_case(storage_path, sys.argv[i])
