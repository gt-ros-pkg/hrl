import roslib; roslib.load_manifest('trf_learn')
import rospy
import hrl_lib.util as ut
from trf_learn.recognize_3d import *
import sys
import pdb
import hrl_lib.rutils as ru
import os.path as pt
import glob

#====================================
#Extract image patches from raw dataset
#Save it to something
def convert_really_old_data_format_to_newer(dirname, ext):
    print 'converting data in', dirname
    data_files = glob.glob(pt.join(dirname, '*dataset.pkl'))
    print 'Found %d scans' % len(data_files)

    #data_files = [data_files[0]]
    for n in data_files:
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Loading', n
        cn = convert_scan(n, ext)
        print 'Saved to', cn
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'

def convert_scan(name, ext):
    data = ut.load_pickle(name)
    #['touch_point', 'right_cal', 'high_res', 'left_cal', 'points_laser', 'left_image', 'pro_T_bl', 'prosilica_cal', 'laser_T_bl', 'point_touched', 'right_image']
    #pdb.set_trace()
    data_pkl = {}
    data_pkl['touch_point'] = data['touch_point'] 
    data_pkl['image']       = data['high_res']
    data_pkl['k_T_bl']      = data['pro_T_bl']
    data_pkl['points3d']    = ru.pointcloud_to_np(data['points_laser'])
    data_pkl['cal']         = data['prosilica_cal']
    converted_name = pt.splitext(name)[0] + ext
    ut.save_pickle(data_pkl, converted_name)
    return converted_name

def extract_image_patches_from_raw_dataset(dirname, ext):
    rospy.init_node('extract_features', anonymous=True)
    print 'Preprocessing data in', dirname
    data_files = glob.glob(pt.join(dirname, '*_newer_format.pkl'))
    print 'Found %d scans' % len(data_files)
    for n in data_files:
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Loading', n
        cn = preprocess_scan_extract_features(n, ext)
        print 'Saved to', cn
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'

##
# At this stage it seems that we need to launch the ScanLabeler GUI, label the
# first dataset then run the automatic labeler.
def show_dataset(dirname, ext):
    s = ScanLabeler(dirname, ext=ext)
    s.run_gui()

#def label_data(name):

#====================================
#
#for d in data.metadata:
#    print 'metadata name', d.name
#print data.inputs.shape
#data.select_features(['intensity'])
#print data.inputs.shape
#
#pdb.set_trace()

if __name__ == '__main__':
    convert_really_old_data_format_to_newer(sys.argv[1], ext='_newer_format.pkl')
    extract_image_patches_from_raw_dataset(sys.argv[1], ext='_preprocessed_and_has_image_patches.pkl')
    show_dataset(sys.argv[1], '_preprocessed_and_has_image_patches.pkl')









