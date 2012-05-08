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
def convert_really_old_data_format_to_newer(dirname, dir_save):
    print 'converting data in', dirname
    data_files = glob.glob(pt.join(dirname, '*dataset.pkl'))
    print 'Found %d scans' % len(data_files)

    #data_files = [data_files[0]]
    for n in data_files:
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Loading', n
        cn = convert_scan(n, dir_save)
        print 'Saved to', cn
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'

def convert_scan(name, dir_save):
    data = ut.load_pickle(name)
    #['touch_point', 'right_cal', 'high_res', 'left_cal', 'points_laser', 'left_image', 
    # 'pro_T_bl', 'prosilica_cal', 'laser_T_bl', 'point_touched', 'right_image']
    #pdb.set_trace()
    data_pkl = {}
    data_pkl['touch_point'] = data['touch_point'] 
    data_pkl['image']       = data['high_res']
    data_pkl['k_T_bl']      = data['pro_T_bl']
    data_pkl['points3d']    = ru.pointcloud_to_np(data['points_laser'])
    data_pkl['cal']         = data['prosilica_cal']

    #pdb.set_trace()
    #converted_name = pt.splitext(name)[0] + '.pkl'
    save_name = pt.join(dir_save, pt.split(name)[1])
    ut.save_pickle(data_pkl, save_name)
    return save_name

def extract_image_patches_from_raw_dataset(dirname, ext):
    rospy.init_node('extract_features', anonymous=True)
    print 'Preprocessing data in', dirname
    data_files = glob.glob(pt.join(dirname, '*_dataset.pkl'))
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
def show_dataset(dirname, ext, scan_to_train_on):
    #take one or more dataset, using distance features, create a classifier
        #Doesn't the default scan labeler do this>
    #use this classifier to label others.
    s = ScanLabeler(dirname, ext, scan_to_train_on, None, ['distance'])
    s.run_gui()

def label_fiducials(filename):
    f = FiducialPicker(filename)
    f.run()

def label_data(dirname, ext, scan_to_train_on):
    #take one or more dataset, using distance features, create a classifier
        #Doesn't the default scan labeler do this>
    #use this classifier to label others.
    s = ScanLabeler(dirname, ext, scan_to_train_on, None, ['distance'])
    s.automatic_label()


if __name__ == '__main__':
    #Step 0
    #convert_really_old_data_format_to_newer(sys.argv[1], dir_save='newer_format')

    #Step 1
    #extract_image_patches_from_raw_dataset(sys.argv[1], ext='_preprocessed_and_has_image_patches.pkl')
    #label_fiducials(sys.argv[1])

    #Step 2
    #Use this to generate a training dataset, 
    # First label everything as negative with 0 then click on the few positive points.
    # Switch to training set mode, click on a positive and negative data point, 
    # Train with the key 'r', click to add points to the training set and repeat retraining using 'r'
    # until display shows the correct concept.
    # Then save data with 's'.
    #show_dataset(sys.argv[1], '_preprocessed_and_has_image_patches.pkl', sys.argv[2])
    
    #Step 3
    label_data(sys.argv[1], '_preprocessed_and_has_image_patches.pkl', sys.argv[2])









