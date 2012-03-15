#! /usr/bin/python

import sys
import cPickle as pickle
import numpy as np

        

def main():

    pose_dict = {
            #   lat   lon    height    roll   pitch   yaw
             "near_ear" : [(4 * np.pi/8,   -3 * np.pi/8,     1),      (0,     0,      0)],
             "upper_cheek" : [(4 * np.pi/8,   -1.5 * np.pi/8,     1),      (0,     0,      0)],
             "middle_cheek" : [(4.5 * np.pi/8,   -2 * np.pi/8,     1),      (0,     0,      0)],
             "jaw_bone" : [(5.1 * np.pi/8,   -2 * np.pi/8,     1),      (0,     0,      0)],
             "back_neck" : [(5.1 * np.pi/8,   -3 * np.pi/8,     1),      (0,     0,      0)],
             "nose" : [(4 * np.pi/8,    0 * np.pi/8,     1),      (0,     0,      0)],
             "chin" : [(5.4 * np.pi/8,    0 * np.pi/8,     1),      (0 * np.pi / 2,    np.pi/8,      0)],
             "mouth_corner" : [(4.5 * np.pi/8,   -0.9 * np.pi/8,     1),      (0,     0,      0)]
            }
            
    
    f = file('../../data/piclkled_poses.pkl', 'w')
    pickle.dump(pose_dict, f)
    f.close()

if __name__ == "__main__":
    main()
