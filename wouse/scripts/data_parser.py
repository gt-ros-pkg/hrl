#!/usr/bin/env python

import sys
import csv
import numpy as np
import math
import matplotlib.pyplot as plt
import pprint
import pickle

DEGREES = {'WEAK'   : 0.33,
           'AVERAGE': 0.66,
           'STRONG' : 1.0}

ACTIONS = {'WINCE'  : [0,0,0],
            'SMILE' : [0.5,0,0] ,
            'FROWN' : [0,0.5,0],
            'LAUGH' : [0,0,0.5],
            'GLARE' : [0.5,0.5,0],
            'NOD'   : [0.5,0,0.5],
            'SHAKE' : [0,0.5,0.5],
            'REQUEST FOR BOARD': [0.5,0.5,0.5],
            'EYE-ROLL':[1,0,0],
            'JOY'   :  [0,1,0],
            'SUPRISE': [0,0,1],
            'FEAR'  :  [1,1,0],
            'ANGER' :  [0,1,1],
            'DISGUST': [1,0,1],
            'SADNESS': [0.5,0,0]}
ACT_LIST = ['WINCE', 'NOD', 'SHAKE', 'JOY', "FEAR", "SUPRISE", "ANGER", "DISGUST", "SADNESS"]
WINDOW_DUR = 0.25

def parse(files):
    data = []
    for data_file in files:
        with open(data_file, 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                data.append(row)
    window = []
    legend_labels = []
    svm_label = []
    svm_data = []
    for dat in data:
        #dat[0] = Degree
        #dat[1] = Action
        dat[2]=float(dat[2]) #Timestamp (float seconds)
        dat[3]=float(dat[3]) #X
        dat[4]=float(dat[4]) #Y
        dat.append((dat[3]**2. + dat[4]**2.)**(1./2)) #dat[5] = Magnitude
        dat.append(math.atan2(dat[4], dat[3])) #dat[6] = Direction
        color = tuple(ACTIONS[dat[1]]+[DEGREES[dat[0]]])
        if (dat[5]<2.5):# or (dat[6]<1.75) or (dat[6]>3): #Initial filtering
            continue
        
        window.append(dat)
        while (window[-1][2] - window[0][2]) > WINDOW_DUR:
            window.pop(0)
        dat.append(len(window))#dat[7] = Number of points in window
        movement = [0.,0.]
        for datum in window:
            movement[0] += datum[3]
            movement[1] += datum[4]
        dat.append((movement[0]**2+movement[1]**2)**(1./2))#dat[8] = Mag of window movement
        dat.append(dat[-1]/dat[-2])#dat[9] = Avg Window Movement
        dat.append(math.atan2(movement[1],movement[0]))#dat[10] = Dir of window movement
       
        if dat[1] not in legend_labels:
            legend_labels.append(dat[1])
            #plt.polar(dat[-1], dat[-2], '.', color=color, label=dat[1])
            plt.plot(dat[-1], dat[-2], '.', color=color, label=dat[1])
            #plt.plot(dat[-3], ACT_LIST.index(dat[1]), '.', color=color, label=dat[1])
        else:
            #plt.polar(dat[-1], dat[-2], '.', color=color)
            plt.plot(dat[-1], dat[-2], '.', color=color)
            #plt.plot(dat[-3], ACT_LIST.index(dat[1]), '.', color=color)

        #OUTPUT Data in format for Sci-kit Learn
        if dat[1] == 'WINCE':
            svm_label.append(1)
        else:
            svm_label.append(0)
        svm_data.append([dat[5],dat[6],dat[7],dat[9],dat[10]])
        
    svm_output = {'labels':svm_label,
                  'data':svm_data}
#    with open('../data/svm_data.pkl','wb+') as f_pkl:
#        pickle.dump(svm_output, f_pkl)

    #pprint.pprint(data)
    plt.legend(loc=2,bbox_to_anchor=(1,1))
    plt.show()

if __name__=='__main__':
    files = sys.argv[1:]
    print "Parsing data from the following files: \r\n ", files
    parse(files)
