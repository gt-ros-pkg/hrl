#!/usr/bin/env python

import sys
import csv
import numpy as np
import math
import matplotlib.pyplot as plt
import pprint

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
            'SADNESS': [1,1,1]}

def parse(files):
    data = []
    for data_file in files:
        with open(data_file, 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                data.append(row)

    legend_labels = []
    for dat in data:
        dat[2]=float(dat[2])
        dat[3]=float(dat[3])
        dat[4]=float(dat[4])
        dat.append((dat[3]**2. + dat[4]**2.)**(1./2))
        dat.append(math.atan2(dat[4], dat[3]))
        color = tuple(ACTIONS[dat[1]]+[DEGREES[dat[0]]])
        if dat[1] not in legend_labels:
            legend_labels.append(dat[1])
            plt.polar(dat[-1], dat[-2], '.', color=color, label=dat[1])
        else:
            plt.polar(dat[-1], dat[-2], '.', color=color)
        
    #pprint.pprint(data)
    plt.legend(loc=2,bbox_to_anchor=(1,1))
    plt.show()
       


if __name__=='__main__':
    files = sys.argv[1:]
    print "files: ", files
    parse(files)
