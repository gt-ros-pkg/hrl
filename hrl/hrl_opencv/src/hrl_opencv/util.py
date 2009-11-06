import roslib
roslib.load_manifest('hrl_opencv')
import cv

def print_mat(m):
    for j in range(m.height):
        for i in range(m.width):
            print m[j,i], ' ',
        print ''
