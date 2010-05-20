# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## @author Hai Nguyen/hai@gatech.edu
#import opencv as cv
#import opencv.highgui as hg
import util as ut
import dimreduce as dr
import numpy as np
import pickle as pk
import math
import exceptions

def load_pickle(filename):
    p = open(filename, 'r')
    picklelicious = pk.load(p)
    p.close()
    return picklelicious

def reconstruct_input(nparray):
    #split
    npbig   = nparray[0:675].A
    npsmall = nparray[675:].A

    #reshape
    npbig_rs = npbig.reshape((15,15,3))
    npsml_rs = npsmall.reshape((3,3,3))

    #convert to cv
    img     = ut.np2cv(npbig_rs.astype(np.uint8))
    context = ut.np2cv(npsml_rs.astype(np.uint8))
    return img, context

def normalize_for_display(vec):
    return (255 * ((vec - np.min(vec)) / np.max(vec)))

def scale_image(img, scale):
    new_img = cv.CreateImage((img.width*scale, img.height*scale), img.depth, img.nChannels)
    cv.Resize(img, new_img, cv.CV_INTER_NN)
    return new_img

def tile_images(img_width, img_height, num_width, num_height, images, channels=3):
    w = img_width * num_width
    h = img_height * num_height
    image = cv.CreateImage((int(w), int(h)), 8, channels)
    cv.Set(image, cv.Scalar(255,255,255))
    while len(images) > 0:
        try:
            for y in range(int(num_height)):
                for x in range(int(num_width)):
                    small_tile = images.pop()
                    img_x = x * img_width
                    img_y = y * img_height
                    cropped = cv.GetSubRect(image, cv.Rectangle(img_x, img_y, img_width,img_height))
                    cv.Copy(small_tile, cropped)
        except exceptions.IndexError, e:
            break
    return image

def display(vec, name):
    patch, context = reconstruct_input(vec)
    patch = scale_image(patch, 5)
    context = scale_image(context, 5)
    hg.cvSaveImage(name + '_patch.png', patch)
    hg.cvSaveImage(name + '_context.png', context)
    hg.cvShowImage('image', patch)
    hg.cvShowImage('context', context)
    hg.cvWaitKey()

def tile_nsave(vecs):
    patches = []
    for i in xrange(vecs.shape[1]):
        patch, context = reconstruct_input(vecs[:,i])
        patches.append(patch)
    tile_width = math.ceil(math.pow(vecs.shape[1], .5))
    large_image = tile_images(15, 15, tile_width, tile_width, patches, 3)
    return large_image

show_pca                     = False
save_pca_bases               = False 
process_inputs               = False 
separate_negatives_positives = True
dataset = load_pickle('PatchClassifier.dataset.pickle')
hg.cvNamedWindow('image', 1)
hg.cvNamedWindow('context', 1)
pca_basis = normalize_for_display(dataset.projection_basis)

if show_pca:
    for i in range(pca_basis.shape[1]):
        print 'basis', i
        display(pca_basis[:,i], 'pca_basis'+str(i))

if save_pca_bases:
    large_image = tile_nsave(pca_basis)
    large_image = scale_image(large_image, 8)
    hg.cvSaveImage('pca_large_image.png', large_image)

if process_inputs:
    large_image = tile_nsave(dataset.inputs)
    hg.cvSaveImage('inputs.png', large_image)

if separate_negatives_positives:
    r, c = np.where(dataset.outputs == 0)
    negatives = tile_nsave(dataset.inputs[:,c.A[0]])
    r, c = np.where(dataset.outputs == 1)
    positives = tile_nsave(dataset.inputs[:,c.A[0]])
    hg.cvSaveImage('negatives.png', negatives)
    hg.cvSaveImage('positives.png', positives)


















































































#projection_vectors = dr.pca_vectors(dataset.inputs, 0.95)
#projection_vectors = normalize_for_display(projection_vectors)
#print 'number of projection bases', projection_vectors.shape

#image = hg.cvLoadImage('1frame489.png')
#subrect = cv.cvCloneImage(cv.cvGetSubRect(image, cv.cvRect(40, 40, 20, 30)))
#hg.cvShowImage('image', image)
#hg.cvWaitKey()
#
#hg.cvShowImage('image', subrect)
#print 'original'
#hg.cvWaitKey()
#
#npsub   = ut.cv2np(subrect, 'BGR')
#img     = ut.np2cv(npsub.reshape((30,20,3)))
#hg.cvShowImage('image', img)
#print 'reconstructed'
#hg.cvWaitKey()

#for i in range(dataset.num_examples()):
#    patch, context = reconstruct_input(dataset.inputs[:,i])
#    hg.cvShowImage('image', patch)
#    hg.cvShowImage('context', context)
#    hg.cvWaitKey(33)































#PKG = 'laser_interface'
#import sys, os, subprocess
#try:
#    rostoolsDir = (subprocess.Popen(['rospack', 'find', 'rostools'], stdout=subprocess.PIPE).communicate()[0] or '').strip()
#    sys.path.append(os.path.join(rostoolsDir,'src'))
#    import rostools.launcher
#    rostools.launcher.updateSysPath(sys.argv[0], PKG, bootstrapVersion="0.6")
#except ImportError:
#    print >> sys.stderr, "\nERROR: Cannot locate rostools"
#    sys.exit(1)  
#
##from pkg import *
#import rospy
#from std_msgs.msg import Position
#from std_msgs.msg import RobotBase2DOdom
#import sys
#
#def debug_me(p):
#    print 'received', p.pos.x, p.pos.y, p.pos.th
#rospy.TopicSub('odom', RobotBase2DOdom, debug_me)
#rospy.ready('test')
#rospy.spin()
#def swap_br(npimage):
#    b = npimage[:,:,0].copy()
#    r = npimage[:,:,2].copy()
#    npimage[:,:,0] = r
#    npimage[:,:,2] = b
#    return npimage
#
#image   = hg.cvLoadImage('1frame489.png')
#npimg   = ut.cv2np(image)
#swapped = swap_br(npimg.copy())
#cvimg   = ut.np2cv(swapped)
#
#hg.cvNamedWindow('hai', 1)
#hg.cvShowImage('hai', cvimg)
#hg.cvWaitKey(10)
#
#





















