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
import os
import sys
import camera as cam
import re
import time
import random_forest as rf
#import opencv as cv
#import opencv.highgui as hg
import cv
import util as ut
from laser_detector import *

def count_down(times=3):
    for i in range(times):
        print 'in ', times - i
        time.sleep(1)

def evaluate_classifier():
    rf.evaluate_classifier(rf.RFBreiman, load_pickle('PatchClassifier.dataset.pickle'), 1, .7)

def show_processed(image, masks, detection, blobs, detector):
    masker            = Mask(image)
    splitter          = SplitColors(image)
    r, g, b           = splitter.split(image)
    thresholded_image = masker.mask(masks[0], r, g, b)
    draw_detection(thresholded_image, detection)
    cv.ShowImage('thresholded', thresholded_image)

    draw_detection(image, detection)
    draw_blobs(image, blobs)

    make_visible_binary_image(masks[0])
    draw_detection(masks[0], detection)
    make_visible_binary_image(masks[1])
    make_visible_binary_image(masks[2])

    cv.ShowImage("video",       image)
    cv.ShowImage('motion',      masks[1])
    cv.ShowImage('intensity',   masks[2])

    key = cv.WaitKey(10)
    if detector != None:
        if key == 'T': #down
            detector.intensity_filter.thres_high = detector.intensity_filter.thres_high - 5
            print 'detector.intensity_filter.thres =', detector.intensity_filter.thres_high
        if key == 'R':
            detector.intensity_filter.thres_high = detector.intensity_filter.thres_high + 5
            print 'detector.intensity_filter.thres =', detector.intensity_filter.thres_high
        if key == ' ':
            cv.WaitKey()

def confirmation_prompt(confirm_phrase):
    print confirm_phrase
    print 'y(es)/n(no)'
    k = cv.WaitKey()
    if k == 'y':
        return True
    else:
        return False

def learn_run(exposure = LaserPointerDetector.SUN_EXPOSURE, num_examples_to_collect=200, display_during_run = True):
    cv.NamedWindow("video",       1)
    cv.NamedWindow("thresholded", 1)
    cv.NamedWindow('motion',      1)
    cv.NamedWindow('intensity',   1)

    cv.MoveWindow("video",       0,   0)
    cv.MoveWindow("thresholded", 800, 0)

    cv.MoveWindow("intensity",   0,   600)
    cv.MoveWindow("motion",      800, 600)

    #TODO: plug in some image source
    #video     = cam.VidereStereo(0, gain=96, exposure=exposure)
    frames    = video.next()
    detector  = LaserPointerDetector(frames[0], exposure=exposure, 
                                    dataset=PatchClassifier.DEFAULT_DATASET_FILE,
                                    use_color=False, use_learning=False)
    detector2 = LaserPointerDetector(frames[1], exposure=exposure, 
                                    dataset=PatchClassifier.DEFAULT_DATASET_FILE,
                                    use_color=False, use_learning=False)

    def append_examples_to_file(dataset, file = PatchClassifier.DEFAULT_DATASET_FILE):
        try:
            loaded_set = load_pickle(file)
            dataset.append(loaded_set)
        except IOError:
            pass
        dump_pickle(dataset, file)
        print 'Saved examples!'

    #Gather positive examples from laser detector
    if confirmation_prompt('gather positive examples?'):
        print 'Lets gather some positive examples... we need', num_examples_to_collect, 'examples'
        positive_examples_for_classifier = []
        count_down(0)
        for i in xrange(10):
            frames = video.next()
            detector.detect(frames[0])
            detector2.detect(frames[1])

        for img in video:
            image                 = None
            combined              = None
            motion                = None
            intensity             = None
            laser_blob            = None
            intensity_motion_blob = None

            for raw_image, detect in zip(img, [detector, detector2]):
                before = time.time()
                image, combined, laser_blob, intensity_motion_blob = detect.detect(raw_image)
                diff = time.time() - before
                #print 'took %.2f seconds to run or %.2f fps' % (diff, 1.0/diff)
                if laser_blob != None:
                    instance = blob_to_input_instance(image, laser_blob)
                    if instance is not None:
                        positive_examples_for_classifier.append(instance)
                        print 'got', len(positive_examples_for_classifier), 'instances'
                motion, intensity = detect.get_motion_intensity_images()

            show_processed(image, [combined, motion, intensity], laser_blob, intensity_motion_blob, detector2)
            if len(positive_examples_for_classifier) > num_examples_to_collect:
                break
        positive_instances_dataset = matrix_to_dataset(ut.list_mat_to_mat(positive_examples_for_classifier, axis=1))
        append_examples_to_file(positive_instances_dataset)

    if confirmation_prompt('gather negative examples?'):
        #Gather negative examples from laser detector
        print 'lets gather some negative examples... we need', num_examples_to_collect,' examples'
        negative_examples_for_classifier = []
        count_down(10)
        for i in xrange(10):
            frames = video.next()
            detector.detect(frames[0])
            detector2.detect(frames[1])

        for img in video:
            image                 = None
            combined              = None
            motion                = None
            intensity             = None
            laser_blob            = None
            intensity_motion_blob = None
            for raw_image, detect in zip(img, [detector, detector2]):
                image, combined, laser_blob, intensity_motion_blob = detect.detect(raw_image)
                if laser_blob != None:
                    instance = blob_to_input_instance(image, laser_blob)
                    if instance is not None:
                        negative_examples_for_classifier.append(instance)
                        print 'got', len(negative_examples_for_classifier), 'instances'
                motion, intensity = detect.get_motion_intensity_images()

            show_processed(image, [combined, motion, intensity], laser_blob, intensity_motion_blob, detector2)
            if len(negative_examples_for_classifier) > (num_examples_to_collect*2):
                break
        negative_instances_dataset = matrix_to_dataset(ut.list_mat_to_mat(negative_examples_for_classifier, axis=1))
        append_examples_to_file(negative_instances_dataset)

    if confirmation_prompt('run classifier?'):
        run(exposure, video = video, display=display_during_run)

def run(exposure, video=None, display=False, debug=False):
    if display:
        cv.NamedWindow("video", 1)
        cv.MoveWindow("video",   0,   0)

    if debug:
        cv.NamedWindow('right',       1)
        cv.MoveWindow("right", 800,   0)
        cv.NamedWindow("thresholded", 1)
        cv.NamedWindow('motion',      1)
        cv.NamedWindow('intensity',   1)

        cv.MoveWindow("thresholded", 800, 0)
        cv.MoveWindow("intensity",   0,   600)
        cv.MoveWindow("motion",      800, 600)

    if video is None:
        #video    = cam.VidereStereo(0, gain=96, exposure=exposure)
        video    = cam.StereoFile('measuring_tape_red_left.avi','measuring_tape_red_right.avi')

    frames = video.next()
    detector       = LaserPointerDetector(frames[0], LaserPointerDetector.SUN_EXPOSURE, 
                                            use_color=False, use_learning=True)
    detector_right = LaserPointerDetector(frames[1], LaserPointerDetector.SUN_EXPOSURE, 
                                            use_color=False, use_learning=True, classifier=detector.classifier)
    stereo_cam     = cam.KNOWN_CAMERAS['videre_stereo2']

    for i in xrange(10):
        frames = video.next()
        detector.detect(frames[0])
        detector_right.detect(frames[1])

    lt = cv.CreateImage((640,480), 8, 3)
    rt = cv.CreateImage((640,480), 8, 3)
    for l, r in video:
        start_time = time.time()
        #l = stereo_cam.camera_left.undistort_img(l)
        #r = stereo_cam.camera_right.undistort_img(r)
        cv.Copy(l, lt)
        cv.Copy(r, rt)
        l = lt
        r = rt
        undistort_time = time.time()

        _, _, right_cam_detection, stats = detector_right.detect(r)
        if debug:
            draw_blobs(r, stats)
            draw_detection(r, right_cam_detection)
            cv.ShowImage('right', r)

        image, combined, left_cam_detection, stats = detector.detect(l)
        detect_time = time.time()

        if debug: 
            motion, intensity = detector.get_motion_intensity_images()
            show_processed(l, [combined, motion, intensity], left_cam_detection, stats, detector)
        elif display:
            #draw_blobs(l, stats)
            draw_detection(l, left_cam_detection)
            cv.ShowImage('video', l)
            cv.WaitKey(10)

        if right_cam_detection != None and left_cam_detection != None:
            x  = np.matrix(left_cam_detection['centroid']).T
            xp = np.matrix(right_cam_detection['centroid']).T
            result = stereo_cam.triangulate_3d(x, xp)
            print '3D point located at', result['point'].T, 
            print 'distance %.2f error %.3f' % (np.linalg.norm(result['point']),  result['error'])
        triangulation_time = time.time()

        diff = time.time() - start_time
        print 'Main: Running at %.2f fps, took %.4f s' % (1.0 / diff, diff)
        #print '   undistort     took %.4f s' % (undistort_time - start_time)
        #print '   detection     took %.4f s' % (detect_time - undistort_time)
        #print '   triangulation took %.4f s' % (triangulation_time - detect_time)

if __name__ == '__main__':
    exposure = 0
    if sys.argv[2] == 'sun':
        exposure = LaserPointerDetector.SUN_EXPOSURE
        print 'sun exposure!'
    else:
        exposure = LaserPointerDetector.NO_SUN_EXPOSURE

    if sys.argv[1] == 'run':
        print 'Running only...'
        run(exposure = exposure, display = True)

    if sys.argv[1] == 'learn':
        print 'Running only...'
        learn_run(exposure = exposure)



































##=================================================================================================
##=================================================================================================

#def display_single(detection_source, mask_window='masked', detection_window='cam'):
#    for frame, mask, loc, blobs in detection_source:
#        make_visible_binary_image(mask)
#        hg.cvShowImage(mask_window, mask)
#        draw_detection(frame, loc)
#        hg.cvShowImage(detection_window, frame)
#        hg.cvWaitKey(10)
#        yield frame, mask, loc, blobs

#def save_detection(prefix, frame_number, detection_bundle):
#    #frame_number = 1
#    #for frame, mask, loc, blobs in detection_source:
#        frame, mask, loc, blobs = detection_bundle
#        frame_name = prefix + 'frame' + str(frame_number) + '.png'
#        hg.cvSaveImage(frame_name, frame)
#        hg.cvSaveImage(prefix + 'frame' + str(frame_number) + '_mask.png', mask)
#
#        pickle_file = open(prefix + 'frame' + str(frame_number) + '.pickle', 'w')
#        pk.dump(loc, pickle_file)
#        pickle_file.close()
#
#        pickle_file = open(prefix + 'frame' + str(frame_number) + '_blobs.pickle', 'w')
#        pk.dump(blobs, pickle_file)
#        pickle_file.close()
#
#        #frame_number = frame_number + 1
#        #yield frame, mask, loc, blobs
#        return frame, mask, loc, blobs

##=================================================================================================
##=================================================================================================

#def record_videos(stereo, prefix, folder):
#    videos = []
#    MPEG4 = 0x58564944
#    count = 0
#    while True:
#        key = hg.cvWaitKey()
#        while key != ' ':
#            key = hg.cvWaitKey()
#        left_file  = folder + '/' + prefix+str(count) + '_left.avi'
#        right_file = folder + '/' + prefix+str(count) + '_right.avi'
#        lwrite = hg.cvCreateVideoWriter(left_file,  MPEG4, 30, cvSize(640,480))
#        rwrite = hg.cvCreateVideoWriter(right_file, MPEG4, 30, cvSize(640,480))
#        for left, right in stereo:
#            hg.cvWriteFrame(lwrite, left)
#            hg.cvWriteFrame(rwrite, right)
#            hg.cvShowImage('cam', left)
#            key = hg.cvWaitKey(10)
#            if key == ' ':
#                break
#        count = count + 1
#        videos.append(left_file)
#        videos.append(right_file)
#
#        print 'Do you want to record another clip?'
#        key = hg.cvWaitKey()
#        if key == 'y':
#            continue
#        if key == 'n':
#            break
#    return videos

#def record_learn(session_name):
#    '''
#        NOTE: THIS SET OF FUNCTIONS NEED TO BE TESTED
#    '''
#    os.mkdir(session_name)
#    hg.cvNamedWindow('cam', 1)
#    sample_image = cv.cvCreateImage(cv.cvSize(640,480), 8, 3)
#    hg.cvShowImage('cam', image)
#    video = cam.VidereStereo(0, gain=96, exposure=LaserPointerDetector.SUN_EXPOSURE)
#
#    print 'Let\'s gather positive examples, press space bar then point the laser pointer',
#    print ' at various objects. Remember to not move the camera! Press spacebar again to stop.'
#    positive_files = record_videos(video, 'positive', session_name)
#
#    print 'Let\'s gather negative examples, press space bar then point the laser pointer',
#    print ' at various objects. Remember to not move the camera! Press spacebar again to stop.'
#    negative_files = record_videos(video, 'negative', session_name)
#
#    print 'Performing detections, writing out images...'
#    files_sets = [('positive/', positive_files), ('negative/', negative_files)]
#    for label, files in files_sets:
#        start_dir = session_name + '/' + label
#        os.mkdir(start_dir)
#        idx = 0
#        for pair in files:
#            for file in pair:
#                video        = cam.video_file(file)
#                detector     = LaserPointerDetector(cv.cvCreateImage(cv.cvSize(640,480), 8, 3), use_color=False, use_learning=False)
#                frame_number = 0
#                for img in video:
#                    image, mask, laser_blob, intensity_motion_blob = detector.detect(img)
#                    save_detection(start_dir + str(idx), frame_number, (image, mask, laser_blob, intensity_motion_blob))
#                    frame_number = frame_number + 1
#                idx = idx + 1
#
#    print 'running color learner'
#    run_color_learner(session_name + '/positive')
#
#    print 'running forest learner...'
#    run_forest_learner(session_name)

##=================================================================================================
##=================================================================================================

#def as_iter_block(func, input):
#    for p in input:
#        yield func(p)

#def get_frames_from_folder(folder):
#
#    def files_in(folder):
#        root, dirs, files = os.walk(folder).next()
#        return root, files
#
#    def dir_in(folder):
#        root, dirs, files = os.walk(folder).next()
#        return root, dirs
#
#    def select_frames(reg, name):
#        return None != reg.match(name)
#
#    def append_pref(pref, file):
#        return pref + '/' + file
#
#    reg = re.compile('\dframe\d*\.png')
#    root, files = files_in(folder)
#    selected_files = map(ft.partial(append_pref, root), filter(ft.partial(select_frames, reg), files))
#    return selected_files

#def run_color_learner(folder):
#    cf = ColorFilter((640,480))
#    def bw_img_loader(filename):
#        return hg.cvLoadImage(filename, hg.CV_LOAD_IMAGE_GRAYSCALE)
#    files = get_frames_from_folder(folder)
#    cf.learn_sequence(it.izip(cam.file_sequence(selected_files, hg.cvLoadImage), 
#                              cam.file_sequence(selected_files, bw_img_loader, '_mask.png'),
#                              cam.file_sequence(selected_files, load_pickle, '.pickle')))

#def run_forest_learner(folder):
#    pfiles = get_frames_from_folder(folder + '/positive')
#    nfiles = get_frames_from_folder(folder + '/negative')
#    pc = PatchClassifier()
#    pc.learn(positive_examples = it.izip(cam.file_sequence(pfiles, hg.cvLoadImage),
#                                         cam.file_sequence(pfiles, load_pickle, '_blobs.pickle')),
#             negative_examples = it.izip(cam.file_sequence(nfiles, hg.cvLoadImage),
#                                         cam.file_sequence(nfiles, load_pickle, '_blobs.pickle')))

##=================================================================================================
##=================================================================================================



    #run(exposure = LaserPointerDetector.SUN_EXPOSURE)
    #import sys
    #session_name = sys.argv[1]
    #record_learn(session_name)

##=================================================================================================
##=================================================================================================
##                                       SAND BOX
##=================================================================================================
##=================================================================================================
## ===> Sand!
#def triangulate_cressel(camera, left_coord, right_coord, return_dict = True ):
#    """ left_coord, right_coord -> cvPoint
#        returns -> (3X1) vector in stereohead coordinate frame, error.
#                   stereohead coord frame is as defined in transforms.py
#        code copied and modified slightly from self.get_3d_coord
#    """
#
#    right_point = cv.cvCreateMat(3,1,cv.CV_32F)
#    left_point = cv.cvCreateMat(3,1,cv.CV_32F)
#
#    if left_coord.__class__ == cv.cvPoint(0,0).__class__:
#        left_point[0] = float(left_coord.x)			  # u
#        left_point[1] = float(left_coord.y)			  # v
#
#        left_point[2] = 1.0
#    else:
#        left_point[0] = float(left_coord[0])			  # u
#        left_point[1] = float(left_coord[1])			  # v
#        left_point[2] = 1.0
#
#    if right_coord.__class__ == cv.cvPoint(0,0).__class__:
#        right_point[0] = float(right_coord.x)		   
#        right_point[1] = float(right_coord.y)
#        right_point[2] = 1.0
#    else:
#        right_point[0] = float(right_coord[0])		   
#        right_point[1] = float(right_coord[1])
#        right_point[2] = 1.0
#        # in the normalized image plane _point is the vector denoted as [ u; v; 1 ]
#
#    right_vector  = cv.cvCreateMat(3,1,cv.CV_32F)
#    left_vector   = cv.cvCreateMat(3,1,cv.CV_32F)
#    output_vector = cv.cvCreateMat(3,1,cv.CV_32F)
#
#    left_inv_intrinsic  = camera.camera_left.inv_intrinsic_mat_cv
#    right_inv_intrinsic = camera.camera_right.inv_intrinsic_mat_cv
#    cv.cvGEMM(left_inv_intrinsic , left_point,  1, None, 1, left_vector,  0)   
#    cv.cvGEMM(right_inv_intrinsic, right_point, 1, None, 1, right_vector, 0)
#
#    lv = np.matrix([left_vector[0], left_vector[1], left_vector[2]]).T
#    rv = np.matrix([right_vector[0], right_vector[1], right_vector[2]]).T
#
#    te = camera.R * lv
#
#    a = cv.cvCreateMat( 3, 2, cv.CV_32F )
#    a[0,0], a[1,0], a[2,0] = te[0,0], te[1,0], te[2,0]
#    a[0,1], a[1,1], a[2,1] = rv[0,0], rv[1,0], rv[2,0]
#
#    params = cv.cvCreateMat( 2, 1, cv.CV_32F )
#
#    #linear least squares [X1 -X2]*[al;bet] - trans ~ 0
#    #				  [left right]*[params] -translation ~0
#    translation = cv.cvCreateMat(3,1,cv.CV_32F)
#    translation[0] = -camera.T[0,0]
#    translation[1] = -camera.T[1,0]
#    translation[2] = -camera.T[2,0]
#
#    cv.cvSolve( a, translation, params , cv.CV_SVD )
#    alpha = params[0]
#    beta = -params[1]
#
#    vec_l = alpha * te + camera.T
#    vec_r = beta * rv
#
#    #compute reprojection error
#    detection_error = np.linalg.norm(vec_r-vec_l)
#
#    p = (vec_l + vec_r)/2.
#
#    p = np.row_stack((p, np.matrix([1.])))
#    # my xyz and cressel's are aligned and so I can use the
#    # same transformation matrix -- advait
#    #p = tr._stereoT['right'] * p
#    p = p[0:3]/p[3,0]
#
#    if return_dict:
#        return {'output_vector': p, 'error':detection_error}
#    else:
#        return p, detection_error
##=================================================================================================
##=================================================================================================
