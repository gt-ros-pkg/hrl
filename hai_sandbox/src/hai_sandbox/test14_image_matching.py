import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import hrl_camera.ros_camera as rc
import cv
import hai_sandbox.features as fea
import sys
import threading as thr
import scipy.spatial as sp
import numpy as np
import pdb

class ShowImage(thr.Thread):
    def __init__(self, name):
        thr.Thread.__init__(self)
        self.image = None
        self.name = name

    def run(self):
        while not rospy.is_shutdown():
            if self.image != None:
                cv.ShowImage(self.name, self.image)
                cv.WaitKey(33)

def concat_images(a, b):
    img_height = max(a.height, b.height)
    c = cv.CreateImage((a.width+b.width, img_height), a.depth, a.channels)
    a_area = cv.GetSubRect(c, (0,0, a.width, a.height))
    b_area = cv.GetSubRect(c, (a.width, 0, b.width, b.height))
    cv.Add(a, a_area, a_area)
    cv.Add(b, b_area, b_area)
    return c


#class SURFMatcher:
#    def __init__(self):
#        self.model_images = {}
#        self.model_fea = {}
#
#    def add_file(self, model_name, label): 
#        model_img = cv.LoadImage(model_name)
#        self.add_model(model_img, label)
#
#    def add_model(self, model_img, label):
#        mgray = fea.grayscale(model_img)
#        m_loc, m_desc = fea.surf(mgray)
#        self.model_images[label] = model_img
#        self.model_fea[label] = {'loc': m_loc, 'desc': m_desc}
#
#    def build_db(self):
#        fea_l = []
#        labels_l = []
#        locs_l = []
#        for k in self.model_fea:
#            fea_l.append(np.array(self.model_fea[k]['desc']))
#            locs_l.append(np.array(self.model_fea[k]['loc']))
#            labels_l.append(np.array([k for i in range(len(self.model_fea[k]['desc']))]))
#
#        self.labels = np.row_stack(labels_l)
#        self.locs = np.row_stack(locs_l)
#        self.tree = sp.KDTree(np.row_stack(fea_l))
#
#    def match(self, desc, thres=.6):
#        dists, idxs = self.tree.query(np.array(desc), 2)
#        ratio = dists[0] / dists[1]
#        if ratio < threshold:
#            desc = self.tree.data[idxs[0]]
#            loc = self.locs[idxs[0]]
#            return desc, loc
#        else:
#            return None


def match_images(model_img, cand_img, threshold=.8):
    #pdb.set_trace()
    mgray = fea.grayscale(model_img)
    cgray = fea.grayscale(cand_img)
    
    m_loc, m_desc = fea.surf(mgray)
    dirs = [direction for loc, lap, size, direction, hess in m_loc]
    print 'max min dirs', np.min(dirs), np.max(dirs)

    c_loc, c_desc = fea.surf(cgray)
    
    features_db = sp.KDTree(np.array(m_desc))
    matched = []
    for i, desc in enumerate(c_desc):
        dists, idxs = features_db.query(np.array(desc), 2)
        ratio = dists[0] / dists[1]
        #print "%d %.4f" % (i, ratio),
        if ratio < threshold:
            matched.append((i, idxs[0]))
            #print 'matched!', idxs[0]
        #else:
        #    print 'X|'
    
    c_loc_moved = []
    for loc, lap, size, d, hess in c_loc:
        x, y = loc
        nloc = (x + model_img.width, y)
        c_loc_moved.append((nloc, lap, size, d, hess))
    
    c_loc_matched, m_loc_matched = zip(*[[c_loc_moved[i], m_loc[j]] for i, j in matched])
    joint = concat_images(model_img, cand_img)
    
    joint_viz = joint
    #joint_viz = fea.draw_surf(joint, c_loc_moved, (255,0,0))
    #joint_viz = fea.draw_surf(joint_viz, c_loc_matched, (0,255,0))
    #joint_viz = fea.draw_surf(joint_viz, m_loc, (255,0,0))
    #joint_viz = fea.draw_surf(joint_viz, m_loc_matched, (0,255,0))
    for cloc, mloc in zip(c_loc_matched, m_loc_matched):
        cloc2d, _, _, _, _ = cloc
        mloc2d, _, _, _, _ = mloc
        cv.Line(joint_viz, cloc2d, mloc2d, (0,255,0), 1, cv.CV_AA)
    print '%d matches found' % len(matched)
    
    return joint_viz

def test_thresholds():
    model_name = sys.argv[1]
    candidate = sys.argv[2]
    
    model_img = cv.LoadImage(model_name)
    cand_img = cv.LoadImage(candidate)
    
    for i in range(5):
        thres = .8 - (i * .1)
        print 'thres %.2f' % thres
        joint_viz = match_images(model_img, cand_img, thres)
        win_name = 'surf%.2f' % thres
        cv.NamedWindow(win_name, 0)
        cv.ShowImage(win_name, joint_viz)
    
    while not rospy.is_shutdown():
        cv.WaitKey(10)

###############
##############

if __name__ == '__main__':
    mode = 'image'
    #if mode = 'image':
    #    find pose of model
    #    find normal of model
    #    record angles of features.

    if mode=='image':
        test_thresholds()

    if mode=='live':
        model_name = sys.argv[1]
        model_img = cv.LoadImage(model_name)
        model_gray = fea.grayscale(model_img)
        msurf_loc, msurf_desc = fea.surf(model_gray)
        prosilica = rc.Prosilica('prosilica', 'streaming')
        cv.NamedWindow('surf', 1)
        si = ShowImage('surf')
        si.start()

        #Each feature is a row in db
        features_db = sp.KDTree(np.array(msurf_desc))
        #pdb.set_trace()
 
        while not rospy.is_shutdown():
            print '..'
            image = prosilica.get_frame()
            print 'saving image'
            cv.SaveImage('frame.png', image)
            print '>'
            img_gray = fea.grayscale(image)
            locs, descs = fea.surf(img_gray)
            match_idxs = []
            for i, desc in enumerate(descs):
                dists, idxs = features_db.query(np.array(desc), 2)
                ratio = dists[0] / dists[1]
                if ratio < .49:
                    match_idxs.append(i)
            img_viz = fea.draw_surf(image, locs, (255,0,0))
            img_viz = fea.draw_surf(img_viz, [locs[i] for i in match_idxs], (0,0,255))
            si.image = img_viz
            print '%d matches found' % len(match_idxs)
            #print len(desc), desc.__class__, len(descs[0]), descs[0].__class__
            #si.image = image

   















