import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy
import numpy as np
import pr2_msgs.msg as pm


class PressureListener:
    def __init__(self, topic='/pressure/l_gripper_motor', safe_pressure_threshold = 4000):
        rospy.Subscriber(topic, pm.PressureState, self.press_cb)
        self.lmat  = None
        self.rmat  = None
        self.lmat0 = None
        self.rmat0 = None
        self.lmat_raw = None
        self.rmat_raw = None

        self.safe_pressure_threshold = safe_pressure_threshold
        self.exceeded_safe_threshold = False

        self.threshold = None
        self.exceeded_threshold = False

    def rezero(self):
        self.lmat0 = None
        self.rmat0 = None
        self.exceeded_threshold = False
        self.exceeded_safe_threshold = False

    def check_safety_threshold(self):
        r = self.exceeded_safe_threshold
        self.exceeded_safe_threshold = False #reset 
        return r

    def check_threshold(self):
        r = self.exceeded_threshold
        self.exceeded_threshold = False
        return r

    def set_threshold(self, threshold):
        self.threshold = threshold

    def get_pressure_readings(self):
        return self.lmat, self.rmat

    def press_cb(self, pmsg):
        lmat = np.matrix((pmsg.l_finger_tip)).T
        rmat = np.matrix((pmsg.r_finger_tip)).T
        if self.lmat0 == None:
            self.lmat0 = lmat
            self.rmat0 = rmat
            return
    
        self.lmat_raw = lmat
        self.rmat_raw = rmat

        lmat = lmat - self.lmat0
        rmat = rmat - self.rmat0
       
        #touch detected
        if np.any(np.abs(lmat) > self.safe_pressure_threshold) or np.any(np.abs(rmat) > self.safe_pressure_threshold):
            self.exceeded_safe_threshold = True

        if self.threshold != None and (np.any(np.abs(lmat) > self.threshold) or np.any(np.abs(rmat) > self.threshold)):
            #print 'EXCEEDED threshold', self.threshold
            #print 'PressureListener: ', np.max(np.abs(lmat)), np.max(np.abs(rmat)), 'threshold', self.threshold
            self.exceeded_threshold = True

        self.lmat = lmat
        self.rmat = rmat


