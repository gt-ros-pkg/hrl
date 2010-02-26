# This is highly preliminary, and includes (extremely) old /
# deprecated functions.  It will see numerous updates in the coming
# days / weeks

import roslib; roslib.load_manifest('rfid')
import rospy
import rfid.M5e as M5e
import rfid.M5e_ROS as M5e_R

# Note to self, determine which imports are actually necessary....

import robotis.robotis_servo as rs
import numpy as np, math
import time
import numpy as np, math
import pylab as pl
import scipy
import scipy.ndimage as nd
import math_util as mu
import transforms2d as t2d
import transforms as tr
import scipy.optimize as op

class pollerReads():
    ''' Class used by M5e polling thread to communicate new RFID readings. Should probably be folded into
        M5e poller sometime in the future...  '''
    def __init__(self):
        self.old = [ 'N/A', '', -1, time.time() ]  # [ Antenna Name, TagID String, RSSI, time of read ]
        self.new = list(self.old)
    def call_back(self, trackres):
        ant, id, rssi = trackres
        self.new = [ ant, id, rssi, time.time() ]
    def getNewRead(self):
        newTime = self.new[-1] # Note, this is 'atomic' (thread-safe) operation
        while newTime == self.old[-1]:
            time.sleep(0.02)
            newTime = self.new[-1] # Again, this is 'atomic'
        self.old = list(self.new) # Pretty sure this one is too... but not 100% certain
        print self.old
        return self.old

class Ears(object):
    """
       Basic pan-tilt mechanism with RFID antennas.
    """
    
    def __init__(self):
        ''' robot- 'El-E' or 'HRL2'
        '''
        #self.read = M5e.M5e(readPwr=3000)
        
        self.pan_r = rs.robotis_servo('/dev/robot/servo_right',13)
        self.tilt_r = rs.robotis_servo('/dev/robot/servo_right',14)
        
        self.pan_l = rs.robotis_servo('/dev/robot/servo_left',11)
        self.tilt_l = rs.robotis_servo('/dev/robot/servo_left',12)
        
        #self.earL.move_angle(0)
        #self.earR.move_angle(0)
        #self.pan_l.move_angle(0)
        #self.pan_r.move_angle(0)
        #self.tilt_l.move_angle(0)
        #self.tilt_r.move_angle(0)
        
        self.moving_funcs = [
            self.pan_r.is_moving,
            self.tilt_r.is_moving,
            self.pan_l.is_moving,
            self.tilt_l.is_moving ]
            
        self.reader = M5e.M5e(portSTR='/dev/robot/RFIDreader', readPwr=3000)

        #self.first_start()
        #self.fold_extra_safe()

    def EleLeftEar(self,M5e):
            M5e.ChangeAntennaPorts(2,2)
            return 'EleLeftEar'
        
    def EleRightEar(self,M5e):
        M5e.ChangeAntennaPorts(1,1)
        return 'EleRightEar'
        
    def first_start(self):
        self.pan_l.move_angle(0,angvel=math.radians(25))
        self.pan_r.move_angle(0,angvel=math.radians(25))
        self.tilt_l.move_angle(0,angvel=math.radians(25))
        self.tilt_r.move_angle(0,angvel=math.radians(25))
    
    def fold_extra_safe(self):
        # Fold Right
        self.pan_r.move_angle(math.radians(-140),angvel=math.radians(40),blocking=False)
        self.tilt_r.move_angle(math.radians(0),angvel=math.radians(40),blocking=False)
        
        # Fold Left
        self.pan_l.move_angle(math.radians(140.),angvel=math.radians(40),blocking=False)
        self.tilt_l.move_angle(math.radians(0),angvel=math.radians(40),blocking=False)
        
        while np.any( [f() for f in self.moving_funcs] ):
            t = 0
    
    def unfold_servopos(self):
        # Unfold Right
        self.pan_r.move_angle(math.radians(-45),angvel=math.radians(40),blocking=False)
        self.tilt_r.move_angle(math.radians(0),angvel=math.radians(40),blocking=False)
        
        # Unfold Left
        self.pan_l.move_angle(math.radians(45),angvel=math.radians(40),blocking=False)
        self.tilt_l.move_angle(math.radians(0),angvel=math.radians(40),blocking=False)
        
        while np.any( [f() for f in self.moving_funcs] ):
            t = 0

    def print_angles(self):
        print 'Left Ear: '
        print '\tPan: ', math.degrees( self.pan_l.read_angle() )
        print '\tTilt: ', math.degrees( self.tilt_l.read_angle() )
        print 'Right Ear: '
        print '\tPan: ', math.degrees( self.pan_r.read_angle() )
        print '\tTilt: ', math.degrees( self.tilt_r.read_angle() )


    def disable_left(self):
        self.pan_l.disable_torque()
        time.sleep(0.02)
        self.tilt_l.disable_torque()
        time.sleep(0.02)
    
    def disable_right(self):
        self.pan_r.disable_torque()
        time.sleep(0.02)
        self.tilt_r.disable_torque()
        time.sleep(0.02)
        
    def disable_all(self):
        self.disable_left()
        self.disable_right()
        
    def enable_left(self):
        self.pan_l.enable_torque()
        time.sleep(0.02)
        self.tilt_l.enable_torque()
        time.sleep(0.02)
    
    def enable_right(self):
        self.pan_r.enable_torque()
        time.sleep(0.02)
        self.tilt_r.enable_torque()
        time.sleep(0.02)
    
    def enable_all(self):
        self.enable_left()
        self.enable_right()

# RSSI Scan:

#     Right:
#         Pan: -95 (side) to 30 (toward center)
#         Tilt: -35 (up) to 45 (down)
#     Left:
#         Pan: 95 (side) to -30 (toward center)
#         Tilt: -35 (up) to 45 (down)
    
    def rssi_pan_tilt(self, tagID, pan_servo, tilt_servo, antfunc,
                      panMin, panMax, tiltMin, tiltMax,
                      panSpeed = 20, tilt_res = 8):
        ''' performs a pan-tilt scan '''
        # Right ear: rssi_pan_tilt('Claritin    ', self.pan_r, self.tilt_r, self.EleRightEar,
        #                          -95.0, 30.0, -35.0, 45.0, 20, 15)
        # Left ear:  rssi_pan_tilt('Claritin    ', self.pan_l, self.tilt_l, self.EleLeftEar,
        #                          95.0, -30.0, -35.0, 45.0, 20, 15)
        tiltVals = np.linspace(tiltMin, tiltMax, tilt_res)
        
        self.fold_extra_safe()
        
        tilt_servo.move_angle( math.radians( tiltMin ),angvel=math.radians(40),blocking=False )
        pan_servo.move_angle( math.radians( panMin ),angvel=math.radians(40),blocking=False )
                
        newRead = pollerReads()  # Ensures fresh readings
        
        self.poller = M5e.M5e_Poller(self.reader, antfuncs = [antfunc], callbacks = [newRead.call_back] )

        res = []  # [ Antenna Name, TagID String, RSSI, time of read, pan_angle, tilt_angle ]
        min2max = True
        
        self.poller.track_mode( tagID ) # Begin RFID reads as fast as possible

        for tA in tiltVals:
            tilt_servo.move_angle( math.radians(tA), angvel=math.radians(25), blocking=True )
            if min2max:
                pan_servo.move_angle( math.radians(panMax),angvel=math.radians(panSpeed),blocking=False )
                min2max = False
            else:
                pan_servo.move_angle( math.radians(panMin),angvel=math.radians(panSpeed),blocking=False )
                min2max = True

            newRead.getNewRead() # flush reader results
            while pan_servo.is_moving():
                a = newRead.getNewRead()  # [ Antenna Name, TagID String, RSSI, time of read ]
                pan_read_ang = pan_servo.read_angle()
                time.sleep(0.01)
                tilt_read_ang = tilt_servo.read_angle()
                time.sleep(0.01)

                res += [ a + [pan_read_ang, tilt_read_ang] ]

        # Turn off RFID reader.
        self.poller.pause_poller()  # Stops the thread from reading
        self.poller.stop()  # Tell the thread to die
        time.sleep(0.050)
        
        self.fold_extra_safe()
        raw_readings = list(res)
        
        return raw_readings
    
    def rssi_pan_only(self, tagID, pan_servo, tilt_servo, antfunc,
                      panMin, panMax, tiltAng, panSpeed = 20):
        # Should be panStart and panStop...
        
        # Right ear: rssi_pan_tilt('Claritin    ', self.pan_r, self.tilt_r, self.EleRightEar,
        #                          -95.0, 30.0, -35.0, 45.0, 20, 15)
        # Left ear:  rssi_pan_tilt('Claritin    ', self.pan_l, self.tilt_l, self.EleLeftEar,
        #                          95.0, -30.0, -35.0, 45.0, 20, 15)

        self.fold_extra_safe()
        
        tilt_servo.move_angle( math.radians( tiltAng ),angvel=math.radians(40),blocking=False )
        pan_servo.move_angle( math.radians( panMin ),angvel=math.radians(40),blocking=True )
                
        newRead = pollerReads()  # Ensures fresh readings
        
        self.poller = M5e.M5e_Poller(self.reader, antfuncs = [antfunc], callbacks = [newRead.call_back] )

        res = []  # [ Antenna Name, TagID String, RSSI, time of read, pan_angle, tilt_angle ]

        if tagID:
            self.poller.track_mode( tagID ) # Begin RFID reads as fast as possible
        else:
            self.poller.query_mode()

        pan_servo.move_angle( math.radians(panMax),angvel=math.radians(panSpeed),blocking=False )
        newRead.getNewRead() # flush reader results
        while pan_servo.is_moving():
            a = newRead.getNewRead()  # [ Antenna Name, TagID String, RSSI, time of read ]
            pan_read_ang = pan_servo.read_angle()
            time.sleep(0.01)
            tilt_read_ang = tilt_servo.read_angle()
            time.sleep(0.01)
            res += [ a + [pan_read_ang, tilt_read_ang] ]

        # Turn off RFID reader.
        self.poller.pause_poller()  # Stops the thread from reading
        self.poller.stop()  # Tell the thread to die
        time.sleep(0.050)
        
        self.fold_extra_safe()
        raw_readings = list(res)
        
        return raw_readings
#         ''' performs a pan-only scan '''
#         print 'HAND_WAVE IS DEPRECATED... NEEDS TO BE RE-WRITTEN!'
#         class pollerReads():
#             def __init__(self):
#                 self.old = [time.time(), -1, None]
#                 self.new = list(self.old)
#             def callB(self, trackres):
#                 ant, id, rssi = trackres
#                 self.new = [time.time(), rssi[0], ant]
#             def getNewRead(self):
#                 while self.new[0] == self.old[0]:
#                     time.sleep(0.02)
#                 self.old = list(self.new)
#                 return self.old
        
#         newRead = pollerReads()
#         self.poller.antfuncs = [self.EleLeftEar, self.EleRightEar]
#         self.poller.callbacks = [newRead.callB]
#         self.poller.track_mode(tagID)
        
#         self.fold_extra_safe()
        
#         # Position Right Start
#         #(12.01171875, -45.41015625)
#         self.earR.move_angle(math.radians(12),angvel=math.radians(9),blocking=False)
#         self.pan_r.move_angle(math.radians(-45),angvel=math.radians(10),blocking=False)
#         self.tilt_r.move_angle(math.radians(0),angvel=math.radians(40),blocking=False)
        
#         # Position Left Start
#         #(90.52734375, -34.5703125)
#         self.earL.move_angle(math.radians(90),angvel=math.radians(15),blocking=False)
#         self.pan_l.move_angle(math.radians(-20),angvel=math.radians(7),blocking=False)
#         self.tilt_l.move_angle(math.radians(0),angvel=math.radians(40),blocking=False)
        
#         res = []
#         while np.any( [f() for f in self.moving_funcs] ):
#             a = newRead.getNewRead()
#             print a
#             if a[-1] == 'EleLeftEar':
#                 angs = [self.earL.read_angle(), self.pan_l.read_angle(), self.tilt_l.read_angle()]
#             else:
#                 angs = [self.earR.read_angle(), self.pan_r.read_angle(), self.tilt_r.read_angle()]
#             if a[1] != -1:
#                 res += [ a + angs ]
        
#         # Position Right End
#         #(-41.6015625, 60.9375)
#         self.earR.move_angle(math.radians(-41),angvel=math.radians(10),blocking=False)
#         self.pan_r.move_angle(math.radians(61),angvel=math.radians(20),blocking=False)
#         self.tilt_r.move_angle(math.radians(0),angvel=math.radians(30),blocking=False)
        
#         # Position Left End
#         #(-32.51953125, 43.359375)
#         self.earL.move_angle(math.radians(-33),angvel=math.radians(25),blocking=False)
#         self.pan_l.move_angle(math.radians(43),angvel=math.radians(15),blocking=False)
#         self.tilt_l.move_angle(math.radians(0),angvel=math.radians(30),blocking=False)
        
#         while np.any( [f() for f in self.moving_funcs] ):
#             a = newRead.getNewRead()
#             print a
#             if a[-1] == 'EleLeftEar':
#                 angs = [self.earL.read_angle(), self.pan_l.read_angle(), self.tilt_l.read_angle()]
#             else:
#                 angs = [self.earR.read_angle(), self.pan_r.read_angle(), self.tilt_r.read_angle()]
#             if a[1] != -1:
#                 res += [ a + angs ]
        
#         self.poller.pause_poller()
#         time.sleep(0.050)
#         self.poller.antfuncs = []
#         self.poller.callbacks = []
#         # Done Polling
        
#         self.pan_l.move_angle(math.radians(0),angvel=math.radians(30),blocking=True)
        
#         self.fold_extra_safe()
        
#         # Begin FITTING
        
#         Lpts = []
#         Rpts = []
#         for datum in res:
#             t, rssi, ant, eA, pA, tA = datum
#             if ant == 'EleLeftEar':
#                 trans = tr.createAntLeft(eA,pA,tA)
#                 xyz = tr.globalTantL(np.matrix([5,0,0]).T)
#                 Lpts.append([rssi, mu.pol_of_cart(xyz[0:2])[1,0]+0.1])  # +- 0.1 to account for funkiness @ angle zero to make left/right align
#             else:
#                 trans = tr.createAntRight(eA,pA,tA)
#                 xyz = tr.globalTantR(np.matrix([5,0,0]).T)
#                 Rpts.append([rssi, mu.pol_of_cart(xyz[0:2])[1,0]-0.1])
        
#         Lp = np.matrix(Lpts).T
#         if Lp.shape[0] == 0:
#             Lp = np.matrix([]).reshape(2,0)
        
#         Rp = np.matrix(Rpts).T
#         if Rp.shape[0] == 0:
#             Rp = np.matrix([]).reshape(2,0)
        
#         print Lp.shape,Rp.shape
        
#         LR = np.column_stack([Lp,Rp])
        
#         def func(x,alpha,beta,mu,sigma):
#             return (1.0 / sigma) * alpha * np.exp(-1*np.power(x-mu,2.0) / (2*np.power(sigma,2.0))) + beta
        
#         def residuals(p,xs,ys):
#             alpha, beta, mu, sigma = p
#             ytmp = func(xs,alpha,beta,mu,sigma)
#             return np.power(ys - ytmp,2.0)
        
#         alpha, beta, muV, sigma = op.leastsq(residuals, [30.0, 70.0, 0.0, math.radians(60)], args=(LR[1].A1, LR[0].A1))[0]
        
#         xs = np.linspace(-np.pi, np.pi, 500)
#         ys = func(xs, alpha, beta, muV, sigma)
        
#         maxFit = xs[np.argmax(ys)]
#         mFy = ys[np.argmax(ys)]
        
#         if imgName or showPlot:
#             pl.plot(Lp[1].A1, Lp[0].A1,'go',Rp[1].A1, Rp[0].A1,'ro', xs, ys, 'b-', [maxFit], [mFy], 'co')
#             pl.legend(['Left Antenna', 'Right Antenna', '2nd Order Fit', 'Estimated Tag Heading'])
#             if imgName:
#                 pl.savefig(imgName)
#             if showPlot:
#                 pl.show()
        
        
        
#         return res, maxFit
    
    def hand_wave_QueryEnv(self, pan_servo, tilt_servo, antfunc,
                      panMin, panMax, tiltAng, panSpeed = 20):
        # Right ear: rssi_pan_tilt('Claritin    ', self.pan_r, self.tilt_r, self.EleRightEar,
        #                          -95.0, 30.0, -35.0, 45.0, 20, 15)
        # Left ear:  rssi_pan_tilt('Claritin    ', self.pan_l, self.tilt_l, self.EleLeftEar,
        #                          95.0, -30.0, -35.0, 45.0, 20, 15)
        self.fold_extra_safe()
        
        tilt_servo.move_angle( math.radians( tiltAng ),angvel=math.radians(40),blocking=False )
        pan_servo.move_angle( math.radians( panMin ),angvel=math.radians(40),blocking=True )
                
        newRead = pollerReads()  # Ensures fresh readings
        
        self.poller = M5e.M5e_Poller(self.reader, antfuncs = [antfunc], callbacks = [newRead.call_back] )

        res = []  # [ Antenna Name, TagID String, RSSI, time of read, pan_angle, tilt_angle ]
        
        self.poller.track_mode( tagID ) # Begin RFID reads as fast as possible

        pan_servo.move_angle( math.radians(panMax),angvel=math.radians(panSpeed),blocking=False )
        newRead.getNewRead() # flush reader results
        while pan_servo.is_moving():
            a = newRead.getNewRead()  # [ Antenna Name, TagID String, RSSI, time of read ]
            pan_read_ang = pan_servo.read_angle()
            time.sleep(0.01)
            tilt_read_ang = tilt_servo.read_angle()
            time.sleep(0.01)
            res += [ a + [pan_read_ang, tilt_read_ang] ]

        # Turn off RFID reader.
        self.poller.pause_poller()  # Stops the thread from reading
        self.poller.stop()  # Tell the thread to die
        time.sleep(0.050)
        
        self.fold_extra_safe()
        raw_readings = list(res)
        
        return raw_readings
        self.fold_extra_safe()
        
        return res.keys()
    




if __name__ == '__main__':
    import hrl_lib.util as ut

    e = Ears()
    scan = e.rssi_pan_tilt('cardboard   ')

    ut.save_pickle( scan, ut.formatted_time()+'_rssi_pan_tilt_scan.pkl')
