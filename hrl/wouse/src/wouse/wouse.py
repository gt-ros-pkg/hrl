#!/usr/bin/env python

from threading import Condition

#General
import roslib; roslib.load_manifest('wouse')
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped

from mouse_listener_thread import MouseListener, MouseEvent
from wouse.srv import WouseRunStop, WouseRunStopRequest

class Wouse(object):
    """Listens for mouse events, detects wincing motions, and signals e-stop"""
    def __init__(self):
        try:
            rospy.wait_for_service('wouse_run_stop', 5) 
            self.runstop_client=rospy.ServiceProxy('wouse_run_stop',
                                                        WouseRunStop)
            rospy.loginfo("Found wouse run-stop service")
        except:
            rospy.logerr("Cannot find wouse run-stop service")
        
        self.mouse_event = MouseEvent()
        device_file = rospy.get_param('~wouse_device_file', '/dev/input/mouse1')
        self.condition = Condition()
        self.mouse_listener = MouseListener(self.mouse_event,
                                            self.condition,
                                            device_file)
        self.mouse_listener.start()
        
        self.point_pub = rospy.Publisher('wouse_movement', PointStamped)
        self.ptst = PointStamped()

        self.ping_server_pub = rospy.Publisher('runstop_alive_ping', Header)
        self.ping_timer = rospy.Timer(rospy.Duration(0.01), self.ping_server)

        self.filt_pos = None
        self.stop_cnt = 0
        self.stop_time = rospy.get_time()
       
    def ping_server(self, event):
        """Send updated timestamp to Runstop server."""
        hdr = Header()
        hdr.stamp = rospy.Time.now()
        self.ping_server_pub.publish(hdr)

    def poll(self):
        """Wait for new mouse event from listener thread, then pub/process"""
        with self.condition:
            self.condition.wait() 
            self.ptst.header.stamp = rospy.Time.now()
            self.ptst.point.x = self.mouse_event.rel_x
            self.ptst.point.y = self.mouse_event.rel_x
            if self.mouse_event.x_overflow or self.mouse_event.y_overflow:
                self.ptst.point.z = 1
        self.point_pub.publish(self.ptst)

    def threshold(self,x,y):
        """Detect wince signals based upon simple threshold"""
        if filt_pos == None:
            filt_pos = pos
        else:
            alpha = 0.8
            filt_pos = (alpha * filt_pos) + ((1.0 - alpha) * pos)
        diff = pos - filt_pos
        x_thresh = 1.5
        y_thresh = 1.5
        stop_wait = rospy.Duration(1.0) #seconds
        stop_time_diff = rospy.get_time() - stop_time
        if ((diff[0] > x_thresh) and
            (diff[1] > y_thresh) and
            (stop_time_diff > stop_wait)):
            
            self.runstop_client(WouseRunStopRequest(True, False))

            stop_cnt = stop_cnt + 1
            stop_time = rospy.get_time()
            print '*****************'
            print 'STOP %d!' % stop_cnt
            print 'seconds since last STOP = %f' % stop_time_diff.to_sec()
            print 'diff =', diff
            print '*****************'

if __name__=='__main__':
    rospy.init_node('wouse_node')
    wouse = Wouse()
    while not rospy.is_shutdown():
        wouse.poll()

#    def Xlib_polling(self):
#        curr_point = self.disp_root.query_pointer()
#        return (curr_point._data["root_x"], curr_point._data["root_y"])
#
#
#    def xinput_polling(self):
#        """An implementation which polls xinput from the commandline to get mouse position"""
#        xid = rospy.get_param('~xinput_id', "11")
#        proc = sp.Popen(["xinput query-state "+xid],
#                        shell=True, 
#                        stdout=sp.PIPE)
#        (out, err) = proc.communicate()
#        if not err:
#            mouse_pos = []
#            state_fields = shlex.split(out)
#            for field in state_fields[-2:]:
#                eq_ind = field.find('=')
#                mouse_pos.append(float(field[(eq_ind+1):]))
#            return tuple(mouse_pos)
#
#    def pygame_polling(self):
#        pg.event.get()
#        return pg.mouse.get_pos()
