#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#  \author Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)

import roslib; roslib.load_manifest('hrl_lib')
import rospy
#import rosrecord
import std_srvs.srv as srv
from hrl_msgs.msg import FloatArray
#import tf
#import tf.msg
from rospy.impl.tcpros import DEFAULT_BUFF_SIZE

import time
import numpy as np
import geometry_msgs.msg as gm
import sensor_msgs.msg as sm
import threading

##
# Converts any ros message class into a dictionary
# (currently used to make PoseStamped messages picklable)
# @param msg
# @return a nested dict with rospy.Time turned into normal unix time float
def ros_to_dict(msg):
    d = {}
    for f in msg.__slots__:
        val = eval('msg.%s' % f)
        methods = dir(val)
        if 'to_time' in methods:
            val = eval('val.to_time()')
        elif '__slots__' in methods:
            val = ros_to_dict(val)
        d[f] = val
    return d

###
## Converts a list of pr2_msgs/PressureState into a matrix
##
## @return left mat, right mat, array
#def pressure_state_to_mat(contact_msgs):
#    times = np.array([c.header.stamp.to_time() for c in contact_msgs])
#    left, right = zip(*[[list(c.l_finger_tip), list(c.r_finger_tip)] for c in contact_msgs])
#    
#    left = np.matrix(left).T
#    right = np.matrix(right).T
#    return left, right, times

def np_to_pointcloud(points_mat, frame):
    pc = sm.PointCloud()
    pc.header.stamp = rospy.get_rostime()
    pc.header.frame_id = frame
    for i in range(points_mat.shape[1]):
        p32 = gm.Point32()
        p32.x = points_mat[0,i]
        p32.y = points_mat[1,i]
        p32.z = points_mat[2,i]
        pc.points.append(p32)
    return pc

def np_to_colored_pointcloud(points_mat, intensity, frame):
    pc = np_to_pointcloud(points_mat, frame)
    pc.channels.append(sm.ChannelFloat32())
    pc.channels[0].name = 'intensity'
    pc.channels[0].values = intensity.A1.tolist()
    return pc

##
# @param points_mat 2xn
# @param intensities 3xn
# @param frame
def np_to_rgb_pointcloud(points_mat, intensities, frame):
    pc = np_to_pointcloud(points_mat, frame)
    pc.channels.append(sm.ChannelFloat32())
    pc.channels[0].name = 'r'
    pc.channels[0].values = intensities[2,:].A1.tolist()

    pc.channels.append(sm.ChannelFloat32())
    pc.channels[1].name = 'g'
    pc.channels[1].values = intensities[1,:].A1.tolist()

    pc.channels.append(sm.ChannelFloat32())
    pc.channels[2].name = 'b'
    pc.channels[2].values = intensities[0,:].A1.tolist()
    return pc

def pointcloud_to_np(pc):
    plist = []
    for p in pc.points:
        plist.append([p.x, p.y, p.z])
    return np.matrix(plist).T

#class LaserScanner:
#    def __init__(self, service):
#        srv_name = '/%s/single_sweep_cloud' % service
#        self.sp = rospy.ServiceProxy(srv_name, snp.BuildCloudAngle)
#
#    def scan_np(self, start, end, duration):
#        resp = self.sp(start, end, duration)
#        return pointcloud_to_np(resp.cloud)
#
#    def scan(self, start, end, duration):
#        resp = self.sp(start, end, duration)
#        return resp.cloud

##
# Iterator function for simplified filtered bag reading. Works with large bags/messages.
#
# @param file_name bag file name
# @param topics list of topics that you care about (leave blank to get everything)
# @return list of (topic_name, message, rostime) 
def bag_iter(file_name, topics=[]):
    f = open(file_name)
    tdict = {}
    for t in topics:
        tdict[t] = True

    for r in rosrecord.logplayer(f):
        if tdict.has_key(r[0]):
            yield r

    f.close()

##
# Select topics from a given bag.  Use this only if you're dealing with small
# messages as not everything will fit into RAM.
#
# @param file_name bag file name
# @param topics list of topics that you care about (leave blank to get everything)
# @return dict with each entry containing a list of [(topic_name, message, rostime), (...), ...]
def bag_sel_(file_name, topics=[]):
    print 'deprecated'
    d = {}
    for topic, msg, t in bag_iter(file_name, topics):
        if not d.has_key(topic):
            d[topic] = []
        d[topic].append((topic, msg, t.to_time()))
    return d

def bag_sel(file_name, topics=[]):
    d = {}
    for topic, msg, t in bag_iter(file_name, topics):
        if not d.has_key(topic):
            d[topic] = {'topic':[], 'msg':[], 't':[]}
        d[topic]['topic'].append(topic)
        d[topic]['msg'].append(msg)
        d[topic]['t'].append(t.to_time())
        #d[topic].append((topic, msg, t.to_time()))
    return d

##
# Used on ROS server (service provider) side to conveniently declare
# a function that returns nothing as a service.
#
# ex.  obj = YourObject()
#      rospy.Service('your_service', YourSrv, 
#                    wrap(obj.some_function, 
#                         # What that function should be given as input from the request
#                         ['request_data_field_name1',  'request_data_field_name2'],
#                         # Class to use when construction ROS response
#                         response=SomeClass
#                         ))
def wrap(f, inputs=[], response=srv.EmptyResponse, verbose=False):
    def _f(request):
        arguments = [eval('request.' + arg) for arg in inputs]
        if verbose:
            print 'Function', f, 'called with args:', arguments
        try:
            returns = f(*arguments)
            if returns == None:
                return response()
            else:
                return response(*returns)
        except Exception, e:
            print e
    return _f

def ignore_return(f):
    def _f(*args):
        f(*args)
    return _f

class UnresponsiveServerError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

##
# Listens over ros network
class FloatArrayListener:
    ##
    # Constructor
    #
    # @param node_name name of node if current process has not initialized ROS
    # @param listen_channel name of channel to listen to
    # @param frequency frequency this node should expect to get messages, 
    #        this value is used for determinining when messages are stale
    def __init__(self, node_name, listen_channel, frequency):
        try:
            print node_name, ': inited node.'
            rospy.init_node(node_name, anonymous=True)
        except rospy.ROSException, e:
            #print e
            pass

        self.reading             = None
        self.last_message_number = None
        self.last_msg_time       = None
        self.last_call_back      = None

        self.delay_tolerance     = 300.0 / 1000.0 #Because of Nagel's
        self.delay_time          = None
        self.time_out            = 1.2 #Because of Nagel's algorithm!!! FIX THIS WITH NEW ROS!
        self.count = 0

        self.error = False

        def callback(msg):
            msg_time      = msg.header.stamp.to_time()
            msg_number    = msg.header.seq
            self.reading  = np.matrix(msg.data, 'f').T, msg_number

            #Check for delayed messages
            if self.last_msg_time == None:
                self.last_msg_time = msg_time

            time_diff = msg_time - self.last_msg_time
            if time_diff > self.delay_tolerance:
                self.delay_time = msg_time - self.last_msg_time 
            #print 1000*time_diff
            self.last_msg_time  = msg_time

            ctime = time.time()
            #if self.last_call_back != None:
            #    print 1000.0*(ctime - self.last_call_back)
            #self.last_call_back = time.time()
            #if self.last_call_back != None:
            #    print 'last called back at %.2f ms ago, %d'% (1000*(ctime-self.last_call_back), self.count)
            self.last_call_back = ctime
            self.count = self.count + 1

        rospy.Subscriber(listen_channel, FloatArray, callback)
        self.node_name = node_name
        print node_name,': subscribed to', listen_channel

    def _check_timeout(self):
        if self.last_call_back != None:
            #ctime = time.time()
            time_diff = time.time() - self.last_call_back
            if time_diff > self.delay_tolerance:
                print self.node_name, ': have not heard back from publisher in', 1000*time_diff, 'ms'
                time.sleep(5/1000.0)
            
            if time_diff > 1.0:
                self.error = True

            if time_diff > self.time_out:
                #print 'raising hell!', ctime, self.last_call_back
                #if self.times > 0:
                    #print 'Times', self.times
                print "FloatArrayListener: Server have not responded for %.2f ms" % (1000 * time_diff)
                #exit()
                #raise UnresponsiveServerError("Server have not responded for %.2f ms" % (1000 * time_diff))
                #else:
                #    self.times = self.times + 1


    def read(self, fresh=True):
        if not fresh and self.reading == None:
            return None
        else:
            t = time.time()
            while self.reading  == None:
                time.sleep(1.0)
                print self.node_name, ': no readings for %.2f s' % (time.time() - t)

        reading = self.reading 
        if fresh:
            while reading[1] == self.last_message_number:
                #self._check_timeout()
                time.sleep(1/1000.0)
                if self.delay_time != None:
                    delay_time = self.delay_time
                    self.delay_time = None #prevent multiple Exceptions from being thrown
                    print 'WARNING: delayed by', delay_time * 1000.0
                reading = self.reading 
        else:
            self._check_timeout()

        self.last_message_number = reading[1]
        return reading[0]

##
# Takes a normal ROS callback channel and gives it an on demand query style
# interface.
class GenericListener:
    ##
    # Message has to have a header
    # @param node_name name of node (if haven't been inited)
    # @param message_type type of message to listen for
    # @param listen_channel ROS channel to listen
    # @param frequency the frequency to expect messages (used to print warning statements to console)
    # @param message_extractor function to preprocess the message into a desired format
    # @param queue_size ROS subscriber queue (None = infinite)
    def __init__(self, node_name, message_type, listen_channel,
                 frequency, message_extractor=None, queue_size=None):
        try:
            print node_name, ': inited node.'
            rospy.init_node(node_name, anonymous=True)
        except rospy.ROSException, e:
            pass
        self.last_msg_returned   = None   #Last message returned to callers from this class
        self.last_call_back      = None   #Local time of last received message
        self.delay_tolerance     = 1/frequency #in seconds
        self.reading             = {'message':None, 'msg_id':-1}
        self.curid               = 0
        self.message_extractor = message_extractor

        def callback(*msg):
            #If this is a tuple (using message filter)
            if 'header' in dir(msg):
                if msg.__class__ == ().__class__:
                    msg_number = msg[0].header.seq
                else:
                    msg_number = msg.header.seq
            else:
                msg_number = self.curid
                self.curid += 1

            #*msg makes everything a tuple.  If length is one, msg = (msg, )
            if len(msg) == 1:
                msg = msg[0]
            
            self.reading  = {'message':msg, 'msg_id':msg_number}

            #Check for delayed messages
            self.last_call_back = time.time() #record when we have been called back last

        if message_type.__class__ == [].__class__:
            import message_filters
            subscribers = [message_filters.Subscriber(channel, mtype) for channel, mtype in zip(listen_channel, message_type)]
            queue_size = 10
            ts = message_filters.TimeSynchronizer(subscribers, queue_size)
            ts.registerCallback(callback)
        else:
            rospy.Subscriber(listen_channel, message_type, callback,
                             queue_size = queue_size)

        self.node_name = node_name
        #print node_name,': subscribed to', listen_channel
        rospy.loginfo('%s: subscribed to %s' % (node_name, listen_channel))

    def _check_for_delivery_hiccups(self):
        #If have received a message in the past
        if self.last_call_back != None:
            #Calculate how it has been
            time_diff = time.time() - self.last_call_back
            #If it has been longer than expected hz, complain
            if time_diff > self.delay_tolerance:
                print self.node_name, ': have not heard back from publisher in', time_diff, 's'

    def _wait_for_first_read(self, quiet=False):
        if not quiet:
            rospy.loginfo('%s: waiting for reading ...' % self.node_name)
        while self.reading['message'] == None and not rospy.is_shutdown():
            time.sleep(0.1)
            #if not quiet:
            #    print self.node_name, ': waiting for reading ...'

    ## 
    # Supported use cases
    # rfid   - want to get a reading, can be stale, no duplication allowed (allow None),        query speed important
    # hokuyo - want to get a reading, can be stale, no duplication allowed (don't want a None), willing to wait for new data (default)
    # ft     - want to get a reading, can be stale, duplication allowed    (don't want a None), query speed important
    # NOT ALLOWED                                   duplication allowed,                        willing to wait for new data
    def read(self, allow_duplication=False, willing_to_wait=True, warn=False, quiet=True):
        if allow_duplication:
            if willing_to_wait:
                raise RuntimeError('Invalid settings for read.')
            else: 
                # ft - want to get a reading, can be stale, duplication allowed (but don't want a None), query speed important
                #self._wait_for_first_read(quiet)
                reading                = self.reading
                self.last_msg_returned = reading['msg_id']
                if self.message_extractor is not None:
                    return self.message_extractor(reading['message'])
                else:
                    return reading['message']
        else:
            if willing_to_wait:
                # hokuyo - want to get a reading, can be stale, no duplication allowed (don't want a None), willing to wait for new data (default)
                self._wait_for_first_read(quiet)
                while self.reading['msg_id'] == self.last_msg_returned and not rospy.is_shutdown():
                    if warn:
                        self._check_for_delivery_hiccups()
                    time.sleep(1/1000.0)
                reading = self.reading
                self.last_msg_returned = reading['msg_id']
                if self.message_extractor is not None:
                    return self.message_extractor(reading['message'])
                else:
                    return reading['message']
            else:
                # rfid   - want to get a reading, can be stale, no duplication allowed (allow None),        query speed important
                if self.last_msg_returned == self.reading['msg_id']:
                    return None
                else:
                    reading = self.reading
                    self.last_msg_returned = reading['msg_id']
                    if self.message_extractor is not None:
                        return self.message_extractor(reading['message'])
                    else:
                        return reading['message']

##
#Class for registering as a subscriber to a specified topic, where
#the messages are of a given type. Only calls the callback at a
#prescribed rate.
class RateListener():
    #Constructor.
    
    # Wraps the Subscriber class to only call the callback at
    # no quicker than the specified rate.

    #@param rate: time in seconds for subscriber to call callback
    #@type rate: float
    #@param name: graph resource name of topic, e.g. 'laser'.
    #@type  name: str
    #@param data_class: data type class to use for messages,
    #  e.g. std_msgs.msg.String
    #@type  data_class: L{Message} class
    #@param callback: function to call ( fn(data)) when data is
    #  received. If callback_args is set, the function must accept
    #  the callback_args as a second argument, i.e. fn(data,
    #  callback_args).  NOTE: Additional callbacks can be added using
    #  add_callback().
    #@type  callback: str
    #@param callback_args: additional arguments to pass to the
    #  callback. This is useful when you wish to reuse the same
    #  callback for multiple subscriptions.
    #@type  callback_args: any
    #@param queue_size: maximum number of messages to receive at
    #  a time. This will generally be 1 or None (infinite,
    #  default). buff_size should be increased if this parameter
    #  is set as incoming data still needs to sit in the incoming
    #  buffer before being discarded. Setting queue_size
    #  buff_size to a non-default value affects all subscribers to
    #  this topic in this process.
    #@type  queue_size: int
    #@param buff_size: incoming message buffer size in bytes. If
    #  queue_size is set, this should be set to a number greater
    #  than the queue_size times the average message size. Setting
    #  buff_size to a non-default value affects all subscribers to
    #  this topic in this process.
    #@type  buff_size: int
    #@param tcp_nodelay: if True, request TCP_NODELAY from
    #  publisher.  Use of this option is not generally recommended
    #  in most cases as it is better to rely on timestamps in
    #  message data. Setting tcp_nodelay to True enables TCP_NODELAY
    #  for all subscribers in the same python process.
    #@type  tcp_nodelay: bool
    #@raise ROSException: if parameters are invalid
    def __init__(self, rate, name, data_class, callback=None, callback_args=None,
                 queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):
        self.rate = rate
        self.callback = callback
        self.lasttime = 0.0
        self.sub = rospy.Subscriber(name, data_class, self._process_msg, callback_args,
                              queue_size, buff_size, tcp_nodelay)
    
    def _process_msg(self, data, callback_args=None):
        delay = rospy.Time.now().to_sec() - self.lasttime
        if delay >= self.rate or self.lasttime == 0.0:
            if callback_args == None:
                self.callback(data)
            else:
                self.callback(data, callback_args)
            self.lasttime = rospy.Time.now().to_sec()

    def unregister(self):
        self.sub.unregister()

    def get_num_connections():
        return self.sub.get_num_connections()


class RateCaller():

    def __init__(self, func, rate, args=None):
        self.func = func
        self.rate = rate
        self.args = args
        self.thread = None
        self.is_running = False
        self.beg_time = None

    def run(self):
        if not self.is_running:
            self.beg_time = time.time()
            self.num_time = 0
            self.thread = threading.Timer(self.rate, self._run)
            self.is_running = True
            self.thread.start()

    def _run(self):
        if self.is_running:
            if self.args is None:
                self.func()
            else:
                self.func(*self.args)
            self.num_time += 1

            diff_time = time.time() - self.beg_time
            timer_len = self.rate * (self.num_time + 1) - diff_time
            if timer_len <= 0.:
                timer_len = 0.
            self.thread = threading.Timer(timer_len, self._run)
            self.thread.start()

    def stop(self):
        if self.thread is not None and self.is_running:
            self.thread.cancel()
            self.is_running = False

#class TransformBroadcaster:
#
#    def __init__(self):
#        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)
#
#    ## send transform as a tfmessage.
#    # @param tf_stamped - object of class TransformStamped (rosmsg show TransformStamped)
#    def send_transform(self,tf_stamped):
#        tfm = tf.msg.tfMessage([tf_stamped])
#        self.pub_tf.publish(tfm)

##
# Calls the service with the given name and given parameters and returns
# the output of the service.
# @param service_name the full name of the service to be called
# @param params the list of parameters to be passed into the service
# @filename name of a pickle object to be created if not None
def call_save_service(service_name, service_def, params, filename=None):
    srv = rospy.ServiceProxy(service_name, service_def)
    try:
        resp = srv(*params)
        if not filename is None:
            save_pickle(resp, filename)
        srv.close()
        return resp
    except rospy.ServiceException, e:
        print "Service error"
    srv.close()
    return None

























































#class ROSPoll:
#    def __init__(self, topic, type):
#        self.data       = None
#        self.t          = time.time()
#        self.old        = self.t
#        self.subscriber = rospy.TopicSub(topic, type, self.callback)
#
#    def callback(self, data):
#        self.data = data
#        self.t    = time.time()
#
#    ##
#    # Returns only fresh data!
#    def read(self):
#        t = time.time()
#        while self.old == self.t:
#            time.sleep(.1)
#            if (time.time() - t) > 3.0:
#                print 'ROSPoll: failed to read. Did you do a rospy.init_node(name)?'
#                return None
#        self.old = self.t
#        return self.data
#
##
# Sample Code
#if __name__ == '__main__':
#    from pkg import *
#    from urg_driver.msg import urg as UrgMessage
#    rospy.init_node('test') #Important before doing anything ROS related!
#
#    poller = ROSPoll('urg', UrgMessage)
#    msg    = poller.read()
#    print msg.__class__


    #def _check_freshness(self, current_time):
    #    if self.last_received_time != None:
    #        time_diff = current_time - self.last_received_time
    #        #print '%.2f' %( time_diff * 1000.0)
    #        if time_diff > self.stale_tolerance:
    #            self.throw_exception = time_diff
    #def _stale_alert(self):
    #    if self.throw_exception != None:
    #        self.throw_exception = None
    #        raise RuntimeError('Have not heard from publisher for %.2f ms' % (1000.0 * self.throw_exception))

###
## Wrapper helper function
#def empty(f):
#    def _f(request):
#        print 'Request', request, ' for function', f, 'received.'
#        try:
#            f()
#        except Exception, e:
#            print e
#        return srv.EmptyResponse()
#    return _f

    #def read(self, fresh=True, no_wait, warn=True):
    #    #if reading
    #    reading = self.reading 
    #    if reading != None:
    #        if fresh:
    #            #While the current message is equal the the last message we returned caller
    #            while reading['msg_id'] == self.last_msg_returned:
    #                if warn:
    #                    self._check_for_delivery_hiccups()
    #                reading = self.reading 
    #                time.sleep(1/1000.0)
    #            self.last_msg_returned = reading['msg_id']
    #            return reading['message']

    #        elif allow_duplicates:
    #            self.last_msg_returned = reading['msg_id']
    #            return reading['message']

    #        else:
    #            #not fresh & don't allow duplicates
    #            if reading['msg_id'] == self.last_msg_returned:
    #                return None
    #            else:
    #                self.last_msg_returned = reading['msg_id']
    #                return reading['message']
    #    else:
    #        fresh and allow_duplicates
    #        fresh and not allow_duplicates
    #        not fresh and not allow_duplicates
    #        not fresh and allow_duplicates





   #     #Is this the first time we have been called?
   #     if not fresh and self.reading == None:
   #         return None
   #     else:
   #         while self.reading  == None:
   #             time.sleep(.3)
   #             print self.node_name, ': waiting for reading ...'

   #     reading = self.reading 
   #     if fresh:
   #         #While the current message is equal the the last message we returned caller
   #         while reading[1] == self.last_msg_returned:
   #             if warn:
   #                 self._check_for_delivery_hiccups()
   #             #if self.delay_time != None:
   #             #    delay_time = self.delay_time
   #             #    self.delay_time = None #prevent multiple Exceptions from being thrown
   #             #    if warn:
   #             #        print 'WARNING: delayed by', delay_time * 1000.0
   #             reading = self.reading 
   #             time.sleep(1/1000.0)
   #     else:
   #         self._check_timeout()

   #     self.last_msg_returned = reading[1]
   #     return reading[0]
