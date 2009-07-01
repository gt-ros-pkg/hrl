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

import roslib; roslib.update_path('hrl_lib')
import rospy
import std_srvs.srv as srv
from hrl_lib.msg import FloatArray

import time
import numpy as np

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
            while self.reading  == None:
                time.sleep(.1)
                print self.node_name, ': waiting for reading ...'

        reading = self.reading 
        if fresh:
            while reading[1] == self.last_message_number:
                #self._check_timeout()
                if self.delay_time != None:
                    delay_time = self.delay_time
                    self.delay_time = None #prevent multiple Exceptions from being thrown
                    print 'WARNING: delayed by', delay_time * 1000.0
                reading = self.reading 
        else:
            self._check_timeout()

        self.last_message_number = reading[1]
        return reading[0]


class GenericListener:
    ##
    # Message has to have a header
    def __init__(self, node_name, message_type, listen_channel, frequency, message_extractor=None):
        try:
            print node_name, ': inited node.'
            rospy.init_node(node_name, anonymous=True)
        except rospy.ROSException, e:
            pass
        
        self.reset()
        self.reading             = None

        def callback(msg):
            msg_time      = msg.header.stamp.to_time()
            msg_number    = msg.header.seq
            if message_extractor != None:
                self.reading  = message_extractor(msg), msg_number
            else:
                self.reading  = msg, msg_number

            #Check for delayed messages
            if self.last_msg_time == None:
                self.last_msg_time = msg_time
            time_diff = msg_time - self.last_msg_time
            if time_diff > self.delay_tolerance:
                self.delay_time = msg_time - self.last_msg_time 
            self.last_msg_time  = msg_time
            self.last_call_back = time.time()

        rospy.Subscriber(listen_channel, message_type, callback)
        self.node_name = node_name
        print node_name,': subscribed to', listen_channel

    def reset(self):
        #Statistics from header
        self.last_msg_returned   = None
        self.last_msg_time       = None
        self.last_call_back      = None

        #Delay tolerances
        #Because of Nagel's algorithm!!! FIX THIS WITH NEW ROS!
        self.delay_tolerance     = 300.0 / 1000.0
        self.delay_time          = None
        self.server_timeout      = 1.2 

    def _check_timeout(self):
        if self.last_call_back != None:
            time_diff = time.time() - self.last_call_back
            if time_diff > self.delay_tolerance:
                print self.node_name, ': have not heard back from publisher in', 1000*time_diff, 'ms'
                time.sleep(5/1000.0)

            if time_diff > self.server_timeout:
                print "GenericListener: Server have not responded for %.2f ms" % (1000 * time_diff)
                #exit()
                #raise UnresponsiveServerError("Server have not responded for %.2f ms" % (1000 * time_diff))


    def read(self, fresh=True, warn=True):
        if not fresh and self.reading == None:
            return None
        else:
            while self.reading  == None:
                time.sleep(.3)
                print self.node_name, ': waiting for reading ...'

        reading = self.reading 
        if fresh:
            while reading[1] == self.last_msg_returned     :
                if warn:
                    self._check_timeout()
                if self.delay_time != None:
                    delay_time = self.delay_time
                    self.delay_time = None #prevent multiple Exceptions from being thrown
                    print 'WARNING: delayed by', delay_time * 1000.0
                reading = self.reading 
        else:
            self._check_timeout()

        self.last_msg_returned = reading[1]
        return reading[0]
































































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

