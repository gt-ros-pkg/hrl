# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: __init__.py 12069 2010-11-09 20:31:55Z kwc $

import threading
import time

import roslib.rostime

import rospy.core
import rospy.rostime
import rospy

# author: tfield (Timers)
# author: kwc (Rate, sleep)

class TimerEvent(object):
    """
    Constructor.
    @param last_expected: in a perfect world, this is when the previous callback should have happened
    @type  last_expected: rospy.Time
    @param last_real: when the callback actually happened
    @type  last_real: rospy.Time
    @param current_expected: in a perfect world, this is when the current callback should have been called
    @type  current_expected: rospy.Time
    @param last_duration: contains the duration of the last callback (end time minus start time) in seconds.
                          Note that this is always in wall-clock time.
    @type  last_duration: float
    """
    def __init__(self, last_expected, last_real, current_expected, current_real, last_duration):
        self.last_expected    = last_expected
        self.last_real        = last_real
        self.current_expected = current_expected
        self.current_real     = current_real
        self.last_duration    = last_duration

class Timer(threading.Thread):
    """
    Convenience class for calling a callback at a specified rate
    """

    def __init__(self, period, callback, oneshot=False):
        """
        Constructor.
        @param period: desired period between callbacks
        @type  period: rospy.Time
        @param callback: callback to be called
        @type  callback: function taking rospy.TimerEvent
        @param oneshot: if True, fire only once, otherwise fire continuously until shutdown is called [default: False]
        @type  oneshot: bool
        """
        threading.Thread.__init__(self)
        self._period   = period
        self._callback = callback
        self._oneshot  = oneshot
        self._shutdown = False
        self.setDaemon(True)
        self.start()

    def shutdown(self):
        """
        Stop firing callbacks.
        """
        self._shutdown = True
        
    def run(self):
        r = rospy.Rate(1.0 / self._period.to_sec())
        current_expected = rospy.rostime.get_rostime() + self._period
        last_expected, last_real, last_duration = None, None, None
        while not rospy.core.is_shutdown() and not self._shutdown:
            r.sleep()
            current_real = rospy.rostime.get_rostime()
            start = time.time()
            self._callback(TimerEvent(last_expected, last_real, current_expected, current_real, last_duration))
            if self._oneshot:
                break
            last_duration = time.time() - start
            last_expected, last_real = current_expected, current_real
            current_expected += self._period
