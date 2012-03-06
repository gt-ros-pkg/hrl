#! /usr/bin/python

import os
import time
from threading import Lock
from collections import deque

import roslib
roslib.load_manifest("kelsey_sandbox")

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String

from srv import BehaviorRegistration, BehaviorRequest, BehaviorRequestRequest, BehaviorRelinquish
from srv import BehaviorRegistrationResponse, BehaviorRequestResponse, BehaviorRelinquishResponse
from srv import BehaviorResume, BehaviorResumeResponse

ALLOWED_RESOURCES = ['l_arm', 'r_arm', 'l_gripper', 'r_gripper', 'base', 'head', 'torso']
HEARTBEAT_TIMEOUT = 5.
HEARTBEAT_RATE = 10.

class BMPriorities:
    TOP = 255
    HIGH = 100
    MEDIUM = 50
    LOW = 10
    BOTTOM = 0

def log(s):
    rospy.loginfo("[behavior_manager] %s" % s)

def warn(s):
    rospy.logwarn("[behavior_manager] %s" % s)

def err(s):
    rospy.logerr("[behavior_manager] %s" % s)

class BehaviorManager(object):
    def __init__(self):
        self._resource_lock = Lock()
        self._heartbeat_lock = Lock()
        with self._resource_lock:
            self._clear_manager()
            self._register_srv = rospy.Service('/behavior_manager/register_behavior', 
                                               BehaviorRegistration, self._register_cb)
            self._request_srv = rospy.Service('/behavior_manager/request_resource',
                                              BehaviorRequest, self._request_cb)
            self._relinquish_srv = rospy.Service('/behavior_manager/relinquish_resource',
                                                 BehaviorRelinquish, self._relinquish_cb)
            self._clear_srv = rospy.Service('/behavior_manager/clear_manager',
                                            Empty, self._clear_cb)

    def _clear_manager(self):
        self._active_resources = {}
        self._registered_behaviors = {}
        self._behavior_preempts = {}
        self._behavior_resumes = {}
        self._preempted_stacks = {}
        self._heartbeat_timers = {}
        self._hb_last_times = {}
        for resource in ALLOWED_RESOURCES:
            self._active_resources[resource] = None
            self._preempted_stacks[resource] = deque()

    def _register_cb(self, req):
        with self._resource_lock:
            bname = req.behavior_name
            if req.ctrl_resource not in ALLOWED_RESOURCES:
                err("Resource %s not reservable." % req.ctrl_resource)
                return BehaviorRegistrationResponse(False)
            if bname in self._registered_behaviors:
                err("Behavior %s already registered." % bname)
                return BehaviorRegistrationResponse(False)
            self._registered_behaviors[bname] = req.ctrl_resource
            rospy.wait_for_service('/behavior_manager/%s/preempt' % bname)
            rospy.wait_for_service('/behavior_manager/%s/resume' % bname)
            self._behavior_preempts[bname] = rospy.ServiceProxy('/behavior_manager/%s/preempt' % bname,
                                                                Empty)
            self._behavior_resumes[bname] = rospy.ServiceProxy('/behavior_manager/%s/resume' % bname,
                                                               BehaviorResume)
            log("Behavior %s successfully registered in the behavior manager." % bname)
            return BehaviorRegistrationResponse(True)

    def _heartbeat_monitor(self, te):
        cur_time = rospy.get_time()
        for resource in self._active_resources:
            running_resource = self._active_resources[resource]
            if running_resource is None:
                continue
            bname = running_resource[1]
            if cur_time - self._hb_last_times[bname] > HEARTBEAT_TIMEOUT:
                resource = self._registered_behaviors[bname]
                warn("Behavior %s has timed out and will release %s" % 
                             (bname, resource))
                self._relinquish_from_behavior(bname)

    def _heartbeat_sub_cb(self, msg):
        with self._heartbeat_lock:
            self._hb_last_times[msg.data] = rospy.get_time()

    def _request_cb(self, req):
        with self._resource_lock:
            bname = req.behavior_name
            priority = req.priority
            if bname not in self._registered_behaviors:
                err("Behavior %s not registered." % bname)
                return BehaviorRequestResponse(False)
            resource = self._registered_behaviors[bname]

            if self._active_resources[resource] is None:
                # resource is free, give it out
                self._active_resources[resource] = [priority, bname]
                success = True
            elif priority >= self._active_resources[resource][0]:
                # behavior is of higher importance, preempt currently running
                preempted_behavior = self._active_resources[resource]
                self._preempted_stacks[resource].append(preempted_behavior)
                self._behavior_preempts[bname]()
                self._active_resources[resource] = [priority, bname]
                success = True
            else:
                # current behavior is of higher importance, reject request
                success = False
            if success:
                self._hb_last_times[bname] = rospy.get_time()
                self._heartbeat_sub = rospy.Subscriber('/behavior_manager/%s/heartbeat' % bname, 
                                                       String, self._heartbeat_sub_cb)
                self._heartbeat_timers[bname] = rospy.Timer(rospy.Duration(1./HEARTBEAT_RATE),
                                                            self._heartbeat_monitor)
            return BehaviorRequestResponse(success)

    def _relinquish_cb(self, req):
        bname = req.behavior_name
        self._relinquish_from_behavior(bname)
        return BehaviorRelinquishResponse()

    def _relinquish_from_behavior(self, bname):
        with self._resource_lock:
            if bname not in self._registered_behaviors:
                err("Behavior %s not registered.")
                return
            resource = self._registered_behaviors[bname]
            while len(self._preempted_stacks[resource]) > 0:
                # we have resume the FIFO preempted behavior
                resumed_behavior = self._preempted_stacks[resource].pop()
                will_resume = self._behavior_resumes[resumed_behavior[1]]()
                if will_resume:
                    self._active_resources[resource] = resumed_behavior
                    return 
            self._active_resources[resource] = None

    def _clear_cb(self, req):
        self._clear_manager()
        return EmptyResponse()

class BehaviorManagerClient(object):

    ##
    # Interfaces with the behavior manager to control who has access to each of
    # the robot resources at any given time.
    # @param resource Name of the resource this behavior will register (must be in ALLOWED_RESOURCES)
    #                 ['l_arm', 'r_arm', 'l_gripper', 'r_gripper', 'base', 'head', 'torso']
    # @param preempt_cb Function to be called preempt_cb() when behavior needs to be preempted.
    #                   Make sure this function is non-blocking.
    # @param resume_cb Function to be called will_resume = resume_cb() when the preemptor has released
    #                  control.  The bool will_resume, if True, will give that behavior back control.
    #                  Make sure this function is non-blocking.
    # @param priority The strength of the behavior to overrule others.  A behavior with equal or
    #                 greater priority will be able to preempt the current active behavior.
    # @param name Optional prefix given to the behavior to make services more readable
    def __init__(self, resource, preempt_cb, resume_cb=None,
                 priority=PRIORITIES.MEDIUM, name="behavior"):
        if resource not in ALLOWED_RESOURCES:
            err("Resource %s is not an allowed resource to reserve." % resource)
        self._resource = resource
        self._default_priority = priority
        # this makes the behavior anonymous (exactly like init_node)
        self._bname = "%s_%s_%s" % (name, os.getpid(), int(time.time()*1000))
        self._preempt_cb = preempt_cb
        if resume_cb is None:
            def resume_cb_tmp():
                return False
            self._resume_cb = resume_cb_tmp
        else:
            self._resume_cb = resume_cb

        self._preempt_srv = rospy.Service('/behavior_manager/%s/preempt' % self._bname, 
                                          Empty, self._call_preempt)
        self._resume_srv = rospy.Service('/behavior_manager/%s/resume' % self._bname, 
                                         BehaviorResume, self._call_resume)
        self._heartbeat_pub = rospy.Publisher('/behavior_manager/%s/heartbeat' % self._bname, String)

        rospy.wait_for_service('/behavior_manager/register_behavior')
        register_behavior = rospy.ServiceProxy('/behavior_manager/register_behavior', BehaviorRegistration)
        register_behavior(behavior_name=self._bname, ctrl_resource=resource)

        rospy.wait_for_service('/behavior_manager/request_resource')
        rospy.wait_for_service('/behavior_manager/relinquish_resource')
        self._request_resource = rospy.ServiceProxy('/behavior_manager/request_resource',
                                                    BehaviorRequest)
        self._relinquish_resource = rospy.ServiceProxy('/behavior_manager/relinquish_resource',
                                                       BehaviorRelinquish)

    def _call_preempt(self, req):
        self._preempt_cb()
        return EmptyResponse()

    def _call_resume(self, req):
        return BehaviorResumeResponse(self._resume_cb())

    def set_preempt_cb(self, preempt_cb):
        self._preempt_cb = preempt_cb

    def set_resume_cb(self, resume_cb):
        self._resume_cb = resume_cb

    def _heartbeat_cb(self, te):
        self._heartbeat_pub.publish(String(self._bname))

    ##
    # Requests control of the resource from the behavior manager
    # @param priority Override the default priority set in the initialization.
    # @result Returns True if the resource is available, False if not.
    def request(self, priority=None):
        if priority is None:
            priority = self._default_priority
        try:
            success = self._request_resource(behavior_name=self._bname, priority=priority)
            if success:
                self._heartbeat_timer = rospy.Timer(rospy.Duration(1./HEARTBEAT_RATE), self._heartbeat_cb)
            return success
        except rospy.ServiceException, e:
            err("Request service connection issue: %s" % str(e))
            return False

    ##
    # Relinquishes control of the resource to the behavior manager
    def relinquish(self):
        try:
            self._relinquish_resource(behavior_name=self._bname)
        except rospy.ServiceException, e:
            err("Relinquish service connection issue: %s" % str(e))


##
# Resets control of all resources in the behavior manager (CAREFUL WITH THIS).
def clear_behavior_manager():
    clear_manager = rospy.ServiceProxy('/behavior_manager/clear_manager', Empty)
    rospy.wait_for_service('/behavior_manager/clear_manager')
    try:
        clear_manager()
    except rospy.ServiceException, e:
        err("Clear manager service connection issue: %s" % str(e))

class Behavior(object):
    def __init__(self, resource, priority, name="behavior"):
        self.name = name
        self.priority = priority
        self.bmc = BehaviorManagerClient(resource, self._preempt_cb, self._resume_cb,
                                         priority=priority, name=name)
        self._execute = self.execute
        def wrap_execute(*args, **kwargs):
            if self.bmc.request():
                self._execute(*args, **kwargs)
                self.bmc.relinquish()
                return True
            else:
                return False
        self.execute = wrap_execute

    def _preempt_cb(self):
        self.preempt()

    def _resume_cb(self):
        return self.resume()

    def preempt(self):
        rospy.logerror("[behavior] preempt must be overrided!")

    def resume(self):
        return False

    def execute(self):
        rospy.logerror("[behavior] execute must be overrided!")

def main():
    rospy.init_node("behavior_manager")
    bm = BehaviorManager()
    rospy.spin()

if __name__ == "__main__":
    main()
