#! /usr/bin/python

import os
import time
from threading import Lock
from collections import deque

import roslib
roslib.load_manifest("kelsey_sandbox")

import rospy
from std_srvs.srv import Empty, EmptyResponse

from srv import BehaviorRegistration, BehaviorRequest, BehaviorRequestRequest, BehaviorRelinquish
from srv import BehaviorRegistrationResponse, BehaviorRequestResponse, BehaviorRelinquishResponse
from srv import BehaviorResume, BehaviorResumeResponse

ALLOWED_RESOURCES = ['l_arm', 'r_arm', 'l_gripper', 'r_gripper', 'base', 'head', 'torso']

def log(s):
    rospy.loginfo("[behavior_manager] %s" % s)

def err(s):
    rospy.logerr("[behavior_manager] %s" % s)

class BehaviorManager(object):
    def __init__(self):
        self._lock = Lock()
        with self._lock:
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
        for resource in ALLOWED_RESOURCES:
            self._active_resources[resource] = None
            self._preempted_stacks[resource] = deque()

    def _register_cb(self, req):
        with self._lock:
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

    def _request_cb(self, req):
        with self._lock:
            bname = req.behavior_name
            priority = req.priority
            if bname not in self._registered_behaviors:
                err("Behavior %s not registered.")
                return BehaviorRequestResponse(False)
            resource = self._registered_behaviors[bname]

            if self._active_resources[resource] is None:
                # resource is free, give it out
                self._active_resources[resource] = [priority, bname]
                return BehaviorRequestResponse(True)
            else:
                if priority >= self._active_resources[resource][0]:
                    # behavior is of higher importance, preempt currently running
                    preempted_behavior = self._active_resources[resource]
                    self._preempted_stacks[resource].append(preempted_behavior)
                    self._behavior_preempts[bname]()
                    self._active_resources[resource] = [priority, bname]
                    return BehaviorRequestResponse(True)
                else:
                    # current behavior is of higher importance, reject request
                    return BehaviorRequestResponse(False)

    def _relinquish_cb(self, req):
        with self._lock:
            bname = req.behavior_name
            if bname not in self._registered_behaviors:
                err("Behavior %s not registered.")
                return BehaviorRelinquishResponse()
            resource = self._registered_behaviors[bname]
            while len(self._preempted_stacks[resource]) > 0:
                # we have resume the FIFO preempted behavior
                resumed_behavior = self._preempted_stacks[resource].pop()
                will_resume = self._behavior_resumes[resumed_behavior[1]]()
                if will_resume:
                    self._active_resources[resource] = resumed_behavior
                    return BehaviorRelinquishResponse()
            self._active_resources[resource] = None
            return BehaviorRelinquishResponse()

    def _clear_cb(self, req):
        self._clear_manager()
        return EmptyResponse()

class BehaviorManagerClient(object):
    class PRIORITIES:
        TOP = 255
        HIGH = 100
        MEDIUM = 50
        LOW = 10
        BOTTOM = 0

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

        rospy.wait_for_service('/behavior_manager/register_behavior')
        register_behavior = rospy.ServiceProxy('/behavior_manager/register_behavior', BehaviorRegistration)
        register_behavior(behavior_name=self._bname, ctrl_resource=resource)

        rospy.wait_for_service('/behavior_manager/request_resource')
        rospy.wait_for_service('/behavior_manager/relinquish_resource')
        rospy.wait_for_service('/behavior_manager/clear_manager')
        self._request_resource = rospy.ServiceProxy('/behavior_manager/request_resource',
                                                    BehaviorRequest)
        self._relinquish_resource = rospy.ServiceProxy('/behavior_manager/relinquish_resource',
                                                       BehaviorRelinquish)
        self._clear_manager = rospy.ServiceProxy('/behavior_manager/clear_manager', Empty)

    def _call_preempt(self, req):
        self._preempt_cb()
        return EmptyResponse()

    def _call_resume(self, req):
        return BehaviorResumeResponse(self._resume_cb())

    def set_preempt_cb(self, preempt_cb):
        self._preempt_cb = preempt_cb

    def set_resume_cb(self, resume_cb):
        self._resume_cb = resume_cb

    ##
    # Requests control of the resource from the behavior manager
    # @param priority Override the default priority set in the initialization.
    # @result Returns True if the resource is available, False if not.
    def request(self, priority=None):
        if priority is None:
            priority = self._default_priority
        try:
            result = self._request_resource(behavior_name=self._bname, priority=priority)
            return result
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
    def clear_manager(self):
        try:
            self._clear_manager()
        except rospy.ServiceException, e:
            err("Clear manager service connection issue: %s" % str(e))


def main():
    rospy.init_node("behavior_manager")
    bm = BehaviorManager()
    rospy.spin()

if __name__ == "__main__":
    main()
