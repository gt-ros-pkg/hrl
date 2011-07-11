import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy
import rosparam

from pr2_mechanism_msgs.srv import LoadController, UnloadController, SwitchController

class ControllerSwitcher:
    def __init__(self):
        self.load_controller = rospy.ServiceProxy('pr2_controller_manager/load_controller', 
                                                  LoadController)
        self.unload_controller = rospy.ServiceProxy('pr2_controller_manager/unload_controller', 
                                                    UnloadController)
        self.switch_controller_srv = rospy.ServiceProxy('pr2_controller_manager/switch_controller', 
                                                        SwitchController)

    def switch(self, old_controller, new_controller, param_file=None):
        if param_file is None:
            resp = self.switch_controller_srv([new_controller], [old_controller], 1)
            return resp.ok
        else:
            self.switch_controller_srv([], [old_controller], 1)
            self.unload_controller(old_controller)
            if old_controller != new_controller:
                self.unload_controller(new_controller)
            params = rosparam.load_file(param_file)
            rosparam.upload_params("", params[0][0])
            self.load_controller(old_controller)
            if old_controller != new_controller:
                self.load_controller(new_controller)
            resp = self.switch_controller_srv([new_controller], [], 1)
            return resp.ok
