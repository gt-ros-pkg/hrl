import roslib
roslib.load_manifest("hrl_ellipsoidal_control")
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty

class ControllerInterfaceBackend(object):
    def __init__(self, name):
        self.ctrl_name = name
        self.button_clk_sub = rospy.Subscriber("/arm_cart_ctrl_gui/button_clk", String,
                                               self._button_clk_cb)
        self.buttons_enable_srv = rospy.ServiceProxy("/arm_cart_ctrl_gui/buttons_enable", Empty)
        self.buttons_disable_srv = rospy.ServiceProxy("/arm_cart_ctrl_gui/buttons_disable", Empty)
        self.set_ctrl_name_pub = rospy.Publisher("/arm_cart_ctrl_gui/set_controller_name", String)
        self.set_status_pub = rospy.Publisher("/arm_cart_ctrl_gui/set_status", String)

        rospy.sleep(0.1)
        self.set_ctrl_name_pub.publish(self.ctrl_name)
        self.buttons_enable_srv()

    def _button_clk_cb(self, msg):
        self.set_ctrl_name_pub.publish(self.ctrl_name)
        self.set_status_pub.publish("Moving arm.")
        self.buttons_disable_srv()
        self.run_controller(msg.data)
        self.set_status_pub.publish("Finished moving arm.")
        rospy.sleep(0.1)
        self.buttons_enable_srv()

    def run_controller(self, button_press):
        rospy.logerr("run_controller not implemented!")

    def disable_interface(self, status_message):
        self.buttons_disable_srv()
        self.set_status_pub.publish(status_message)

    def enable_interface(self, status_message):
        self.buttons_enable_srv()
        self.set_status_pub.publish(status_message)

