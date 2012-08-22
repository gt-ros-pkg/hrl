#import roslib; roslib.load_manifest('rcommander_core')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import smach
import rcommander_ar_tour.srv as rsrv

class ActionIDLookupTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'actionid_lookup', 'Action ID Lookup', ActionIDLookupState)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return ActionIDLookupState(nname)

    def set_node_properties(self, my_node):
        pass

    def reset(self):
        pass

class ActionIDLookupState(tu.StateBase):

    def __init__(self, name):
        tu.StateBase.__init__(self, name, outputs={name: dict})

    def get_smach_state(self):
        return ActionIDLookupStateSmach(self.get_name())

class ActionIDLookupStateSmach(smach.State):

    def __init__(self, output_variable_name):
        smach.State.__init__(self, outcomes=['done'], input_keys=[], output_keys=[output_variable_name])
        self.output_variable_name = output_variable_name
        self.get_action_id = rospy.ServiceProxy('get_active_action_id', rsrv.GetActiveActionID)

    def execute(self, userdata):
        resp = self.get_action_id()
        ret = {'actionid': resp.actionid}
        exec("userdata.%s = ret" % self.output_variable_name)
        return 'done'


