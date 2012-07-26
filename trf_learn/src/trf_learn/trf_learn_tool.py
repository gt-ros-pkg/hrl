#import roslib; roslib.load_manifest('rcommander_core')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import smach
import os.path as pt
import rcommander.graph_model as gm


class TRFLearnTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'trf_learner', 'TRF Learn', TRFLearnNode)
        self.child_gm = None

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.filename_edit = QLineEdit(pbox)
        self.filename_edit.setText("...")
        self.open_file_button = QPushButton(pbox)
        self.open_file_button.setText('Select...')
        self.rcommander.connect(self.open_file_button, SIGNAL('clicked()'), self.open_file_cb)

        formlayout.addRow('&Filename', self.filename_edit)
        formlayout.addRow(self.open_file_button)

    def open_file_cb(self):
        dialog = QFileDialog(self.rcommander, 'Open State Machine', '~')
        dialog.setFileMode(QFileDialog.Directory)
        dialog.setViewMode(QFileDialog.List)

        #Fix messed up bug, weird interaction between file dialog and rospy!
        import rospy.core as rpc
        rpc._shutdown_flag = False

        if dialog.exec_():
            filenames = dialog.selectedFiles()
            filename = str(filenames[0])
            self.filename_edit.setText(filename)

    def new_node(self, name=None):
        child_gm = self.child_gm
        self.child_gm = None

        if (child_gm == None) and (str(self.filename_edit.text()) != '...'):
            #nname = pt.split(str(self.filename_edit.text()))[1]
            print 'state machine tool loading', self.filename_edit.text()
            child_gm = gm.GraphModel.load(str(self.filename_edit.text()))
            #curr_document = gm.FSMDocument(child_gm.get_document().get_name(), modified=True, real_filename=False)
            curr_document = gm.FSMDocument(name, modified=True, real_filename=False)
            child_gm.set_document(curr_document)
        else:
            if child_gm != None:
                curr_document = gm.FSMDocument(name, modified=True, real_filename=False)
                child_gm.set_document(curr_document)
                return TRFLearnNode(name, child_gm)
            if name == None:
                nname = self.name + str(self.counter)
                return TRFLearnNode(nname, None)
            else:
                #raise RuntimeError('Need to specify filename.')
                return None

        return TRFLearnNode(name, child_gm)

    def set_node_properties(self, my_node):
        self.child_gm = my_node.child_gm
        if self.child_gm == None:
            return
        if self.child_gm.get_document() != None:
            fname = self.child_gm.get_document().get_filename()
            if fname != None:
                self.filename_edit.setText(fname)

    def reset(self):
        self.filename_edit.setText("...")
        self.child_gm = None


class TRFLearnNode(tu.EmbeddableState):

    def __init__(self, name, child_gm):
        tu.EmbeddableState.__init__(self, name, child_gm)

    def get_smach_state(self):
        return TRFLearnNodeSmach(self.child_gm)

    def recreate(self, graph_model):
        #return TRFLearnNode(graph_model.document.get_name(), graph_model)
        return TRFLearnNode(self.get_name(), graph_model)

class TRFLearnNodeSmach(smach.State):

    def __init__(self, child_gm):
        self.child_gm = child_gm

    def set_robot(self, robot):
        self.robot = robot
        input_keys = []
        output_keys = []
        outcomes = []
        if self.child_gm != None:
            sm = self.child_gm.create_state_machine(robot)
            input_keys = list(sm.get_registered_input_keys())
            output_keys = list(sm.get_registered_output_keys())
            outcomes = list(sm.get_registered_outcomes()) + ['preempted']
        smach.State.__init__(self, outcomes = outcomes, input_keys = input_keys, output_keys = output_keys)


    def execute(self, userdata):

        #Get current mode, should we record or not?

        #Get the current classifier ID

        #Classify

        #Set it on ar tour server, wait for result

        #Run success function part 1

        child_gm = self.child_gm
        sm = child_gm.create_state_machine(self.robot, userdata=userdata._ud)
        child_gm.run(self.child_gm.get_start_state(), state_machine=sm)
        rthread = child_gm.sm_thread['run_sm']

        preempted = False
        r = rospy.Rate(30)
        while True:
            if rthread.exception != None:
                raise rthread.exception

            if rthread.outcome != None:
                rospy.loginfo('State Machine Node: child node finished with outcome ' + rthread.outcome)
                break

            if not rthread.isAlive():
                rospy.loginfo('State Machine Node: child node died')
                break

            if self.preempt_requested():
                rospy.loginfo('State Machine Node: preempt requested')
                rthread.preempt()
                self.service_preempt()
                preempted = True
                break

            r.sleep()

        #Run success function part2

        #Return success/failure based on success function

        #child_gm.sm_thread = {} #Reset sm thread dict
        if preempted:
            return 'preempted'
        else:
            return rthread.outcome


