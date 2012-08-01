#import roslib; roslib.load_manifest('rcommander_core')
import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu
import smach
import os.path as pt
import rcommander.graph_model as gm
import rcommander_ar_tour.msg as atmsg


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

        self.success_detector_box = tu.ComboBox()
        self.success_detector_box.create_box()
        self.success_detector_box.combobox.addItem('light_switch')
        self.success_detector_box.combobox.addItem('drawer_pull')
        self.success_detector_box.set_text('light_switch')

        formlayout.addRow('&Filename', self.filename_edit)
        formlayout.addRow(self.open_file_button)

    def open_file_cb(self):
        dialog = QFileDialog(self.rcommander, 'Open State Machine', '~')
        dialog.setFileMode(QFileDialog.Directory)
        dialog.setViewMode(QFileDialog.List)

        #Fix messed up bug, weird interaction between file dialog and rospy!
        #import rospy.core as rpc
        #rpc._shutdown_flag = False

        if dialog.exec_():
            filenames = dialog.selectedFiles()
            filename = str(filenames[0])
            self.filename_edit.setText(filename)

    def new_node(self, name=None):
        child_gm = self.child_gm
        self.child_gm = None

        if (child_gm == None) and (str(self.filename_edit.text()) != '...'):
            #nname = pt.split(str(self.filename_edit.text()))[1]
            #print 'state machine tool loading', self.filename_edit.text()
            child_gm = gm.GraphModel.load(str(self.filename_edit.text()))
            curr_document = gm.FSMDocument(name, modified=True, real_filename=False)
            child_gm.set_document(curr_document)
        else:
            if child_gm != None:
                curr_document = gm.FSMDocument(name, modified=True, real_filename=False)
                child_gm.set_document(curr_document)
                return TRFLearnNode(name, child_gm, self.success_detector_box.text())
            if name == None:
                nname = self.name + str(self.counter)
                return TRFLearnNode(nname, None, self.success_detector_box.text())
            else:
                return None

        return TRFLearnNode(name, child_gm, self.success_detector_box.text())

    def set_node_properties(self, my_node):
        self.child_gm = my_node.child_gm
        if self.child_gm == None:
            return
        if self.child_gm.get_document() != None:
            fname = self.child_gm.get_document().get_filename()
            if fname != None:
                self.filename_edit.setText(fname)
        self.success_detector_box.set_text(my_node.success_detector)

    def reset(self):
        self.filename_edit.setText("...")
        self.child_gm = None
        self.success_detector_box.set_text('light_switch')


class TRFLearnNode(tu.EmbeddableState):

    def __init__(self, name, child_gm, success_detector):
        tu.EmbeddableState.__init__(self, name, child_gm)
        self.set_remapping_for('mode', 'learning_mode')
        self.success_detector = success_detector

    def get_smach_state(self):
        return TRFLearnNodeSmach(self.child_gm)

    def recreate(self, graph_model):
        #return TRFLearnNode(graph_model.document.get_name(), graph_model)
        return TRFLearnNode(self.get_name(), graph_model, self.success_detector)


def wait_for_tf_change(tf_listener, desired, destination_frame, source_frame, timeout):
    start_time = rospy.get_time()
    changed = False
    while (not rospy.is_shutdown()) and not changed:
        current_tf = tfu.transform(destination_frame, source_frame, tf_listener)
        changed = np.all(np.abs(desired - current_tf) < .001)
        if (rospy.get_time() - start_time) > timeout:
            break
    return changed

def pose_to_tup(p):
    return [p.position.x, p.position.y, p.position.z], \
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

class TRFLearnNodeSmach(smach.State):

    def __init__(self, child_gm, success_detector):
        self.child_gm = child_gm
        self.success_detector = success_detector

        #Handles request to recognize the pose of an object.
        self.recognize_pose_srv = rospy.ServiceProxy('recognize_pose', atmsg.RecognizePose)

        #Messages that tells this node about the outcome of actions.
        self.action_result_srv = rospy.ServiceProxy('action_result', atmsg.ActionResult)
        self.classify_success_snapshot = rospy.ServiceProxy('classify_success_snapshot', tm.ClassifySuccessSnapshot)
        self.classify_success = rospy.ServiceProxy('classify_success', tm.ClassifySuccess)
        self.set_behavior_pose = rospy.ServiceProxy('set_behavior_pose', rsrv.SetBehaviorPose)

    def set_robot(self, robot):
        self.robot = robot
        self.tf_listener = robot.tf_listener
        input_keys = []
        output_keys = []
        outcomes = []
        if self.child_gm != None:
            sm = self.child_gm.create_state_machine(robot)
            input_keys = list(sm.get_registered_input_keys())
            output_keys = list(sm.get_registered_output_keys())
            outcomes = list(sm.get_registered_outcomes()) + ['preempted']
        smach.State.__init__(self, outcomes = outcomes, 
                input_keys = input_keys, output_keys = output_keys)

    def execute(self, userdata):
        #Get current mode, should we record or not?
        recognition_mode = userdata.mode 

        #Get the current classifier ID
        actionid = userdata.actionid
        
        #Classify
        resp = self.recognize_pose_srv(actionid, recognition_mode)
        new_loc       = resp.posestamped
        instance_info = resp.pickled_dict

        #Set it on ar tour server, wait for result
        self.set_behavior_pose(actionid, new_loc)

        #Make sure that the transform was set correctly
        if not (wait_for_tf_change(self.tf_listener, tfu.tf_as_matrix(pose_to_tup(new_loc.pose)),
                'map', 'task_frame', 10)):
            raise Exception('Unable to change tf frame using recognition results!')

        #Run success function part 1
        success_request_id = self.classify_success_snapshot(actionid, self.success_detector).request_id

        child_gm = self.child_gm
        sm = child_gm.create_state_machine(self.robot, userdata=userdata._ud)
        child_gm.run(self.child_gm.get_start_state(), state_machine=sm)
        rthread = child_gm.sm_thread['run_sm']

        preempted = False
        r = rospy.Rate(30)
        success = True
        while not rospy.is_shutdown():
            if rthread.exception != None:
                raise rthread.exception

            if rthread.outcome != None:
                rospy.loginfo('State Machine Node: child node finished with outcome ' \
                        + rthread.outcome)
                if None == re.compile('^succeeded\d+').match(rthread.outcome):
                    success = False
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
        success = self.classify_success(success_request_id).success

        #Return success/failure based on success function
        self.action_result_srv(actionid, success, recognition_mode)

        #child_gm.sm_thread = {} #Reset sm thread dict
        if preempted:
            return 'preempted'
        else:
            return rthread.outcome

