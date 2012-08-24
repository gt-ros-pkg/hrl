#!/usr/bin/python
import roslib; roslib.load_manifest('trf_learn')
import pypr2.pr2_utils as pu
import rcommander_ar_tour.rcommander_ar_server2 as rcs
import sys
import tf 
import rospy
import trf_learn_tool as tlt
import actionlib
import rcommander_ar_tour.msg as atmsg


class TRFBehaviorServer(rcs.BehaviorServer):

    def __init__(self, action_tag_database_name, ar_tag_database_name, 
                    path_to_rcommander_files, tf_listener, robot):
        rcs.BehaviorServer.__init__(self, action_tag_database_name, ar_tag_database_name, 
                    path_to_rcommander_files, tf_listener, robot)

        self.actserv_runactionid_train = actionlib.SimpleActionServer('run_actionid_train_mode', atmsg.RunScriptIDAction, 
                                    execute_cb=self.run_actionid_train_mode_cb, auto_start=False)
        self.actserv_runactionid_train.start()

    def run_actionid_train_mode_cb(self, req):
        self.actionid = req.actionid
        actserv = self.actserv_runactionid_train

        entry = self.action_marker_manager.marker_db.get(actionid)
        self.action_marker_manager.set_task_frame(actionid)

        #scan the loaded action, find the trf node and get its name
        def trf_param_set(graph_model, robot_object):
            for node_name in graph_model.real_states():
                node = graph_model.get_state(node_name)
                if isinstance(node, tlt.TRFLearnNode):
                    node.mode = 'train'
            state_machine = graph_model.create_state_machine(robot_object)
            return state_machine

        #Execute loaded action
        self.loaded_actions[entry['behavior_path']]['function'](actserv, state_machine_modifier=trf_param_set)

        #This will stop the publishing process
        self.action_marker_manager.set_task_frame(None) 

def run():
    rospy.init_node('rcommander_pr2_server_trf')
    action_database_name = sys.argv[1]
    ar_tag_database_name = sys.argv[2]
    path_to_rcommander_files = sys.argv[3]
    
    tflistener = tf.TransformListener()
    pr2 = pu.PR2(tflistener)
    def trf(action_tag_database_name, ar_tag_database_name, 
            path_to_rcommander_files, tf_listener, robot):
        return TRFBehaviorServer(action_tag_database_name, ar_tag_database_name, 
                path_to_rcommander_files, tf_listener, robot)
    rcs.run(pr2, tflistener, action_database_name, ar_tag_database_name, 
            path_to_rcommander_files, server_func=trf)

if __name__ == '__main__':
    run()
