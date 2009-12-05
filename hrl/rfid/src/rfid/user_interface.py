#!/usr/bin/python

import roslib; roslib.update_path('hrl_lib'); import rospy
from hrl_lib.msg import String
from hrl_lib.msg import RFID_Interface
import hrl_lib.util as ut
import hrl_lib.rutils as ru
import time


class ROS_UI_Robot():
    def __init__(self, init=True):
        if init:
            try:
                print 'Initializing RFID UI on Robot'
                rospy.init_node('RFID_UI_Robot',anonymous=True)
            except rospy.ROSException:
                pass
        self.tag_pub = rospy.Publisher('/hrl/ele/UI_Robot',RFID_Interface)
        print 'RFID UI on Robot Running'

        print 'Connecting to RFID UI on PC'
        self.selected_tag = ''
        self.selection = []
        rospy.Subscriber('/hrl/ele/UI_PC_Selection', RFID_Interface, self.selection_cb, None, 1)

#     def publish_tag_ids(self, tagIDs):
#         ''' tagIDs is a list of strings '''
#         for tag in tagIDs:
#             rt = String()
#             rt.data = tag
#             self.tag_pub.publish( rt )
#             time.sleep(0.2)
#         rt.data = '-1'
#         self.tag_pub.publish( rt )
#         time.sleep(0.002)
    def publish_tag_ids(self, tagIDs):
        pub_msg = RFID_Interface()
        pub_msg.tagIDs = tagIDs
        pub_msg.humanread = []
        pub_msg.actions = []

        self.tag_pub.publish( pub_msg )
        time.sleep(0.002)

#     def selection_cb(self, pc_msg):
#         self.selected_tag = pc_msg.data
#         print 'Received tag selection from UI: ', self.selected_tag
#         return self.selected_tag
    def selection_cb(self, rfid_msg):
        tagID = rfid_msg.tagIDs[0]
        action = rfid_msg.actions[0]
        humanread = rfid_msg.humanread[0]
        self.selection = [tagID, humanread, action]
        #print 'Received selection from UI: ', self.selection

    def receive_response(self):
        while self.selection == []:
            time.sleep(0.02)
        rv = self.selection
        self.selection = []
        return rv

#     def receive_response(self):
#         rt = String()
#         rt = self.listener.read()
#         return rt.data

class ROS_UI_PC():
    def __init__(self, init=True):
        if init:
            try:
                print 'Initializing RFID UI on PC'
                rospy.init_node('RFID_UI_PC',anonymous=True)
            except rospy.ROSException:
                pass
        #self.tag_pub = rospy.Publisher('/hrl/ele/UI_PC',String)
        self.ui_selection = rospy.Publisher('/hrl/ele/UI_PC_Selection',RFID_Interface)
        print 'RFID UI on PC Running'

        print 'Connecting to RFID UI on Robot'
        #rospy.Subscriber('/hrl/ele/UI_Robot', String, self.selection_cb, None, 1)
        rospy.Subscriber('/hrl/ele/UI_Robot', RFID_Interface, self.process_robot_request, None, 1)
#         self.listener = ru.GenericListener('RFID_UI_Robot', String,
#                                            '/hrl/ele/UI_Robot', 20)
        self.ids = []
        self.ids_done = False

    def process_robot_request(self, rfid_interface_msg):
        msg = rfid_interface_msg

        # Get data out of msg to prevent overwriting!
        # From the robot, msg: tagIDs = ['id1','id2',...], humanread = [], actions = []
        tagIDs = msg.tagIDs
        humanread = []
        actions = []

        # Remote interface is responsible for populating other categories
        pps_db = ut.load_pickle('/home/haidai/svn/robot1/src/projects/08_03_dog_commands/ele_rfid.pickle')

        for i, tag in enumerate( tagIDs ):
            if pps_db.has_key( tag ):
                humanread.append( pps_db[tag]['properties']['name'] )
                acts = pps_db[tag]['actions'].keys()
                actions.append( acts )
            else:
                humanread.append( tag )
                actions.append( ['fetch'] )

        # Get the selection (returned as RFID_Interface message)
        selection = self.get_selection( tagIDs, humanread, actions )
        
        # Publish the message
        self.ui_selection.publish( selection )
        print '\n\n Waiting for next request... \n\n'
                
    def get_selection( self, tagIDs, humanread, actions ):
        print '\n\nSelect a tag:'
        if len(tagIDs) == 1:
            print '\tOnly one option available: ', humanread[0]
            tag_ind = 0
        else:
            for i, tag in enumerate( tagIDs ):
                print '\t(%d) %s' % (i, humanread[i])
            tag_ind = int(raw_input())

        print 'Select an action for that tag:'
        if len( actions[tag_ind] ) == 1:
            print '\tOnly one option available: ', actions[tag_ind][0]
            act_ind = 0
        else:
            for i, act in enumerate( actions[tag_ind] ):
                print '\t(%d) %s' % (i, actions[tag_ind][i])
            act_ind = int(raw_input())

        retmsg = RFID_Interface()
        retmsg.tagIDs = [ tagIDs[tag_ind] ]
        retmsg.humanread = [ humanread[tag_ind] ]
        retmsg.actions = [ actions[tag_ind][act_ind] ]
        return retmsg
            


if __name__ == '__main__':
    pc = ROS_UI_PC()
    while not rospy.is_shutdown():
        time.sleep(0.2)
#     while True:
#         print 'Waiting for robot action(s)...\n'
#         pc.publish_selected_id()
#         pc.ids = []
#         pc.ids_done = False


# On the Robot's side:
# ro = ROS_UI_Robot()
# ro.publish_tag_ids(['one','two','hello'])
# ro.receive_response()
