#!/usr/bin/python

import roslib; roslib.load_manifest('hrl_lib'); import rospy
# from hrl_lib.msg import String
# from hrl_lib.msg import RFID_Interface
import hrl_lib.util as ut
import hrl_lib.rutils as ru
import time
import pygame
import pygame.display
import pygame.locals
import pygame.transform
import numpy as np, math


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
    def __init__(self, init=True, graphical=False):
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
        self.graphical = graphical
        self.images_db = '/home/travis/svn/robot1/src/projects/08_03_dog_commands/images_db/'
        self.pps_db = ut.load_pickle('/home/travis/svn/robot1/src/projects/08_03_dog_commands/ele_rfid.pickle')

    def process_robot_request(self, rfid_interface_msg):
        msg = rfid_interface_msg

        # Get data out of msg to prevent overwriting!
        # From the robot, msg: tagIDs = ['id1','id2',...], humanread = [], actions = []
        tagIDs = msg.tagIDs
        humanread = []
        actions = []

        # Remote interface is responsible for populating other categories


        ids_in_pps = []
        for i, tag in enumerate( tagIDs ):
            if self.pps_db.has_key( tag ):
                ids_in_pps.append( tag )
                humanread.append( self.pps_db[tag]['properties']['name'] )
                acts = self.pps_db[tag]['actions'].keys()
                actions.append( acts )
#             else:
#                 humanread.append( tag )
#                 actions.append( ['fetch'] )

        # Get the selection (returned as RFID_Interface message)
        if self.graphical:
            selection = self.get_selection_graphical( ids_in_pps, humanread, actions )
        else:
            selection = self.get_selection_text( ids_in_pps, humanread, actions )
        
        # Publish the message
        self.ui_selection.publish( selection )
        print '\n\n Waiting for next request... \n\n'

                
    def get_selection_text( self, tagIDs, humanread, actions ):
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

    def smart_scale(self, image):
        ims = np.array(image.get_size(),dtype='float')
        scale = self.imheight / np.max(ims)
        return pygame.transform.scale(image, tuple(ims*scale))

    def calc_blit_loc(self, image, center_pos):
        ims = np.array(image.get_size(), dtype='float')
        horiz = center_pos[0] - self.imheight/2 + (self.imheight - ims[0]) / 2.
        vert = center_pos[1] - self.imheight/2 + (self.imheight - ims[1]) / 2.
        return (horiz, vert)
                
    def get_selection_graphical(self, tagIDs, humanread, actions):
        pygame.init()
        self.s_width = 600
        self.s_height = 700
        srf = pygame.display.set_mode((self.s_width,self.s_height))
        fps = 100
        loopFlag = True
        clk = pygame.time.Clock()
        obj = [srf, fps, clk]
        self.imheight = 175.

        w = self.s_width * 1.0
        h = self.s_height * 1.0
        blit_pos = [[ w/3-w/6,       h/3-h/6],
                    [ w/3-w/6+w/3,   h/3-h/6],
                    [ w/3-w/6+2*w/3, h/3-h/6],
                    [ w/3-w/6,       h/3-h/6+h/3],
                    [ w/3-w/6+w/3,   h/3-h/6+h/3],
                    [ w/3-w/6+2*w/3, h/3-h/6+h/3],
                    [ w/3-w/6,       h/3-h/6+2*h/3],
                    [ w/3-w/6+w/3,   h/3-h/6+2*h/3],
                    [ w/3-w/6+2*w/3, h/3-h/6+2*h/3]]
        tag_images = []
        tag_surfaces = []
        blit_loc = []
        for i, tag in enumerate(tagIDs):
            print 'Loading image for tag ', tag
            tag_image = pygame.image.load(self.images_db + tag + '.jpg').convert()
            tag_image = self.smart_scale(tag_image)
            tag_images.append( tag_image )
            blit_loc.append( blit_pos[i] )
            #pygame.display.set_mode(tag_images[i].get_size())
            #tag_surfaces.append(pygame.display.get_surface())
            srf.blit(tag_image, self.calc_blit_loc(tag_image,blit_loc[i]))
        tag_ind = self.get_selection( obj, tag_images, humanread, blit_loc )
        print 'Selected tag ', tag_ind, ': ', humanread[tag_ind]

        act_images = []
        act_surfaces = []
        blit_loc = []
        for i, act in enumerate(actions[tag_ind]):
            print 'Loading image for act ', act
            act_image = pygame.image.load(self.images_db + tag + act + '.jpg').convert()
            act_image = self.smart_scale(act_image)
            act_images.append( act_image )
            blit_loc.append( blit_pos[i] )
            #pygame.display.set_mode(tag_images[i].get_size())
            #tag_surfaces.append(pygame.display.get_surface())
            srf.blit(act_image, self.calc_blit_loc(tag_image,blit_loc[i]))
        act_ind = self.get_selection( obj, act_images, actions[tag_ind], blit_loc )
        print 'Selected action ', act_ind, ': ', actions[tag_ind][act_ind]

        retmsg = RFID_Interface()
        retmsg.tagIDs = [ tagIDs[tag_ind] ]
        retmsg.humanread = [ humanread[tag_ind] ]
        retmsg.actions = [ actions[tag_ind][act_ind] ]
        return retmsg

    def put_bottom_text( self, srf, text ):
        font = pygame.font.Font(None, 25)
        box = font.render(text, 1,(10, 10, 10, 0))
        ts = box.get_size()
        horiz = self.s_width / 2.0 - ts[0]/2.0
        vt = self.s_height - 50.0 - ts[1]/2.0
        srf.blit(box, (horiz, vt))
        return True

    def draw_rect(self, srf, blit_loc):
        width = self.imheight * 1.10
        height = self.imheight *1.10
        horiz = blit_loc[0] - width / 2.0
        vert = blit_loc[1] - height / 2.0
        pygame.draw.rect(srf, (255, 0, 0), (horiz, vert, width, height))
        width = self.imheight * 1.01
        height = self.imheight *1.01
        horiz = blit_loc[0] - width / 2.0
        vert = blit_loc[1] - height / 2.0
        pygame.draw.rect(srf, (255, 255, 255), (horiz, vert, width, height))
        return True
    
    def get_selection( self, obj, images, humanread, blit_loc ):
        [srf, fps, clk] = obj
        loopFlag = True
        ind = 0
        pos = (0,0)
        while loopFlag:
            # Clear the screen
            srf.fill((255,255,255))
            
            diffs = np.array(blit_loc) - np.array(pos)
            ind = np.argmin( ut.norm( diffs.T ))
            self.put_bottom_text( srf, humanread[ind] )

            self.draw_rect(srf, blit_loc[ind])

            for i, image in enumerate(images):
                srf.blit(image, self.calc_blit_loc(image,blit_loc[i]))

            #print 'going'
            pygame.display.flip()

            events = pygame.event.get()
            for e in events:
                if e.type==pygame.locals.QUIT:
                    loopFlag=False
                if e.type==pygame.locals.KEYDOWN:
                    if e.key == 27: # Esc
                        loopFlag=False
                if e.type == pygame.locals.MOUSEMOTION:
                    pos = e.pos
                if e.type==pygame.locals.MOUSEBUTTONDOWN:
                    if e.button == 1:
                        # left button
                        pos = e.pos
                        diffs = np.array(blit_loc) - np.array(pos)
                        ind = np.argmin( ut.norm( diffs.T ))
                        loopFlag = False

            clk.tick(fps)
        srf.fill((255,255,255))
        pygame.display.flip()
        clk.tick(fps)
        return ind
        

# pc = ROS_UI_PC(init = False, graphical = True)
# pc.get_selection_graphical(['LightSwitch1','LightSwitch1'],
#                            ['lightswitch','LightSwitch1'],
#                            [['on','off'],['on','off']])


if __name__ == '__main__':
    import optparse

    p = optparse.OptionParser()
    p.add_option('-d', action='store_true', dest='graphical',
                 help='Use a graphical display.')
    p.add_option('-g', action='store_true', dest='client',
                 help='Build Client?', default=False)
    opt, args = p.parse_args()

    if opt.client:
        pc = ROS_UI_PC(graphical = opt.graphical)
        pc.get_selection_graphical( ['person      '
        rospy.spin()
    else:
        ro = ROS_UI_Robot()
        ro.publish_tag_ids([ 'one', 'two', 'hello' ])
        ro.receive_response()

#     while True:
#         print 'Waiting for robot action(s)...\n'
#         pc.publish_selected_id()
#         pc.ids = []
#         pc.ids_done = False


# On the Robot's side:
# ro = ROS_UI_Robot()
# ro.publish_tag_ids(['one','two','hello'])
# ro.receive_response()
