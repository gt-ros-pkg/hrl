#!/usr/bin/python

import roslib
roslib.load_manifest('rfid_people_following')
import rospy

import rfid_people_following.db_rfid as db_rfid
from rfid_people_following.srv import rfid_gui as gui_srv

import time
import string
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

class RFID_GUI():
    def __init__(self, graphical=False, db = None):
        try:
            rospy.init_node('rfid_gui',anonymous=True)
        except rospy.ROSException:
            pass

        rospy.logout( 'rfid_gui: Initializing' )

        self.graphical = graphical
        self.images_path = 'db_images/'
        self.db = db

        self._srv = rospy.Service( '/rfid_gui/select', gui_srv, self.gui_response )

        rospy.logout( 'rfid_gui: Waiting for service calls.' )

    def gui_response(self, request):
        keys = request.keys

        keys = [ k for k in keys if self.db.has_key( k ) ]
        humanread = [ self.db[ k ]['humanread'] for k in keys ]
        imgs = [ self.db[ k ]['img'] for k in keys ]

        if len( keys ) == 0:
            rospy.logout( 'rfid_gui: No items to select from.  Fail.' )
            return False, ''
#         elif len( keys ) == 1:
#             rospy.logout( 'rfid_gui: Only one to select from.  Autoreturn: %s' % keys[0] )
#             return True, keys[0]

        # Get the selection (returned as RFID_Interface message)
        if self.graphical:
            success, selection = self.get_selection_graphical( keys, humanread, imgs )
        else:
            success, selection = self.get_selection_text( keys, humanread, imgs )

        return int(success), selection

                
    def get_selection_text( self, keys, humanread, imgs ):
        # keys has at least one items.
        ind = -1
        while ind.__class__ != (1).__class__ or ind < 0 or ind > len( keys ) - 1:
            print 'Make a selection:'
            for i, hr in enumerate( humanread ):
                print '\t(%d) %s' % (i, hr)
            resp = raw_input()
            try:
                ind = string.atoi( resp )
            except:
                ind = ''
                pass
        rospy.logout( 'rfid_gui: Returning selection: %s' % humanread[ ind ] )
        return True, keys[ ind ]

    def smart_scale(self, image):
        ims = np.array(image.get_size(),dtype='float')
        scale = self.imheight / np.max(ims)
        return pygame.transform.scale(image, tuple(ims*scale))

    def calc_blit_loc(self, image, center_pos):
        ims = np.array(image.get_size(), dtype='float')
        horiz = center_pos[0] - self.imheight/2 + (self.imheight - ims[0]) / 2.
        vert = center_pos[1] - self.imheight/2 + (self.imheight - ims[1]) / 2.
        return (horiz, vert)
                
    def get_selection_graphical( self, keys, humanread, imgs ):
        # keys has at least one items.
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
        images = []
        surfaces = []
        blit_loc = []
        for i, k in enumerate(keys):
            rospy.logout( 'rfid_gui: Loading image: %s' % self.images_path + imgs[i] )
            k_image = pygame.image.load(self.images_path + imgs[i]).convert()
            k_image = self.smart_scale(k_image)
            images.append( k_image )
            blit_loc.append( blit_pos[i] )
            #pygame.display.set_mode(tag_images[i].get_size())
            #tag_surfaces.append(pygame.display.get_surface())
            srf.blit(k_image, self.calc_blit_loc(k_image,blit_loc[i]))

        ind = self.get_selection( obj, images, humanread, blit_loc )
        
        rospy.logout( 'rfid_gui: Returning selection: %s' % humanread[ ind ] )
        return True, keys[ ind ]

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
            ind = np.argmin( np.sum( np.power( diffs, 2.0 ), axis = 1 ) )

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
                        ind = np.argmin( np.sum( np.power( diffs, 2.0 ), axis = 1 ) )
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
                 help='Use a graphical display.', default=False)
    opt, args = p.parse_args()

    gui = RFID_GUI( graphical = opt.graphical, db = db_rfid.db )
    rospy.spin()


#     while True:
#         print 'Waiting for robot action(s)...\n'
#         pc.publish_selected_id()
#         pc.ids = []
#         pc.ids_done = False


# On the Robot's side:
# ro = ROS_UI_Robot()
# ro.publish_tag_ids(['one','two','hello'])
# ro.receive_response()
