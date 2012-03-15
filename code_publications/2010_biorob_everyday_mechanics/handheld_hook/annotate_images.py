#    print 'Following are the key that you can use:'
#    print '1. Hit ESC to end'
#    print '2. Hit - to decrease the size of the image'
#    print '3. Hit + (without the shift key) to increase the image size'
#    print '4. Hit c to detect chessboard corners'

import sys, time
import numpy as np, math
import glob, commands

import roslib; roslib.load_manifest('modeling_forces')
import cv
from cv_bridge.cv_bridge import CvBridge, CvBridgeError

import hrl_lib.util as ut

# call func on the 8 neighbors of x,y and on x,y
def call_neighborhood(func,func_params,x,y):
    func(x+0,y+0,*func_params)
    func(x+1,y+0,*func_params)
    func(x+1,y-1,*func_params)
    func(x+0,y-1,*func_params)
    func(x-1,y-1,*func_params)
    func(x-1,y+0,*func_params)
    func(x-1,y+1,*func_params)
    func(x-0,y+1,*func_params)
    func(x+1,y+1,*func_params)


def on_mouse(event, x, y, flags, param):
    im, clicked_list = param[0], param[1]
    scale_factor = param[2]
    if event == cv.CV_EVENT_LBUTTONDOWN:
        #call_neighborhood(cv.SetPixel,(im,(255,0,0)),x,y)
        clicked_list.append((x/scale_factor, y/scale_factor)) # these are in mm


def annotate_image(cv_im, mech_info_dict, dir):
    #cv_im = cv.LoadImage(sys.argv[1], cv.CV_LOAD_IMAGE_GRAYSCALE)
    size = cv.GetSize(cv_im)

    print 'Image size:', size[0], size[1] # col, row
    wnd = 'Image Annotate'
    cv.NamedWindow(wnd, cv.CV_WINDOW_AUTOSIZE)
    disp_im = cv.CloneImage(cv_im)
    new_size = (size[0]/2, size[1]/2)
    scale_factor = 1

    checker_origin_height = mech_info_dict['height']
    # chesscoard corners mat
    cb_dims = (5, 8) # checkerboard dims. (x, y)
    sq_sz = 19 # size of checkerboard in real units.
    cb_offset = 500,500
    cb_coords = cv.CreateMat(2, cb_dims[0]*cb_dims[1], cv.CV_64FC1)
    n = 0
    for r in range(cb_dims[1]):
        for c in range(cb_dims[0]):
            cb_coords[0,n] = c*sq_sz + cb_offset[0] # x coord
            cb_coords[1,n] = r*sq_sz + cb_offset[1] # y coord
            n += 1

    clicked_list = []
    recreate_image = False
    mechanism_calc_state = 0
    mechanism_calc_dict = {}
    while True:
        for p in clicked_list:
            x,y = p[0]*scale_factor, p[1]*scale_factor
            cv.Circle(disp_im, (x,y), 3, (255,0,0))
        cv.ShowImage(wnd, disp_im)
        k = cv.WaitKey(10)
        k = k%256

        if k == 27:
            # ESC
            break
        elif k == ord('='):
            # + key without the shift
            scale_factor = scale_factor*1.2
            recreate_image = True
        elif k == ord('-'):
            # - key
            scale_factor = scale_factor/1.2
            recreate_image = True
        elif k == ord('c'):
            # find chessboard corners.
            succ, corners = cv.FindChessboardCorners(disp_im, cb_dims)
            if succ == 0:
                print 'Chessboard detection FAILED.'
            else:
                # chessboard detection was successful
                cv.DrawChessboardCorners(disp_im, cb_dims, corners, succ)
                cb_im = cv.CreateMat(2, cb_dims[0]*cb_dims[1], cv.CV_64FC1)
                corners_mat = np.array(corners).T
                n = 0
                for r in range(cb_dims[1]):
                    for c in range(cb_dims[0]):
                        cb_im[0,n] = corners_mat[0,n] # x coord
                        cb_im[1,n] = corners_mat[1,n] # y coord
                        n += 1
                H = cv.CreateMat(3, 3, cv.CV_64FC1)
                H1 = cv.FindHomography(cb_im, cb_coords, H)
                Hnp = np.reshape(np.fromstring(H1.tostring()), (3,3))
                print 'Homography:'
                print Hnp

                d = cv.CloneImage(disp_im)
                cv.WarpPerspective(d, disp_im, H1, cv.CV_WARP_FILL_OUTLIERS)
                cv_im = cv.CloneImage(disp_im)
        elif k == ord('1'):
            # calculate width of the mechanism
            del clicked_list[:]
            cv.SetMouseCallback(wnd, on_mouse, (disp_im, clicked_list, scale_factor))
            recreate_image = True
            print 'Click on two endpoints to mark the width of the mechanism'
            mechanism_calc_state = 1
        elif k == ord('2'):
            # distance of handle from the hinge
            del clicked_list[:]
            cv.SetMouseCallback(wnd, on_mouse, (disp_im, clicked_list, scale_factor))
            recreate_image = True
            print 'Click on handle and hinge to compute distance of handle from hinge.'
            mechanism_calc_state = 2
        elif k == ord('3'):
            # height of the handle above the ground
            del clicked_list[:]
            cv.SetMouseCallback(wnd, on_mouse, (disp_im, clicked_list, scale_factor))
            recreate_image = True
            print 'Click on handle top and bottom to compute height of handle above the ground.'
            mechanism_calc_state = 3
        elif k == ord('4'):
            # top and bottom edge of the mechanism
            del clicked_list[:]
            cv.SetMouseCallback(wnd, on_mouse, (disp_im, clicked_list, scale_factor))
            recreate_image = True
            print 'Click on top and bottom edges of the mechanism.'
            mechanism_calc_state = 4
        elif k ==ord('d'):
            # display the calculated values
            print 'mechanism_calc_dict:', mechanism_calc_dict
            print 'mech_info_dict:', mech_info_dict
        elif k == ord('s'):
            # save the pkl
            ut.save_pickle(mechanism_calc_dict, dir+'/mechanism_calc_dict.pkl')
            print 'Saved the pickle'
        #elif k != -1:
        elif k != 255:
            print 'k:', k

        if recreate_image:
            new_size = (int(size[0]*scale_factor),
                        int(size[1]*scale_factor))
            disp_im = cv.CreateImage(new_size, cv_im.depth, cv_im.nChannels)
            cv.Resize(cv_im, disp_im)
            cv.SetMouseCallback(wnd, on_mouse, (disp_im, clicked_list, scale_factor))
            recreate_image = False

        if len(clicked_list) == 2:
            if mechanism_calc_state == 1:
                w = abs(clicked_list[0][0] - clicked_list[1][0])
                print 'Width in mm:', w
                mechanism_calc_dict['mech_width'] = w/1000.
            if mechanism_calc_state == 2:
                w = abs(clicked_list[0][0] - clicked_list[1][0])
                print 'Width in mm:', w
                mechanism_calc_dict['handle_hinge_dist'] = w/1000.
            if mechanism_calc_state == 3:
                p1, p2 = clicked_list[0], clicked_list[1]
                h1 = (cb_offset[1] - p1[1])/1000. + checker_origin_height
                h2 = (cb_offset[1] - p2[1])/1000. + checker_origin_height
                mechanism_calc_dict['handle_top'] = max(h1, h2)
                mechanism_calc_dict['handle_bottom'] = min(h1, h2)
            if mechanism_calc_state == 4:
                p1, p2 = clicked_list[0], clicked_list[1]
                h1 = (cb_offset[1] - p1[1])/1000. + checker_origin_height
                h2 = (cb_offset[1] - p2[1])/1000. + checker_origin_height
                mechanism_calc_dict['mechanism_top'] = max(h1, h2)
                mechanism_calc_dict['mechanism_bottom'] = min(h1, h2)

            mechanism_calc_state = 0




if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p.add_option('-d', '--dir', action='store', default='',
                 type='string', dest='dir', help='mechanism directory')
    opt, args = p.parse_args()

#    dir_list = commands.getoutput('ls -d %s/*/'%(opt.dir)).splitlines()
#    dir_list = dir_list[:]
#    for direc in dir_list:
    img_list = glob.glob(opt.dir+'/*.jpg')
    mech_info_dict = ut.load_pickle(opt.dir+'/mechanism_info.pkl')
    for img in img_list:
        cv_im = cv.LoadImage(img)
        annotate_image(cv_im, mech_info_dict, opt.dir)

    sys.exit()

#    cv.DestroyAllWindows()
    #print 'k:', chr(k)


