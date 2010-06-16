import roslib
roslib.load_manifest('hrl_opencv')
import cv

def print_mat(m):
    for j in range(m.height):
        for i in range(m.width):
            print m[j,i], ' ',
        print ''


##Display a list of OpenCV images tiled across the screen
#with maximum width of max_x and maximum height of max_y
# @param save_images - will save the images(with timestamp) (NOT IMPLEMENTED)
def display_images(image_list, max_x = 1200, max_y = 1000, save_images_flag=False):
    if image_list == []:
        return
    loc_x, loc_y = 0, 0
    wins = []
#    if save_images_flag:
#        save_images(image_list)

    for i, im in enumerate(image_list):
        window_name = 'image %d' % i
        wins.append((window_name, im))
        cv.NamedWindow(window_name, cv.CV_WINDOW_AUTOSIZE)
        cv.MoveWindow(window_name, loc_x, loc_y)
        loc_x = loc_x + im.width
        if loc_x > max_x:
            loc_x = 0
            loc_y = loc_y + im.height
            if loc_y > max_y:
                loc_y = 0
    while True:
        for name, im in wins:
            cv.ShowImage(name, im)
        keypress = cv.WaitKey(10)
        if keypress & 255 == 27:
            break





