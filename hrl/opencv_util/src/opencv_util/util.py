import roslib
roslib.update_path('opencv_util')
from image_msgs.msg import Image as RosImage
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

import opencv.adaptors as ad
import Image as pil
import numpy as np

def ros2cv(image):
    #ros to pil then pil to opencv
    if image.encoding != 'bgr' or image.encoding != 'rgb':
        raise RuntimeError('Unsupported format "%s"' % image.encoding)
    if image.depth != 'uint8':
        raise RuntimeError('Unsupported depth "%s"' % image.depth)

    height    = image.uint8_data.layout.dim[0].size
    width     = image.uint8_data.layout.dim[1].size
    channels  = image.uint8_data.layout.dim[2].size
    np_image  = np.reshape(np.fromstring(image.uint8_data.data, dtype='uint8', count=height*width*channels), [height, width, channels])
    return ad.NumPy2Ipl(np_image)

if __name__ == '__main__':
    import opencv.highgui as hg
    import rospy
    from photo.srv import *
    #a = create_ros_image()
    #print a

    rospy.wait_for_service('/photo/capture')
    say_cheese = rospy.ServiceProxy('/photo/capture', Capture)
    ros_img = say_cheese().image
    print dir(ros_img)
    cv_img = ros2cv(ros_img)
    hg.cvSaveImage('test.png', cv_img)

#def create_ros_image(width=1, height=1, channels=2, data='12'):
#    d1 = MultiArrayDimension(label='height',   size=height,   stride=width*height*channels)
#    d2 = MultiArrayDimension(label='width',    size=width,    stride=width*channels)
#    d3 = MultiArrayDimension(label='channels', size=channels, stride=channels)
#
#    layout     = MultiArrayLayout(dim = [d1,d2,d3])
#    multiarray = UInt8MultiArray(layout=layout, data=data)
#    return RosImage(label='image', encoding='bgr', depth='uint8', uint8_data=multiarray)
