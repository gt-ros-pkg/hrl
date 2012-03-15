import roslib; roslib.load_manifest('hai_sandbox')
import rospy
from geometry_msgs.msg import PoseStamped

def ros_to_dict(msg):
    d = {}
    for f in msg.__slots__:
        val = eval('msg.%s' % f)
        methods = dir(val)
        if 'to_time' in methods:
            val = eval('val.to_time()')
        elif '__slots__' in methods:
            val = ros_to_dict(val)
        d[f] = val
    return d

a = PoseStamped()
print ros_to_dict(a)
