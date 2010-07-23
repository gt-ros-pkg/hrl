import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import rosrecord
import sys

f = open(sys.argv[1])
i = 0
for topic, message, time in rosrecord.logplayer(f):
    i = i + 1
    print topic, time
    if i > 10:
        break
f.close()
