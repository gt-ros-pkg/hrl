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

##
# In this bag, give me messages from these topics
# @param file_name
# @param topics
def bag_reader(file_name, topics):
    f = open(file_name)
    tdict = {}
    for t in topics:
        tdict[t] = True
    for r in rosrecord.logplayer(f):
        if tdict.has_key(r[0]):
            yield r
    f.close()

