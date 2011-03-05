#!/usr/bin/env python
from object_manipulator.cluster_bounding_box_finder import ClusterBoundingBoxFinder
from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult




cbbf = ClusterBoundingBoxFinder()
object_detector = rospy.ServiceProxy("/object_detection", TabletopDetection)
detects = object_detector(True, False).detection
object_detector.close()
if detects.result != 4:
    err("Detection failed (err %d)" % (detects.result))
table_z = detects.table.pose.pose.position.z
objects = []
for cluster in detects.clusters:
    (object_points, 
     object_bounding_box_dims, 
     object_bounding_box, 
     object_to_base_link_frame, 
     object_to_cluster_frame) = cbbf.find_object_frame_and_bounding_box(cluster)
            # log("bounding box:", object_bounding_box)
    (object_pos, object_quat) = cf.mat_to_pos_and_quat(object_to_cluster_frame)
    angs = euler_from_quaternion(object_quat)
    log("angs:", angs)
            # position is half of height
    object_pos[2] = table_z + object_bounding_box[1][2] / 2. + DETECT_ERROR
    log("pos:", object_pos)
    log("table_z:", table_z)
    objects += [[object_pos, angs[2]]]
