#!/usr/bin/env python

import roslib
roslib.load_manifest('hrl_tactile_controller')
import rospy

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import pr2_skin_client as sc

print "Before rospy init"

rospy.init_node('TaxelArray_test')
print "After rospy init"
rospy.sleep(1.0)
print "After sleep"
tac1 = sc.PR2_SkinClient(['/skin/test_taxel_array_1', '/skin/test_taxel_array_2'])

# Unit tests for functions

ta1 = haptic_msgs.TaxelArray()
ta1.header.seq = 42
ta1.header.stamp = rospy.get_rostime()
ta1.header.frame_id = 'torso_lift_link'
ta1.sensor_type = 'bosch_dist'

ta1.centers_x = [1.0, 2.0, 3.0]
ta1.centers_y = [4.0, 5.0, 6.0]
ta1.centers_z = [7.0, 8.0, 9.0]

ta1.normals_x = [10.0, 11.0, 12.0]
ta1.normals_y = [13.0, 14.0, 15.0]
ta1.normals_z = [16.0, 17.0, 18.0]

ta1.values_x = [19.0, 0.0, 21.0]
ta1.values_y = [22.0, 0.0, 24.0]
ta1.values_z = [25.0, 0.0, 27.0]

ta1.link_names = ['shoulder_pan', 'shoulder_lift', 'upper_arm']

# Test: Trim taxel array
print "\nTrimming TaxelArray"
new_ta1 = tac1.trimTaxelArray(ta1, 0.5)
print new_ta1

# Test: Trim multiple taxel arrays
print "\n\nTrimming multiple TaxelArrays"
ta2 = haptic_msgs.TaxelArray()
ta2.header.seq = 43
ta2.header.stamp = rospy.get_rostime()
ta2.header.frame_id = 'torso_lift_link'
ta2.sensor_type = 'bosch_dist'

ta2.centers_x = [x * 2 for x in ta1.centers_x]
ta2.centers_y = [x * 2 for x in ta1.centers_y]
ta2.centers_z = [x * 2 for x in ta1.centers_z]

ta2.normals_x = [x * 2 for x in ta1.normals_x]
ta2.normals_y = [x * 2 for x in ta1.normals_y]
ta2.normals_z = [x * 2 for x in ta1.normals_z]

ta2.values_x = [x * 2 for x in ta1.values_x]
ta2.values_y = [(5 + x * 2) for x in ta1.values_y] # Non-zero value means that this taxel array won't be trimmed
ta2.values_z = [x * 2 for x in ta1.values_z]

ta2.link_names = ['shoulder_pan', 'shoulder_lift', 'wrist_flex']

tac1.skin_data = {'/skin/test_taxel_array_1': ta1, '/skin/test_taxel_array_2': ta2}
tac1.skin_data = tac1.trimSkinContacts()

print tac1.skin_data

# Test: getContactLocationsFromTaxelArray. Used for the Jacobian calc. 
# This data processing should probable be done by the kinematics so that the common data structure is simply a TaxelArray 
print "Testing: getContactLocationsFromTaxelArray(ta1)"
points_list = tac1.getContactLocationsFromTaxelArray(ta2)
print points_list

print "Testing: get ContactLocations() - on all known taxelarrays"
print tac1.getTaxelLocationAndJointList()

rospy.sleep(2.0)
tac2 = sc.PR2_SkinClient(['/skin/bosch/forearm_taxel_array', '/skin/bosch/upperarm_taxel_array'])

print "\n\nUsing real taxel data now..."
rospy.sleep(2.0)

print tac2.transformTaxelArray(tac2.skin_data['/skin/bosch/forearm_taxel_array'], 'torso_lift_link')

print "Spinning..."

# Test TF transforms.
# ... How? TODO: Use PR2.

while not rospy.is_shutdown():
  rospy.spin()
