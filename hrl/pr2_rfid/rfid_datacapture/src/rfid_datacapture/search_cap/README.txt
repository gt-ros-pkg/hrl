# Search for tags
[rfid_datacapture] ./sm_explore_capture.py --fname woot --radius 4.0

# For each tag of interest:

   # Move to best tag location:
     rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped  --latch -f woot_tag__person.yaml

   # Bag capture of environment pre-servo
     [rfid_datacapture] ./sm_head_capture.py --fname woot_pre
     (optionally) get rviz screenshot(s).     

   # Servo approach
     [rfid_datacapture] ./sm_servo_capture.py --fname woot --tag 'person      '

   # Bag capture of environment post-servo
     [rfid_datacapture] ./sm_head_capture.py --fname woot_post
     (optionally) get rviz screenshot(s).

