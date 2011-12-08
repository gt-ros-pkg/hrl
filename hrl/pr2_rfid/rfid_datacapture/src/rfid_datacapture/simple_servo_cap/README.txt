The data was (semi)autonomously captured using:
  ./sm_servo_capture_simple --yaml datacap_obj.yaml
  ./sm_servo_capture_simple --yaml datacap_vert.yaml

The pertinent Friis parameters were extracted from the bag files using:
  ./../process_bags_friis.py --yaml bag_process_obj.yaml
  ./../process_bags_friis.py --yaml bag_process_vert.yaml

View the trajectories in rviz:
  roslaunch rfid_nav map_only.launch
  ./process_traj_plots.py --yaml traj_plot_vert.yaml
  Take screenshot.



