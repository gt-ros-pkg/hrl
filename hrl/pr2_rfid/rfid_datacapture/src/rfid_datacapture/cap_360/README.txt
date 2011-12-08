The data was (semi)autonomously captured using:
  ./sm_cap_360 --yaml datacap_head.yaml

The pertinent Friis parameters were extracted from the bag files using:
  ./process_bags_friis.py --yaml cap_360/bag_process_head.yaml

Get the Intercept for the Friis model:

  ./../process_friis_plots.py --yaml friis_plot_combined.yaml 

  # Modify xval / yval for other friis_plot_*.yaml files.  Then:
  ./gen_plots.sh  # This will generate the intermediate pickles
  ./gen_plots.sh  # This will actually perform the plots the 2nd pass.

  # THIS WILL TAKE A WHILE!
  ./heading_est.sh



**NOTE: The shoulder_datacap files are copied from the old captures in
data_robot1/rfid_data/pan_captures/datacap
