The data was (semi)autonomously captured using:
  ./sm_rad_cap --yaml datacap.yaml

The pertinent Friis parameters were extracted from the bag files using:
  ./process_bags_friis.py --yaml rad_pattern_cap/bag_process_table.yaml

Get the Intercept for the Friis model:
  # Skip this:  ./../process_friis_plots.py --yaml friis_plot_combined.yaml 

  # Modify xval / yval for other friis_plot_*.yaml files.  Then:
  ./gen_plots.sh
  ./gen_plots.sh




