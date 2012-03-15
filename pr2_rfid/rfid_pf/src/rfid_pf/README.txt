# Make sure that all the process results (including the servo!) are run in rfid_datacapture
# Pick your sensor model in rfid_model.py  (yaml_fname)

# Compute the likelihood map for each tag read (stores 
#   (rfid_datacapture)/search_cap/search_aware_home/*_pf*.pkl)

time python -u ros_pf.py | tee log_ros_pf.txt

$ For pf stats: (generates Obj*)
# To compute all the screenshots  (first launch map and clock -- instead of gazebo)


rosbag play -r 4.0 --clock woot_150_8.bag
rfid_pf vis_node.launch 
python -u pf_stats.py --trial 9 | tee pf_results.txt


# For default stats:
python -u stats_best_uniform.py | tee stats_best_uniform.txt


# For relative stats  (rfid_datacapture/search)
python -u rel_stats.py --trial 9 | tee rel_stats.txt

# Build a pkl with _all_ the results in one structure: (generates summarize*.pkl)
python summarize_results.py --recalc
python summarize_results.py | tee summarize_results.txt


# All results in current (rfid_pf) directory.  Move into own dir to run next set

(optional) mkdir curr_results

(optional) mv log_ros_pf.txt curr_results/
(optional) mv pf_results.txt curr_results/
(optional) mv stats_best_uniform.txt curr_results/
(optional) mv summarize*.pkl curr_results/
(optional) mv summarize_results.txt curr_results/

(optional) mkdir curr_results/pf_results/
(optional) mv Obj* curr_results/pf_results/

(optional) mkdir curr_results/pf_searchservo/
(optional) mv (rfid_datacapture)/search_cap/search_aware_home/*_pf*.pkl curr_results/pf_searchservo/



# MISC
# To view a single instace of pf results
python display_particles.py --trial 4 --obj 5 --servo


