#!/bin/bash

# To see a breakdown...
./process_dir_estimate.py  --yaml dir_est_head_combined.yaml      | tee dir_est_head_combined_results.txt
./process_dir_estimate.py  --yaml dir_est_head_datacap.yaml       | tee dir_est_head_datacap.txt
./process_dir_estimate.py  --yaml dir_est_head_datacap2.yaml      | tee dir_est_head_datacap2.txt
./process_dir_estimate.py  --yaml dir_est_head_OrangeMedBot.yaml  | tee dir_est_head_OrangeMedBot.txt

./process_dir_estimate.py  --yaml dir_est_shoulder_left_combined.yaml | tee dir_est_shoulder_left_combined.txt
./process_dir_estimate.py  --yaml dir_est_shoulder_left_datacap.yaml  | tee dir_est_shoulder_left_datacap.txt
./process_dir_estimate.py  --yaml dir_est_shoulder_left_datacap2.yaml | tee dir_est_shoulder_left_datacap2.txt

./process_dir_estimate.py  --yaml dir_est_shoulder_right_combined.yaml | tee dir_est_shoulder_right_combined.txt
./process_dir_estimate.py  --yaml dir_est_shoulder_right_datacap.yaml  | tee dir_est_shoulder_right_datacap.txt
./process_dir_estimate.py  --yaml dir_est_shoulder_right_datacap2.yaml | tee dir_est_shoulder_right_datacap2.txt

mv est/*magherr_*.png est/condensed/
mv est/*magserr_*.png est/condensed/


# To generate A LOT OF PLOTS:

./process_dir_estimate.py  --yaml dir_est_head_combined.yaml --plot  | tee dir_est_head_combined_results.txt
./process_dir_estimate.py  --yaml dir_est_shoulder_left_combined.yaml --plot | tee dir_est_shoulder_left_combined.txt
./process_dir_estimate.py  --yaml dir_est_shoulder_right_combined.yaml --plot | tee dir_est_shoulder_right_combined.txt
