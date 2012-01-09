#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
args=("$@")
ccargs=""

# subjects for wiping:
#study_users=("1" "2" "4" "5" "6" "7" "8" "10")
#posh=0.02
#ptrim=0.50
# cheek multipliers:
#multipliers=( "0.6050" "0.4263" "0.2705" "0.3242" "0.2679" "0.8833" "0.3389" "0.5271" )
# nose multipliers:
#multipliers=( "0.2541" "0.5229" "0.1734" "0.4440" "0.2056" "0.6015" "0.5051" "0.5565" )
# chin multipliers:
#multipliers=( "0.5717" "0.5614" "0.2166" "0.5330" "0.3362" "0.8247" "0.4630" "0.9243" )

# subjects for shaving:
study_users=("1" "6" "8")
posh=0.02
ptrim=0.30
# shaving multipliers:
multipliers=( "0.3921" "0.4101" "0.6048" )

#multipliers=( "1" "1" "1" "1" "1" "1" "1" "1" )

num_users=${#study_users[@]}
set -x
for (( i=0; i<${num_users}; i++ ));
do
    rosrun hrl_phri_2011 extract_ell_face_function.sh ${study_users[$i]} $1 $2 $3 ${multipliers[$i]}
done
rosrun hrl_phri_2011 density_est_all.sh $1 $2 $3 ${study_users[*]}
#rosrun hrl_phri_2011 concat_ell_face_clouds.sh $1 $2 $3 ${study_users[*]}
#rosrun hrl_phri_2011 color_combo_face_cloud.sh $1 $2 $3 
