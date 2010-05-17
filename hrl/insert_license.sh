#!/bin/bash
# usage: ./insert_license.sh <path to ros package>
# e.g. ./insert_license.sh code_publications/2010_icra_epc_pull

if [ "$1" = "" ]; then
    echo "Usage: ./insert_license.sh <path to ros package>"
    exit
fi

for f in `find $1 -name "*.py" -and ! -name "__init__.py"`
do
    sed -i '1i \\' $f
    sed -i '1i \# \\author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)' $f
    sed -i '1i \#' $f
    sed '1r LICENSE.txt'< $f > a.py
    mv a.py $f
done


