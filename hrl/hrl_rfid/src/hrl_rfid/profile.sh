#!/bin/bash

python -m cProfile -o $2.profile $1.py
python gprof2dot.py -f pstats $2.profile | dot -Tpng -o $2.profile.png
eog $2.profile.png &
runsnake $2.profile &
