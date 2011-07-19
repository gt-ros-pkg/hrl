#!/usr/bin/env python

import dynamics_utils as dynutils


(name, time, pos, vel, acc) = dynutils.load_motion_ascii("/u/dhennes/Desktop/reaching.txt")
dynutils.save_motion("../motions/reaching.pkl", name, time, pos, vel, acc)
