#
# Copyright (c) 2011, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#  \author Charlie Kemp (Healthcare Robotics Lab, Georgia Tech.)


import numpy as np
import circular_buffer as cb

t = cb.CircularBuffer(5,())

def compare_lists(proven_list, test_list, slice_str):
    proven = proven_list
    test = test_list
    proven_command = "proven" + "[" + slice_str + "]"
    proven_out = eval(proven_command)
    test_command = "test" + "[" + slice_str + "]"
    test_out = eval(test_command)
    passed = True
    if len(proven_out) != len(test_out):
        passed = False
    for pi, ti in zip(proven_out, test_out):
        if (pi - ti) > 0.0000001:
            passed = False
    if not passed:
        print "***************************"
        print "******  FAILED TEST *******"
        print proven_command + " ="
        print proven_out
        print test_command + " ="
        print test_out
        print "***************************"
    else:
        print "---------------------------"
        print "------  passed test -------"
        print proven_command + " ="
        print proven_out
        print test_command + " ="
        print test_out
        print "---------------------------"


t.append(0)
t.append(1)
t.append(2)
t.append(3)
m = np.arange(0.0, 4.0, 1.0).tolist()
print 't =', t
print 'len(t) =', len(t)
print 'm =', m
print 'len(m) =', len(m)
print

compare_lists(m, t, ":")
compare_lists(m, t, "1:")
compare_lists(m, t, "1:2")
compare_lists(m, t, "2:2")
compare_lists(m, t, "::2")
compare_lists(m, t, "-1:")

print "t.append(4)"
t.append(4)
print "t.append(5)"
t.append(5)
print "t.append(6)"
t.append(6)
print "t.append(7)"
t.append(7)
print "m = range(3,8)"
m = np.arange(3.0, 8.0, 1.0).tolist()
print
print
print 't =', t
print 'len(t) =', len(t)
print 'm =', m
print 'len(m) =', len(m)
print

compare_lists(m, t, ":")
compare_lists(m, t, "1:")
compare_lists(m, t, "1:2")
compare_lists(m, t, "2:2")
compare_lists(m, t, "::2")
compare_lists(m, t, "-1:")

compare_lists(m, t, ":")
compare_lists(m, t, "0:")
compare_lists(m, t, "1:")
compare_lists(m, t, "2:")
compare_lists(m, t, "3:")
compare_lists(m, t, "4:")
compare_lists(m, t, "5:")

compare_lists(m, t, "-1:3")
compare_lists(m, t, "-3:3")
compare_lists(m, t, ":3")
compare_lists(m, t, ":10")
compare_lists(m, t, ":100")
compare_lists(m, t, "-3:")
compare_lists(m, t, "-10:")
compare_lists(m, t, "-100:")
compare_lists(m, t, ":-3")
compare_lists(m, t, ":-10")
compare_lists(m, t, ":-100")
compare_lists(m, t, "10:")
compare_lists(m, t, "100:")



