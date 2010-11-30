#
# Copyright (c) 2010, Georgia Tech Research Corporation
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

import numpy as np, math
import roslib; roslib.load_manifest('hrl_lib')
import hrl_lib.transforms as tr

    

def residualXform( residuals ):
    '''
    residuals are np.array([ Rz2, Rx, Rz1, dx, dy, dz ])
    returns rotResid, dispResid
    '''
    rotResid = tr.Rz( residuals[0] ) * tr.Rx( residuals[1] ) * tr.Rz( residuals[2] )
    dispResid = np.matrix([ residuals[3], residuals[4], residuals[5] ]).T
    return rotResid, dispResid    

def camTlaser( res = np.zeros(7) ):
    # @ Duke, res = np.array([0.8, 0.9, -1.7, 3.1, 0.061, 0.032, -0.035 ])
    rot = tr.Ry( math.radians( 0.0 + res[0] )) * tr.Rz( math.radians( 0.0 + res[1] )) * tr.Rx( math.radians( -90.0 + res[2] )) * tr.Rz( math.radians( -90.0 + res[3]))
    disp = np.matrix([ res[4], res[5], res[6] ]).T + np.matrix([ 0.0, 0.0, 0.0 ]).T
    return tr.composeHomogeneousTransform(rot, disp)

def rollTtool_pointer( residuals = np.zeros(6) ):
    rotResid, dispResid = residualXform( residuals )
    rot = rotResid * tr.Rz( math.radians( -10.0 ))
    disp = dispResid + np.matrix([ 0.008, 0.0, 0.0 ]).T
    return tr.composeHomogeneousTransform(rot, disp)

def rollTtool_MA( residuals = np.zeros(6) ):
    rotResid, dispResid = residualXform( residuals )
    rot = rotResid * tr.Ry( math.radians( -90.0 ))
    disp = dispResid + np.matrix([ 0.0476, 0.0, 0.0 ]).T
    return tr.composeHomogeneousTransform(rot, disp)


def panTroll(rollAng, residuals = np.zeros(6) ):
    rotResid, dispResid = residualXform( residuals )
    rot = rotResid * tr.Rx( -1.0 * rollAng )
    disp = dispResid + np.matrix([0.02021, 0.0, 0.04236 ]).T
    return tr.composeHomogeneousTransform(rot, disp)

def tiltTpan(panAng, residuals = np.zeros(6) ):
    rotResid, dispResid = residualXform( residuals )
    rot = rotResid * tr.Rz( -1.0 * panAng )
    disp = dispResid + np.matrix([ 0.07124, 0.0, 0.02243 ]).T
    return tr.composeHomogeneousTransform(rot, disp)

def laserTtilt(tiltAng, residuals = np.zeros(6) ):
    rotResid, dispResid = residualXform( residuals )
    rot = rotResid * tr.Ry( +1.0 * tiltAng )
    disp = dispResid + np.matrix([ 0.03354, 0.0, 0.23669 ]).T
    return tr.composeHomogeneousTransform(rot, disp)

def laserTtool_pointer(rollAng, panAng, tiltAng, residuals = np.zeros([4,6])):
    '''
    This is specifically for the off-axis laser pointer!  Tool coordinate frame will change for each tool.
    Here, residuals are 4x6 array where:
       res[0] = rollTtool
       res[1] = panTroll
       res[2] = tiltTpan
       res[3] = laserTtilt
    '''
    res = residuals
    return laserTtilt(tiltAng, res[3] ) * tiltTpan(panAng, res[2] ) * panTroll(rollAng, res[1] ) * rollTtool_pointer(res[0])

def tool_pointerTlaser(rollAng, panAng, tiltAng, residuals = np.zeros([4,6])):
    return tr.invertHomogeneousTransform( laserTtool_pointer(rollAng, panAng, tiltAng, residuals) )


def laserTtool_MA(rollAng, panAng, tiltAng, residuals = np.zeros([4,6])):
    '''
    This is specifically for the multi-antenna (MA) tool attachment!  Tool coordinate frame will change for each tool.
    Here, residuals are 4x6 array where:
       res[0] = rollTtool
       res[1] = panTroll
       res[2] = tiltTpan
       res[3] = laserTtilt
    '''
    res = residuals
    return laserTtilt(tiltAng, res[3] ) * tiltTpan(panAng, res[2] ) * panTroll(rollAng, res[1] ) * rollTtool_MA(res[0])

def tool_MATlaser(rollAng, panAng, tiltAng, residuals = np.zeros([4,6])):
    return tr.invertHomogeneousTransform( laserTtool_MA(rollAng, panAng, tiltAng, residuals) )

