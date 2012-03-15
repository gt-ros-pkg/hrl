#
# Copyright (c) 2009, Georgia Tech Research Corporation
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

#  \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)


import pylab as pl
import math, numpy as np
from matplotlib.patches import Ellipse


## calls pylab.figure() and returns the figure
# other params can be added if required.
# @param dpi - changing this can change the size of the font.
def figure(fig_num=None, dpi=None):
    #mpu.pl.figure(num=None, figsize=(4, 4), dpi=100, facecolor='w', edgecolor='k')
    return pl.figure(fig_num, dpi=dpi, facecolor='w')

## calls pylab.savefig()
def savefig(fname):
    pl.savefig(fname)

## legend drawing helper
# @param loc - 'best', 'upper left' ...
# @param display_mode - 'normal', 'less_space'
def legend(loc='best',display_mode='normal', draw_frame = True,
        handlelength=0.003):
    params = {'legend.fontsize': 10}
    pl.rcParams.update(params)
    if display_mode == 'normal':
        leg = pl.legend(loc=loc)
        leg.draw_frame(draw_frame)
    elif display_mode == 'less_space':
        leg = pl.legend(loc=loc,handletextpad=0.7,handlelength=handlelength,labelspacing=0.01,
                        markerscale=0.5)
        leg.draw_frame(draw_frame)

##
# generate a random color.
# @return string of the form #xxxxxx
def random_color():
    r = '%02X'%np.random.randint(0, 255)
    g = '%02X'%np.random.randint(0, 255)
    b = '%02X'%np.random.randint(0, 255)
    c = '#' + r + g + b
    return c

##
# @param figure width in cm
# @param figure height in cm
def set_figure_size(fig_width, fig_height):
    inches_per_cm = 1./2.54
    fig_width = fig_width * inches_per_cm     # width in inches
    fig_height = fig_height * inches_per_cm   # height in inches
    fig_size =  [fig_width, fig_height]
    params = {'backend': 'WXAgg',
              'axes.labelsize': 12,
              'text.fontsize': 12,
              'legend.fontsize': 12,
              'xtick.labelsize': 10,
              'ytick.labelsize': 10,
              'text.usetex': True,
              'figure.figsize': fig_size}
    pl.rcParams.update(params)

## typical usage: ax = pp.gca(); mpu.flip_x_axis(ax)
def flip_x_axis(ax):
    ax.set_xlim(ax.get_xlim()[::-1])

## typical usage: ax = pp.gca(); mpu.flip_y_axis(ax)
def flip_y_axis(ax):
    ax.set_ylim(ax.get_ylim()[::-1])


## plot an ellipse
# @param mn - center of ellipe. (2x1 np matrix)
# @param P - covariance matrix.
def plot_ellipse_cov(pos, P, edge_color, face_color='w', alpha=1.):
    U, s , Vh = np.linalg.svd(P)
    ang = math.atan2(U[1,0],U[0,0])
    w1 = 2.0*math.sqrt(s[0])
    w2 = 2.0*math.sqrt(s[1])
    return plot_ellipse(pos, ang, w1, w2, edge_color, face_color,
                        alpha)

## plot an ellipse
# @param mn - center of ellipe. (2x1 np matrix)
# @param angle of the ellipse (RADIANS)
def plot_ellipse(pos, angle, w1, w2, edge_color, face_color='w',
                 alpha=1.):
    orient = math.degrees(angle)
    e = Ellipse(xy=pos, width=w1, height=w2, angle=orient,
                facecolor=face_color, edgecolor=edge_color)
    e.set_alpha(alpha)
    ax = pl.gca()
    ax.add_patch(e)
    return e

## plot circle (or an arc counterclockwise starting from the y-axis)
# @param cx - x coord of center of circle.
# @param cy - y coord of center of circle.
# @param rad - radius of the circle
# @param start_angle -  starting angle for the arcin RADIANS. (0 is y axis)
# @param end_angle -  ending angle for the arcin RADIANS. (0 is y axis)
# @param step - step size for the linear segments.
# @param color - color of the circle.
# @param label - legend label.
#
# circle plotted as bunch of linear segments. back to LOGO days.
def plot_circle(cx, cy, rad, start_angle, end_angle, step=math.radians(2),
                color='k', label='', alpha=1.0):
    if start_angle>end_angle:
        step = -step

    n_step = int((end_angle-start_angle)/step+0.5)
    x,y=[],[]
    for i in range(n_step):
        x.append(cx-rad*math.sin(start_angle+i*step))
        y.append(cy+rad*math.cos(start_angle+i*step))
    x.append(cx-rad*math.sin(end_angle))
    y.append(cy+rad*math.cos(end_angle))

    pl.axis('equal')
    return pl.plot(x,y,c=color,label=label,linewidth=2, alpha=alpha)

## plot radii at regular angular intervals.
# @param cx - x coord of center of circle.
# @param cy - y coord of center of circle.
# @param rad - radius of the circle
# @param start_angle -  starting angle for the arcin RADIANS. (0 is y axis)
# @param end_angle -  ending angle for the arcin RADIANS. (0 is y axis)
# @param interval - angular intervals for the radii
# @param color - color of the circle.
# @param label - legend label.
def plot_radii(cx, cy, rad, start_angle, end_angle, interval=math.radians(15),
               color='k', label='', alpha=1.0):
    if start_angle < 0.:
        start_angle = 2*math.pi+start_angle
    if end_angle < 0.:
        end_angle = 2*math.pi+end_angle
    if start_angle>end_angle:
        interval = -interval

    n_step = int((end_angle-start_angle)/interval+0.5)
    x,y=[],[]
    for i in range(n_step):
        x.append(cx)
        y.append(cy)
        x.append(cx-rad*math.sin(start_angle+i*interval))
        y.append(cy+rad*(math.cos(start_angle+i*interval)))
    x.append(cx)
    y.append(cy)
    x.append(cx-rad*math.sin(end_angle))
    y.append(cy+rad*math.cos(end_angle))

    pl.plot(x,y,c=color,label=label,linewidth=1,alpha=alpha)
    pl.axis('equal')

## plot a quiver
# @param y - list/1D array
# @param x - list/1D array
# @param v - 2xN or 3xN np matrix of vectors.
# @param width - thickness of the arrows.
# @param scale - decreasing the scale increases the length of the arrows.
def plot_quiver_yxv(y, x, v, color='k', width=0.005, scale=100.0):
    pl.quiver(x, y, v[0,:].A1,v[1,:].A1, width=width, color=color, scale=scale)


## plot a histogram.
# @param left - left edge of the bin. (array-like)
# @param height - height of each bin (array-like)
def plot_histogram(left, height, width=0.8, label='',
                   align='center', color='b', alpha=1.):
    pb_obj = pl.bar(left, height, width=width, align=align,
                    color=color, alpha=alpha, label=label, linewidth=0)
    return pb_obj


# 3d in matplotlib is buggy. Using mayavi instead.
##---------- 3D plotting ---------------
#
###
## @param scatter_size - as of Aug 17, 2010 and matplotlib version 0.99.1.1, scatter_size does not work
#def plot_xyz(x, y, z, color='b', scatter_size=20, fig=None, xlabel='',
#             ylabel='', zlabel=''):
#    if fig == None:
#        fig = pl.figure()
#    ax = fig.gca(projection='3d')
##    ax = Axes3D(fig)
##    ax.plot(x, y, z, c=color)
#    ax.scatter(x, y, z, s=scatter_size)
#
##    ax.set_xlabel(xlabel)
##    ax.set_ylabel(ylabel)
##    ax.set_zlabel(zlabel)
##    ax.legend()





