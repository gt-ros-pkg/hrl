import csv
import numpy as np
import pylab as pb
import matplotlib_util.util as mpu

def load_data():
    mk = csv.reader(open('Mechanism Kinematics.csv', 'U'))
    data = []
    for r in mk:
        data.append(r)

    return data

##
# Deal with repetition
def expand(data, keys):
    repetition_idx = np.where(np.array(keys) == 'repetition')[0]
    data[repetition_idx] = [int(e) for e in data[repetition_idx]]
    repeated_data = []

    for i in range(len(data)):
        repeated_data.append([])

    for current_instance in range(len(data[0])):
        if data[repetition_idx][current_instance] > 1:
            for rep in range(1, data[repetition_idx][current_instance]):
                for l, rl in zip(data, repeated_data):
                    rl.append(l[current_instance])
    for l, rl in zip(data, repeated_data):
        l += rl

    data.pop(repetition_idx)
    keys.pop(repetition_idx)
    return data

def test_expand():
    data = [['a', 'b', 'c'], ['A', 'B', 'C'], [1, 4, 10]]
    keys = ['letters', 'LETTERS', 'repetition']
    new_data = expand(data, keys)
    print new_data

def extract_keys(csv_data, keys):
    format = csv_data[1]
    indices = []
    llists = []
    for k in keys:
        indices.append(np.where(np.array(format) == k)[0])
        llists.append([])

    for i in range(2, len(csv_data)):
        if len(csv_data[i]) == 0:
            break
        if len(csv_data[i][indices[0]]) > 0:
            for lidx, data_idx in enumerate(indices):
                llists[lidx].append(csv_data[i][data_idx])
    return llists

def plot_radii(csv_data, color='#3366FF'):
    keys = ['radius', 'type', 'name', 'repetition']
    llists = expand(extract_keys(csv_data, keys), keys)
    rad_list = np.array([float(r) for r in llists[0]]) / 100.0
    types = llists[1]
    names = np.array(llists[2])
    all_types = set(types)
    print 'Radii types', all_types

    types_arr = np.array(types)
    # np.where(types_arr == 'C')
    cabinet_rad_list = rad_list[np.where(types_arr == 'C')[0]]
    others_rad_list = rad_list[np.where(types_arr != 'C')[0]]
    other_names = names[np.where(types_arr != 'C')[0]]
    print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
    print 'radii other names'
    for i, n in enumerate(other_names):
        print n, others_rad_list[i]
    print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'

    rad_lists = [rad_list, cabinet_rad_list, others_rad_list]
    titles = ['Radii of Rotary Mechanisms', 'Radii of Cabinets', 'Radii of Other Mechanisms']
    bin_width = 0.05
    max_radius = np.max(rad_list)
    print 'MIN RADIUS', np.min(rad_list)
    print 'MAX RADIUS', max_radius

    mpu.set_figure_size(5.,5.)
    for idx, radii in enumerate(rad_lists):
        f = pb.figure()
        f.set_facecolor('w')
        bins = np.arange(0.-bin_width/2., max_radius+2*bin_width, bin_width)
        hist, bin_edges = np.histogram(radii, bins)
        h = mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                           width=0.8*bin_width, xlabel='Radius (meters)',
                           ylabel='\# of mechanisms',
                           plot_title=titles[idx],
                           color=color, label='All')
        pb.xlim(.1, 1.)
        pb.ylim(0, 55)
        mpu.legend(display_mode = 'less_space', handlelength=1.)

    #-- different classes in different colors in the same histogram.
    f = pb.figure()
    f.set_facecolor('w')
    bins = np.arange(0.-bin_width/2., max_radius+2*bin_width, bin_width)
    hist, bin_edges = np.histogram(rad_lists[0], bins)
    h = mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                       width=0.8*bin_width, xlabel='Radius (meters)',
                       ylabel='\# of mechanisms',
                       plot_title=titles[1],
                       color='g', label='Cabinets')
    hist, bin_edges = np.histogram(rad_lists[2], bins)
    h = mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                       width=0.8*bin_width, xlabel='Radius (meters)',
                       ylabel='\# of mechanisms',
                       plot_title='Cabinets and Other Mechanisms',
                       color='y', label='Other')
    pb.xlim(.1, 1.)
    pb.ylim(0, 55)
    mpu.legend(display_mode = 'less_space', handlelength=1.)

    color_list = ['g', 'b', 'r']
    marker_list = ['s', '^', 'v']
    label_list = ['All', 'Cabinets', 'Other']
    scatter_size_list = [8, 5, 5]
    mpu.set_figure_size(5.,5.)
    mpu.figure()
    for idx, radii in enumerate(rad_lists):
        bins = np.arange(0.-bin_width/2., max_radius+2*bin_width, bin_width)
        hist, bin_edges = np.histogram(radii, bins)
        bin_midpoints = np.arange(0., max_radius+bin_width, bin_width)
        mpu.plot_yx(hist, bin_midpoints, color = color_list[idx],
                    alpha = 0.6, marker = marker_list[idx],
                    scatter_size = scatter_size_list[idx], xlabel='Radius (meters)',
                    ylabel='\# of mechanisms', label = label_list[idx])
    mpu.legend(display_mode = 'less_space')

def plot_opening(csv_data):
    keys = ['distance', 'type', 'repetition']
    llists = expand(extract_keys(csv_data, keys), keys)
    dists = np.array([float(f) for f in llists[0]])/100.
    types = llists[1]

    types_arr = np.array(types)
    drawer_dists = dists[np.where(types_arr == 'R')[0]]
    other_dists = dists[np.where(types_arr != 'R')[0]]

    print 'Opening distances types', set(types)
    bin_width = 0.02
    bins = np.arange(-bin_width/2., np.max(dists)+2*bin_width, bin_width)
    dists_list = [dists]#, drawer_dists, other_dists]
    titles = ['Opening Distances of Drawers']#, 'drawer', 'other']

#    print 'Total number of drawers:', len(dists)
    mpu.set_figure_size(5.,4.)
    for idx, d in enumerate(dists_list):
        f = pb.figure()
        f.set_facecolor('w')
        hist, bin_edges = np.histogram(d, bins)
        #import pdb; pdb.set_trace()
        mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                           width=0.8*bin_width, xlabel='Opening Distance (meters)',
                           ylabel='\# of Mechanisms',
                           plot_title=titles[idx], color='#3366FF')

#        pb.xticks(bins[np.where(bins > np.min(dists))[0][0]-2:-1])
#        pb.yticks(range(0, 26, 5))
#        pb.ylim(0, 25)

def handle_height_histogram(mean_height_list, plot_title='', color='#3366FF', max_height=2.5, bin_width=.1, ymax=35):
    bins = np.arange(0.-bin_width/2., max_height+2*bin_width, bin_width)
    hist, bin_edges = np.histogram(np.array(mean_height_list), bins)
    f = pb.figure()
    f.set_facecolor('w')
    f.subplots_adjust(bottom=.16, top=.86)
    mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                       width=bin_width*0.8, plot_title=plot_title,
                       xlabel='Height (meters)', ylabel='\# of mechanisms', color=color)
    pb.xlim(0, max_height)
    pb.ylim(0, ymax)

def handle_height_histogram_advait(mean_height_list, plot_title='',
                                   color='#3366FF', max_height=2.2, bin_width=.1, ymax=35,
                                   new_figure = True, label = '__no_legend__'):
    if new_figure:
        mpu.set_figure_size(5.,4.)
#        f = mpu.figure()
#        f.subplots_adjust(bottom=.25, top=.99, right=0.99, left=0.12)
    bins = np.arange(0.-bin_width/2., max_height+2*bin_width, bin_width)
    hist, bin_edges = np.histogram(np.array(mean_height_list), bins)
    h = mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                           width=bin_width*0.8, plot_title=plot_title,
                           xlabel='Height (meters)', ylabel='\# of mechanisms', color=color, label = label)
    pb.xlim(0, max_height)
    pb.ylim(0, ymax)
    return h

def plot_rotary_heights(csv_data):
    keys = ['radius', 'bottom', 'top', 'type', 'name', 'repetition']
    llists = expand(extract_keys(csv_data, keys), keys)
    types = llists[3]
    names = np.array(llists[4])
    print 'Handle bottom edge types', set(types)
    bottom_pts = np.array([float(f) for f in llists[1]])/100.

    top_pts = []
    for i,f in enumerate(llists[2]):
        if f == '':
            f = llists[1][i]
        top_pts.append(float(f)/100.)
    top_pts = np.array(top_pts)
    print 'total number of doors:', len(top_pts)
    mid_pts = (bottom_pts + top_pts)/2.

    types_arr = np.array(types)
    cabinet_mid_pts = mid_pts[np.where(types_arr == 'C')[0]]
    other_mid_pts = mid_pts[np.where(types_arr != 'C')[0]]
    other_names = names[np.where(types_arr != 'C')[0]]

    print 'other names', other_names
    # Hai, Advait apologizes for changing code without making a copy.
    # He didn't realise. He'll make a copy from an earlier revision in
    # subversion soon.
    ymax = 85
    handle_height_histogram_advait(mid_pts, plot_title='Handle Heights', ymax=ymax, label = 'All')
    mpu.legend(display_mode = 'less_space', handlelength=1.)
    handle_height_histogram_advait(mid_pts, plot_title='Handle Heights', ymax=ymax, color = 'g', label = 'Cabinets')
    handle_height_histogram_advait(other_mid_pts, plot_title='Handle Heights', ymax=ymax, color = 'y', new_figure = False, label = 'Other')
    mpu.legend(display_mode = 'less_space', handlelength=1.)

def plot_prismatic_heights(csv_data):
    keys = ['distance', 'bottom', 'top', 'type', 'name', 'home', 'repetition']
    llists = expand(extract_keys(csv_data, keys), keys)
    types = llists[3]
    names = np.array(llists[4])
    home = np.array(llists[5])
    print 'Handle bottom edge types', set(types)
    bottom_pts = np.array([float(f) for f in llists[1]])/100.
    top_pts = []
    for i,f in enumerate(llists[2]):
        if f == '':
            f = llists[1][i]
        top_pts.append(float(f)/100.)
    top_pts = np.array(top_pts)
    mid_pts = (bottom_pts + top_pts)/2.

    types_arr = np.array(types)
    max_height = np.max(bottom_pts)

    sort_order = np.argsort(bottom_pts)
    names = names[sort_order]
    bottom_pts = bottom_pts[sort_order]
    home = home[sort_order]
    for i, name in enumerate(names):
        print home[i], name, bottom_pts[i]


    handle_height_histogram(bottom_pts, plot_title='Heights of Handle Lower Edge (Drawers)', max_height = max_height)

    max_height = np.max(mid_pts)
    handle_height_histogram_advait(mid_pts, plot_title='Handle Heights', max_height = max_height+0.1, ymax=43)

#test_expand()
csv_data = load_data()
plot_prismatic_heights(csv_data)
#plot_radii(csv_data)
plot_rotary_heights(csv_data)
#plot_opening(csv_data)
pb.show()


