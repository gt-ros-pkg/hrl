import roslib; roslib.load_manifest('trf_learn')
import hrl_lib.util as ut
import pylab as pb
from PIL import Image
import os.path as pt
import pdb
import numpy as np

def minmax(mat):
    return (np.min(mat[0,:]), np.max(mat[0,:]), 
            np.min(mat[1,:]), np.max(mat[1,:]))

def num_bins(points, bin_size):
    minx, maxx, miny, maxy = minmax(points)

    rangex = maxx-minx
    xbins = np.ceil(rangex/bin_size)

    rangey = maxy-miny
    ybins = np.ceil(rangey/bin_size)

    print 'XBINS', xbins, 'YBINS', ybins
    return xbins, ybins

##
#   ['neg_pred', 'image', 'pos_pred', 'tried', 'center']
def density_plot(pickle_file_name):
    BIN_SIZE = 20
    #PICKLE_FOLDER = 'pickle_files'

    data_dict = ut.load_pickle(pickle_file_name)
    #pdb.set_trace()
    orig_pickle_folder, _ = pt.split(pickle_file_name)
    folder_name, img_name = pt.split(data_dict['image'])
    nimg_path = pt.join(orig_pickle_folder, img_name)
    img_obj = Image.open(nimg_path)
    w, h = img_obj.size
    pb.imshow(img_obj, origin='lower')

    data_dict['neg_pred'][1,:] = h - data_dict['neg_pred'][1,:]  
    data_dict['pos_pred'][1,:] = h - data_dict['pos_pred'][1,:]

    all_pts = np.column_stack((data_dict['neg_pred'], data_dict['pos_pred']))
    Hall, xedges, yedges = np.histogram2d(all_pts[0,:].A1, all_pts[1,:].A1, 
                                          bins=num_bins(all_pts, BIN_SIZE))
    Hneg, xedges, yedges = np.histogram2d(data_dict['neg_pred'][0,:].A1, data_dict['neg_pred'][1,:].A1, 
                                          bins=[xedges, yedges])

    extent = [xedges[0], xedges[-1], yedges[-1], yedges[0]]
    Himage = (Hall-Hneg).T
    max_val, min_val = np.max(Himage), np.min(Himage)
    Hrgba = np.zeros((Himage.shape[0], Himage.shape[1], 4), dtype='uint8')
    Hrgba[:,:,0] = 0
    Hrgba[:,:,1] = 255 #Himage*80
    Hrgba[:,:,2] = 0
    Hrgba[:,:,3] = 255
    r,c = np.where(Himage == 0)
    Hrgba[r,c,3] = 0

    print 'max', max_val, 'min', min_val
    pb.imshow(Hrgba, extent=extent, interpolation='spline36', origin='upper', alpha = .7)
    #pdb.set_trace()
    #pb.plot(data_dict['neg_pred'][0,:].A1, data_dict['neg_pred'][1,:].A1, 'rx')
    #pb.plot(data_dict['pos_pred'][0,:].A1, data_dict['pos_pred'][1,:].A1, 'x')
    min_x, max_x, min_y, max_y = minmax(all_pts)
    pb.axis([max(min_x-100,0), min(max_x+100,w), max(min_y-100, 0), min(max_y+100, h)])
    #pb.axis([0, w, 0, h])
    name, extension = pt.splitext(img_name)
    pb.savefig(pt.join(orig_pickle_folder, name + '_plot.png'))
    #pb.show()

if __name__ == '__main__':
    import sys
    import optparse
    if len(sys.argv) > 1:
        for i in range(1, len(sys.argv)):
            density_plot(sys.argv[i])


