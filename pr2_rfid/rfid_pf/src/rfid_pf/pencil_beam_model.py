import numpy as np, math
import cPickle as pkl

EXT = 10.0
DIVS = 200  # Keep this even


d = {}
d['xedges'] = np.linspace( -1 * EXT, EXT, DIVS )
d['yedges'] = np.linspace( -1 * EXT, EXT, DIVS )
d['stddev_model'] = np.ma.array(np.ones( (DIVS,DIVS) ))
d['detect_model'] = np.ma.array(np.ones( (DIVS,DIVS) ))
d['rssi_model'] = np.ma.array( np.ones( (DIVS,DIVS) ) * 55 )

t = d['rssi_model'][DIVS/2-1,DIVS/2-1:]
d['rssi_model'][DIVS/2-1,DIVS/2-1:] = np.array([-35.0 / (DIVS / 2) * i + 100
                                                for i in xrange( len( t ))])

# Need to build a fake yaml config file for this fake beam model.

f = open( 'pencil_beam_MODEL.pkl', 'w' )
pkl.dump( d, f )
f.close()



