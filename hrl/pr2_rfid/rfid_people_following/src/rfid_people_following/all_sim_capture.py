import numpy as np,math
import sim_capture
import glob
import pickle as pkl
import time

class request():
    def __init__( self, x, y, ang ):
        self.x = x
        self.y = y
        self.z = 0
        self.ang = ang

def has_item( lst, item ):
    try:
        lst.index( item )
        return True
    except:
        return False
    

X,Y = np.meshgrid( range(0,11), range(0,7) )
xy = np.column_stack([ X.flatten(), Y.flatten() ]).tolist()
xy.remove( [7,3] )
xy.remove( [6,3] )

d_func = lambda lst: ( lst[0] - 7 )**2 + ( lst[1] - 3 )**2
cmp_func = lambda x,y : cmp(d_func(x),d_func(y))
xy.sort( cmp=cmp_func )

xya = []
for ang in [ math.radians(p) for p in [0.0, 90.0, 180.0, 270.0]]:
    for x,y in xy:
        xya.append( [x,y,ang] )

if not glob.glob( 'trajectory_caps/resutls.pkl' ):
    print 'Creating pickle file'
    d = { 'desired_pts': xya,
          'capture_pts': [],
          'captures': []
          }
    f = open( 'trajectory_caps/resutls.pkl', 'w' )
    pkl.dump( d, f )
    f.close()


f = open( 'trajectory_caps/resutls.pkl', 'r' )
d = pkl.load( f )
f.close()

remaining = [ x for x in d['desired_pts'] if not has_item( d['capture_pts'], x )]
print 'Remaining to capture: %d' % len( remaining )

sc = sim_capture.SimCapture()

for x,y,ang in remaining:
    fname = 'trajectory_caps/' + str(int(time.time())) + '_cap.pkl'
    sc.capture( request( x, y, ang ), fname )
    d[ 'capture_pts' ].append( [x,y,ang] )
    d[ 'captures' ].append( [x,y,ang,fname] )
    
    f = open( 'trajectory_caps/resutls.pkl', 'w' )
    pkl.dump( d, f )
    f.close()
    


