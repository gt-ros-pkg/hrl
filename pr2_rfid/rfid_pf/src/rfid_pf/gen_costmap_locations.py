import roslib
roslib.load_manifest( 'costmap_services' )
import rospy

import numpy as np, math
import costmap_services.python_client as costmap
from display_particles import DisplayParticles
import cPickle as pkl




if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--recalc', action='store_true', dest='recalc',
                 help='recalculate the costmap points?', default = False)
    p.add_option('--fname', action='store', type='string', dest='fname',
                 help='pkl file to use', default = False)
    p.add_option('--combine', action='store_true', dest='combine',
                 help='Combine results of multiple costmaps', default = False)
    opt, args = p.parse_args()
    

    rospy.init_node( 'tmp2243425' )

    # fname = 'costmap_costs.pkl'
    fname = opt.fname

    if opt.combine:
        # OK, this is just a hack.  I need a way to combine a masked map with a previously captured costmap.
        print 'YAY!'

        f = open( 'gen_costmap_mask.pkl', 'r' )
        mask = pkl.load( f )
        f.close()

        f = open( 'gen_costmap_aware_home.pkl', 'r' )
        obs = pkl.load( f )
        f.close()

        # fname = 'pf_costmap.pkl'

        ind_m = np.where( mask[:,2] < 127.0 )[0]
        ind_o = np.where( obs[:,2] < 127.0 )[0]
        ind = np.intersect1d( ind_m, ind_o )  # Locations that are good in both costmaps

        p_set = np.copy( obs )
        p_set[:,2] = np.zeros( obs.shape[0] )
        p_set[:,2][ind] = True

        f = open( fname, 'w' )
        pkl.dump( p_set, f )
        f.close()


        
    if not opt.fname:
        print 'fname required here on out.'
        exit()

    if opt.recalc:
        res = 0.05

        cs = costmap.CostmapServices()
        X,Y = np.meshgrid( np.arange( -5, 8, res ), np.arange( -5, 8, res ))
        xy = zip( X.flatten(), Y.flatten() )

        print 'Computing Map Costs...'
        mc = []
        for i,xyi in enumerate( xy ):
            if i % 100 == 0:
                print 'Still working ( %d of %d -- %3.2f%%)' % (i, len(xy), 100.0*i/len(xy))
            mc += [ cs.getMapCost( *xyi ) ]
        print 'Done.'

        p_set = np.column_stack([ np.array( X.flatten() ),
                                  np.array( Y.flatten() ),
                                  np.array( mc ) ])

        f = open( fname, 'w' )
        pkl.dump( p_set, f )
        f.close()


    dp = DisplayParticles()

    f = open( fname, 'r' )
    p_set = pkl.load( f )
    f.close()

    while not rospy.is_shutdown():
        print 'Displaying particles'
        dp.update( p_set )
        rospy.sleep( 0.3 )
