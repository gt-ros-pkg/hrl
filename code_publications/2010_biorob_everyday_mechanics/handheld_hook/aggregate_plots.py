
import commands
import os


def aggregate_plots(dir, name_list, extension_list):
    n = len(name_list)
    for i in range(n):
    #for nm in name_list:
        nm = name_list[i]
        ex = extension_list[i]
        pngs_list = commands.getoutput('find %s/0* -name "*%s.%s"'%(dir, nm, ex))
        pngs_list = pngs_list.splitlines()

        st = '%s/%s'%(dir, nm)
        os.system('rm -rf %s'%st) # remove previously existing directory.
        os.system('mkdir %s'%st)
        for p in pngs_list:
            sp = p.split('/')
            fname = str.join('_',sp[1:])
            print 'saving ', fname
            os.system('cp %s %s'%(p, st+'/'+fname))


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('-d', '--dir', action='store', default='',
                 type='string', dest='dir', help='directory with logged data')
    p.add_option('-a', '--agg', action='store_true', dest='agg',
                 help='aggregate plots from different trials into one folder for comparison')

    opt, args = p.parse_args()

    if opt.dir == '':
        raise RuntimeError('Need a directory to work with (-d or --dir)')

    if opt.agg:
        name_list = ['mechanism_trajectories_handhook']
        extension_list = ['pkl']
#        name_list = ['trajectory_check', 'force_components',
#                     'mechanism_trajectories_handhook', 'poses_dict']
#        extension_list = ['png', 'png', 'pkl', 'pkl']
        aggregate_plots(opt.dir, name_list, extension_list)


