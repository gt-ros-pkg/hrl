import roslib; roslib.load_manifest('hai_sandbox')
import hrl_lib.util as ut
import pylab as pb
import numpy as np
import pdb

def conf_to_percent(rec):
    conf = rec['mat']
    conf[0,:] = conf[0,:] / rec['neg']
    conf[1,:] = conf[1,:] / rec['pos']
    return conf[0,0], conf[1,1]

def plot_classifier_performance(fname, pname, plot_all):
    results = ut.load_pickle(fname)
    #pdb.set_trace()
    #results['train_set_statistics']    # [ {'conf', 'size'}, {}...]
    #results['current_scan_statistics'] # [ {'conf'} {}...]
    #results['perf_on_other_scans']     # [[{'name', 'conf'}, {}...] [{} {}...]...]
    #where conf is {'mat', 'neg', 'pos'}
    
    scores = {}
    for rlist in results['perf_on_other_scans']:
        for d in rlist:
            if scores.has_key(d['name']):
                scores[d['name']].append(conf_to_percent(d['conf']))
            else:
                scores[d['name']] = [conf_to_percent(d['conf'])]
    for k in scores.keys():
        scores[k] = zip(*scores[k])
    
    if results.has_key('train_set_statistics'):
        train_neg, train_pos = zip(*[conf_to_percent(d['conf']) for d in results['train_set_statistics']])
    else:
        train_neg = train_pos = None

    if results.has_key('current_scan_statistics'):
        pdb.set_trace()
        test_neg, test_pos = zip(*[conf_to_percent(d['conf']) for d in results['current_scan_statistics']])
    else:
        test_neg = test_pos = None

    n_iterations = np.array(range(len(results['train_set_statistics'])))
    
    #======================================================================
    pb.figure(1)
    if results.has_key('train_set_statistics'):
        pb.plot(n_iterations, train_neg, label='train ' + pname)
    if test_neg != None:
        pb.plot(n_iterations, test_neg, label='test ' + pname)
    if plot_all:
        for i, k in enumerate(scores.keys()):
            pb.plot(n_iterations, scores[k][0], '--', label=str(i))
    #if results.has_key('current_scan_statistics'):
    if results.has_key('converged_at_iter'):
        pb.plot([results['converged_at_iter'], results['converged_at_iter']], [0., 1.], 'r')

    pb.title('True negatives')
    pb.legend()
    
    #======================================================================
    pb.figure(2)
    if train_pos != None:
        pb.plot(n_iterations, train_pos, label='train ' + pname)
    if test_pos != None:
        pb.plot(n_iterations, test_pos, label='test ' + pname)
    #if results.has_key('current_scan_statistics'):

    print 'mapping from dataset to id'
    if plot_all:
        for i, k in enumerate(scores.keys()):
            pb.plot(n_iterations, scores[k][1], '--', label=str(i))
            print 'ID', i, 'dataset', k

    if results.has_key('converged_at_iter'):
        pb.plot([results['converged_at_iter'], results['converged_at_iter']], [0., 1.], 'r')

    pb.title('True positives')
    pb.legend()

def plot_features_perf(fnames, pnames):

    all_scores = {}
    dset_names = None
    for fname, pname in zip(fnames, pnames):
        results = ut.load_pickle(fname)
        train_neg, train_pos = zip(*[conf_to_percent(d['conf']) for d in results['train_set_statistics']])
        scores = {}
        for rlist in results['perf_on_other_scans']:
            for d in rlist:
                if scores.has_key(d['name']):
                    scores[d['name']].append(conf_to_percent(d['conf']))
                else:
                    scores[d['name']] = [conf_to_percent(d['conf'])]
        for k in scores.keys():
            scores[k] = zip(*scores[k])
        scores['train'] = [(train_neg), (train_pos)]
        all_scores[pname] = scores
        if dset_names == None:
            dset_names = scores.keys()


    neg_by_dset = {}
    for n in dset_names:
        posn = []
        for pname in pnames:
            posn.append(all_scores[pname][n][0][0])
        neg_by_dset[n] = posn

    pos_by_dset = {}
    for n in dset_names:
        posn = []
        for pname in pnames:
            posn.append(all_scores[pname][n][1][0])
        pos_by_dset[n] = posn

    ind = np.arange(len(pnames))
    width = 0.05


    fig = pb.figure(1)
    ax = fig.add_subplot(111)
    rects=[]
    for i, name in enumerate(dset_names):
        rect = ax.bar(ind+(width*i), pos_by_dset[name], width, color=tuple(np.random.rand(3).tolist()))
        rects.append(rect)
    ax.set_ylabel('accuracy')
    ax.set_title('True positives by dataset and features used')
    ax.set_xticks(ind+width)
    ax.set_xticklabels(tuple(pnames))

    fig = pb.figure(2)
    ax = fig.add_subplot(111)
    rects=[]
    for i, name in enumerate(dset_names):
        rect = ax.bar(ind+(width*i), neg_by_dset[name], width, color=tuple(np.random.rand(3).tolist()))
        rects.append(rect)
    ax.set_ylabel('accuracy')
    ax.set_title('True negatives by dataset and features used')
    ax.set_xticks(ind+width)
    ax.set_xticklabels(tuple(pnames))

if __name__ == '__main__':
    import sys
    import optparse
    p = optparse.OptionParser()
    p.add_option("-m", "--mode", action="store", type="string")
    p.add_option("-f", "--file", action="append", type="string")
    p.add_option('-n', '--name', action="append", type="string")
    opt, args = p.parse_args()

    if opt.mode == 'active':
        if len(opt.file) <= 1:
            plot_all = True
        else:
            plot_all = False

        for i in range(len(opt.file)):
            plot_classifier_performance(opt.file[i], opt.name[i], plot_all)
        pb.show()

    if opt.mode == 'features':
        plot_features_perf(opt.file, opt.name)
        pb.show()

#For comparing between different algorithms, don't need to plot performance on all scans just









