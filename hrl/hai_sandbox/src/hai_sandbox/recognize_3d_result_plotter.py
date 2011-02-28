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
    pdb.set_trace()
    results['train_set_statistics']    # [ {'conf', 'size'}, {}...]
    results['current_scan_statistics'] # [ {'conf'} {}...]
    results['perf_on_other_scans']     # [[{'name', 'conf'}, {}...] [{} {}...]...]
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
    
    train_neg, train_pos = zip(*[conf_to_percent(d['conf']) for d in results['train_set_statistics']])
    test_neg, test_pos = zip(*[conf_to_percent(d['conf']) for d in results['current_scan_statistics']])
    n_iterations = np.array(range(len(results['train_set_statistics'])))
    
    pb.figure(1)
    pb.plot(n_iterations, train_neg, label='train ' + pname)
    pb.plot(n_iterations, test_neg, label='test ' + pname)
    if plot_all:
        for i, k in enumerate(scores.keys()):
            pb.plot(n_iterations, scores[k][0], '--', label=str(i))
    pb.title('True negatives')
    pb.legend()
    
    pb.figure(2)
    pb.plot(n_iterations, train_pos, label='train ' + pname)
    pb.plot(n_iterations, test_pos, label='test ' + pname)
    print 'mapping from dataset to id'
    if plot_all:
        for i, k in enumerate(scores.keys()):
            pb.plot(n_iterations, scores[k][1], '--', label=str(i))
            print 'ID', i, 'dataset', k
    pb.title('True positives')
    pb.legend()


if __name__ == '__main__':
    import sys
    import optparse
    p = optparse.OptionParser()
    p.add_option("-f", "--file", action="append", type="string")
    p.add_option('-n', '--name', action="append", type="string")
    opt, args = p.parse_args()

    if len(opt.file) <= 1:
        plot_all = True
    else:
        plot_all = False

    for i in range(len(opt.file)):
        plot_classifier_performance(opt.file[i], opt.name[i], plot_all)

    pb.show()

#For comparing between different algorithms, don't need to plot performance on all scans just









