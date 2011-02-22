import roslib; roslib.load_manifest('hai_sandbox')
import hrl_lib.util as ut
import sys
import pylab as pb
import numpy as np
import pdb

def conf_to_percent(rec):
    conf = rec['mat']
    conf[0,:] = conf[0,:] / rec['neg']
    conf[1,:] = conf[1,:] / rec['pos']
    return conf[0,0], conf[1,1]

results = ut.load_pickle(sys.argv[1])
results['train_set_statistics']    # [ {'conf', 'size'}, {}...]
results['current_scan_statistics'] # [ {'conf'} {}...]
results['perf_on_other_scans']     # [[{'name', 'conf'}, {}...] [{} {}...]...]
#where conf is {'mat', 'neg', 'pos'}

#what to plot?
n_iterations = np.array(range(len(results['train_set_statistics'])))

train_neg, train_pos = zip(*[conf_to_percent(d['conf']) for d in results['train_set_statistics']])
test_neg, test_pos = zip(*[conf_to_percent(d['conf']) for d in results['current_scan_statistics']])
scores = {}
for rlist in results['perf_on_other_scans']:
    for d in rlist:
        if scores.has_key(d['name']):
            scores[d['name']].append(conf_to_percent(d['conf']))
        else:
            scores[d['name']] = [conf_to_percent(d['conf'])]
for k in scores.keys():
    scores[k] = zip(*scores[k])


#scores[n] = zip(*[conf_to_percent(d['conf']) for d in rlist])
#pdb.set_trace()

#performance vs iterations, for test set for scan being trained on and
#                               one line for each dataset, 

pb.figure(1)
pb.plot(n_iterations, train_neg, label='train')
pb.plot(n_iterations, test_neg, label='test')
for i, k in enumerate(scores.keys()):
    pb.plot(n_iterations, scores[k][0], '--', label=str(i))
pb.title('True negatives')
pb.legend()

pb.figure(2)
pb.plot(n_iterations, train_pos, label='train')
pb.plot(n_iterations, test_pos, label='test')
print 'mapping from dataset to id'
for i, k in enumerate(scores.keys()):
    pb.plot(n_iterations, scores[k][1], '--', label=str(i))
    print 'ID', i, 'dataset', k
pb.title('True positives')
pb.legend()

pb.show()

#
