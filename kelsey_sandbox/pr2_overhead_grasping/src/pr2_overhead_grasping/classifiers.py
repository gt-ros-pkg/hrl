#! /usr/bin/python

import numpy as np
import random
import sys
import matplotlib.pyplot as plt
import multiprocessing as mp
import cPickle as pickle

import roslib; roslib.load_manifest('pr2_overhead_grasping')
import rospy

from helpers import log, err, node_name, FileOperations

import ml_lib.random_forest as rf
import ml_lib.dataset as ds
import ml_lib.dimreduce as ldds
                                            
class AbstractClassifier(object):

    def __init__(self):
        # self.load_parameters()
        self.fos = FileOperations()

    # def load_parameters(self):
    #     self.RTREE_CLASSIFIER = rospy.get_param("/overhead_grasping/rtree_classifier")

    def train(self, compiled_dataset):
        err("Classifier not implemented!")

    def predict(self, instance):
        err("Classifier not implemented!")
        return 0.

    def build(self, compiled_dataset, filename):
        err("Classifier not implemented!")

    def load(self, filename):
        err("Classifier not implemented!")

    def cross_valid(self, compilied_dataset, num_folds = 10, seed = 1):

        c_mat_list, traj_dicts = self._fold_predictions(compiled_dataset, num_folds, seed)
        confusion_mat = sum(c_mat_list)

        confusion_matrix_stats(confusion_mat)
        log("-"*60)
        log("Analysis")
        first_coll_diffs = []
        for traj_dict in traj_dicts:
            traj_list = [traj_dict[k] for k in traj_dict]
            first_coll_diffs.extend(self.analyze_testing_results(traj_list))
        log("min: %1.3f, max: %1.3f, median: %1.3f, mean: %1.3f, std: %1.3f" % (np.min(first_coll_diffs), np.max(first_coll_diffs), np.median(first_coll_diffs), np.mean(first_coll_diffs), np.std(first_coll_diffs)))
        log("[" + ", ".join(["%1.3f" % v for v in first_coll_diffs]) + "]")
        # self.plot_testing_results(traj_lists[0])

    def _fold_predictions(self, compiled_dataset, num_folds, seed):
        cdata = compiled_dataset
        times_list = cdata["training_times_list"]
        colls_list = cdata["training_colls_list"]

        train = cdata["training_data"]
        responses = cdata["training_labels"]
        random.seed(seed)
        labels = {}
        for i, l in enumerate(cdata["trajectory_labels"]):
            if l[0] not in labels:
                labels[l[0]] = {l[1] : [i]}
            elif l[1] not in labels[l[0]]:
                labels[l[0]][l[1]] = [i]
            else:
                labels[l[0]][l[1]].append(i)

        train_test_combos = [{"train" : [], "test" : [], "test_traj_labels" : []} for i in range(num_folds)]
        for dir in labels:
            fns = labels[dir].keys()
            split_ind = int(round(float(len(fns)) / num_folds))
            fns.sort()
            random.shuffle(fns)
            last_f_ind = 0
            for nf in range(num_folds):
                cur_f_ind = int(round(float(len(fns) * (nf + 1)) / num_folds))
                train_inds, test_inds = [], []
                test_range = range(last_f_ind, cur_f_ind)
                for f_ind, fn in enumerate(fns):
                    if f_ind not in test_range:
                        train_inds.extend(labels[dir][fns[f_ind]])
                    else:
                        test_inds.extend(labels[dir][fns[f_ind]])
                train_test_combos[nf]["train"].extend(train_inds)
                train_test_combos[nf]["test"].extend(test_inds)
                train_test_combos[nf]["test_traj_labels"].extend([dir for i in test_inds])

                last_f_ind = cur_f_ind
        
        params = []
        for fold_i in range(num_folds):
            log("Fold:", fold_i + 1)
            print train.shape
            train_fold = train[:, train_test_combos[fold_i]["train"]]
            train_fold_resp = responses[0, train_test_combos[fold_i]["train"]]
            train_fold_times = times_list[train_test_combos[fold_i]["train"]]
            train_fold_coll_times = colls_list[train_test_combos[fold_i]["train"]]
            test_fold = train[:, train_test_combos[fold_i]["test"]]
            test_fold_resp = responses[0, train_test_combos[fold_i]["test"]]
            test_fold_times = times_list[train_test_combos[fold_i]["test"]]
            test_fold_coll_times = colls_list[train_test_combos[fold_i]["test"]]
            test_fold_traj_labels = train_test_combos[fold_i]["test_traj_labels"]
            new_comp_dataset = {"training_data" : train_fold,
                                "training_labels" : train_fold_resp,
                                "training_times_list" : train_fold_times, 
                                "training_colls_list" : train_fold_coll_times, 
                                "testing_data" : test_fold,
                                "testing_labels" : test_fold_resp,
                                "testing_times_list" : test_fold_times, 
                                "testing_colls_list" : test_fold_coll_times,
                                "trajectory_labels" : test_fold_traj_labels}
            params.append((self, new_comp_dataset,
                           train_test_combos[fold_i]["test"]))

        pool = mp.Pool()
        results = pool.map(run_fold_process, params)
        c_mat_list, traj_dicts = zip(*results)
        return c_mat_list, traj_dicts

    def analyze_testing_results(self, traj_list):
        t_labels, pred_label_lists, diff_lists = [], [], []
        for i, cur_traj in enumerate(traj_list):
            times, labels, pred_labels, coll_times, indicies = zip(*cur_traj)
            traj_label = 0.
            coll_time = coll_times[0]
            for l in labels:
                if l != 0.:
                    traj_label = l
                    break
            pred_label_list, diff_list = [None] * 10, [None] * 10
            for i, l in enumerate(pred_labels):
                if l != 0.:
                    for pll_i, _ in enumerate(pred_label_list):
                        if pred_label_list[pll_i] is None:
                            pred_label_list[pll_i] = l
                            diff_list[pll_i] = times[i] - coll_time
                            break
            # log(traj_label, pred_label_list, diff_list)
            t_labels.append(traj_label)
            pred_label_lists.append(pred_label_list)
            diff_lists.append(diff_list)

        first_coll_diffs = list(zip(*diff_lists)[0])
        for n in range(5):
            log("Num in a row: %d" % (n+1))
            first_preds = list(zip(*pred_label_lists)[n])
            for i, v in enumerate(first_coll_diffs):
                if v is None:
                    first_coll_diffs[i] = 0.0
                if first_preds[i] is None:
                    first_preds[i] = 0.
            confus_mat = np.zeros((3,3))
            for i, v in enumerate(t_labels):
                confus_mat[int(t_labels[i]), int(first_preds[i])] += 1
            log(confus_mat)
        log("min: %1.3f, max: %1.3f, median: %1.3f, mean: %1.3f, std: %1.3f" % (np.min(first_coll_diffs), np.max(first_coll_diffs), np.median(first_coll_diffs), np.mean(first_coll_diffs), np.std(first_coll_diffs)))
        log("[" + ", ".join(["%1.3f" % v for v in first_coll_diffs]) + "]")
        return first_coll_diffs

    ##
    # Takes the trajectory list from eval_predicts and plots it.
    # each trajectory is stacked in a horizontal lines.  Vertical placement
    # indicates predicted result.  Color indicates actual result.
    # Collision trajectories are offset so that the colllision occurs at t = 0
    # Empty grasps are offset so that the end of the trajectory is at t = 0
    # Green = No collision, Red = External collision, Blue = Table collision
    def plot_testing_results(self, traj_list):

        plt.figure(1)
        plt.fill_between([-20.0, 20.0], -1., 1., facecolor='green', alpha=0.3)
        plt.fill_between([-20.0, 20.0], 1., 2., facecolor='red', alpha=0.3)
        plt.fill_between([-20.0, 20.0], 2., 3., facecolor='blue', alpha=0.3)

        fp_diffs = []
        for i, cur_traj in enumerate(traj_list):
            (pred_norm, pred_norm_t, pred_ext, 
             pred_ext_t, pred_tab, pred_tab_t) = [], [], [], [], [], []
            for pt in cur_traj:
                if pt[3] <= 0.:
                    pt[3] = cur_traj[-1][0]
                if pt[1] == 0. and pt[2] != 0.:
                    fp_diffs.append(pt[0] - pt[3])
                if pt[1] == 0:
                    pred_norm.append(pt[2] + i * 0.018)
                    pred_norm_t.append(pt[0] - pt[3])
                elif pt[1] == 1:
                    pred_ext.append(pt[2] + i * 0.018)
                    pred_ext_t.append(pt[0] - pt[3])
                elif pt[1] == 2:
                    pred_tab.append(pt[2] + i * 0.018)
                    pred_tab_t.append(pt[0] - pt[3])
            if len(pred_norm) > 0:
                plt.scatter(pred_norm_t, pred_norm, c='green', marker='o', s = 30)
            if len(pred_ext) > 0:
                plt.scatter(pred_ext_t, pred_ext, c='red', marker='o', s = 30)
            if len(pred_tab) > 0:
                plt.scatter(pred_tab_t, pred_tab, c='blue', marker='o', s = 30)
        plt.axvline(0.0, color='black', linestyle = '-')
        plt.axhline(1.0, color='black', linestyle = '-')
        plt.axhline(2.0, color='black', linestyle = '-')
        plt.axis([-4.8, 1.2, -0.1, 2.8])
        plt.show()

def confusion_matrix_stats(confusion_mat):
    c_mat = np.matrix(confusion_mat.astype(float))
    assert c_mat.shape[0] == c_mat.shape[1]
    N = c_mat.shape[0]
    print "-"*60
    print "Confusion Matrix Statistics\n"
    print
    print "            Predicted        "
    print     "              ",
    for c in range(N):
        print "%4d" % c,
    print ""
    for r in range(N):
        print "Actual: %d" % r,
        print "|",
        for c in range(N):
            print "%6d" % int(c_mat[r, c]),
        print "|"

    print c_mat
    print 
    print "Number of instances: %d" % np.sum(c_mat)
    acc = np.trace(c_mat) / np.sum(c_mat) 
    print "Accuracy: %1.4f" % acc
    print
    for l in range(N):
        tpr = c_mat[l, l] / np.sum(c_mat[l, :])
        fpr = ((np.sum(c_mat[:, l]) - c_mat[l, l]) / 
                (np.sum(c_mat) - np.sum(c_mat[l, :])))
        spc = 1. - fpr
        print "Class %d stats: TPR %1.4f, SPEC %1.4f, FPR %1.4f" % (l, tpr, spc, fpr)

def run_fold_process(data):
    (classifier, cdata, test_fold_indicies) = data
    confusion_mat = np.zeros((3, 3))
    log("Building classifier...")
    st_time = rospy.Time.now().to_sec()
    classifier.train(cdata)
    end_time = rospy.Time.now().to_sec()
    log("Done building classifier (Time taken: %3.3f)" % (end_time - st_time))
    traj_dict = {}
    for i, t_inst in enumerate(cdata["testing_data"].T):
        pred = classifier.predict(t_inst.T)
        confusion_mat[int(cdata["testing_labels"][0, i]), int(pred)] += 1
        if not cdata["trajectory_labels"][i] in traj_dict:
            traj_dict[cdata["trajectory_labels"][i]] = []
        traj_dict[cdata["trajectory_labels"][i]].append([cdata["testing_times_list"][i], int(cdata["testing_labels"][0, i]), 
                     int(pred), cdata["testing_times_list"][i], test_fold_indicies[i]])
    log("Confusion Matrix:")
    log(confusion_mat)
    return (confusion_mat, traj_dict)

def pool_loading(fns):
    NUM_PROCESSES = 12
    pool = mp.Pool(NUM_PROCESSES)
    learners = pool.map(hard_load_pickle, fns) 
    return learners

def pool_saving(objs, fns):
    NUM_PROCESSES = 12
    pool = mp.Pool(NUM_PROCESSES)
    pool.map(hard_save_pickle, zip(objs, fns))

def hard_save_pickle(params):
    pickle_data, fn = params
    f = open(fn, "w")
    pickle.dump(pickle_data, f)
    f.close()

def hard_load_pickle(fn):
    f = open(fn, "r")
    p = pickle.load(f)
    f.close()
    return p

class RFBreimanClassifier(AbstractClassifier):
    
    def __init__(self, num_learners=100):
        super(RFBreimanClassifier, self).__init__()
        self.rfb = rf.RFBreiman(None, None, num_learners)
        self.num_learners = num_learners

    def predict(self, instance):
        pred, _ = self.rfb.predict(instance)
        return pred[0,0]

    def train(self, compiled_dataset):
        train = compiled_dataset["training_data"]
        responses = compiled_dataset["training_labels"]
        dataset = ds.Dataset(train, responses)
        self.rfb.train(dataset)

    def build(self, compiled_dataset, filename):
        # dataset = ldds.LinearDimReduceDataset(train, responses)
        # log("PCA dimension reduction")
        # dataset.pca_reduce(percent_var)
        # log("Reducing dataset to %d dimensions" % (dataset.projection_basis.shape[1]))
        # dataset.reduce_input()
        log("Building classifier...")
        st_time = rospy.Time.now().to_sec()
        self.train(compiled_dataset)
        end_time = rospy.Time.now().to_sec()
        log("Done building classifier (Time taken: %3.3f)" % (end_time - st_time))
        log("Average tree depth: %3f" % self.rfb.avg_tree_depth())
        # pb = dataset.projection_basis
        log("Saving...")
        pb = None
        self.fos.save_pickle((pb, self.rfb), 
                             filename)
        self._random_forest_split(filename)
        log("Finished saving")

    ##
    # multithreaded saving
    def _random_forest_split(self, filename, num_processes = 8):
        proj_basis, classifier = self.fos.load_pickle(filename)
        self.fos.save_pickle((classifier.number_of_learners, proj_basis), 
                        filename.split(".")[0] + "_split_index.pickle")
        fns = [self.fos.get_pickle_name(filename.split(".")[0] + 
                                 "_%03d.pickle" % (i)) for i in range(
                                                classifier.number_of_learners)]
        pool_saving(classifier.learners, fns)

    ##
    # multithreaded load
    def load(self, filename):
        try:
            log("Loading random forest classifier from pickle...")
            num_trees, projection_basis = self.fos.load_pickle(filename.split(".")[0] + "_split_index.pickle")
            self.rfb = rf.RFBreiman(number_of_learners=num_trees)
            fns = [self.fos.get_pickle_name(filename.split(".")[0] + 
                                 "_%03d.pickle" % (i)) for i in range(num_trees)]
            self.rfb.learners = pool_loading(fns)
            log("Classifier loaded")
        except Exception as e:
            err("Problem loading classifier (Has it been built?)")
            print e
            sys.exit()

class RFBreimanRefinedClassifier(RFBreimanClassifier):
    
    def __init__(self, num_learners=100, refine_folds=5, refine_learners=100, 
                                         refine_runs=5, refine_cut=2):
        super(RFBreimanRefinedClassifier, self).__init__(num_learners)
        self.refine_folds = refine_folds
        self.refine_learners = refine_learners
        self.refine_runs = refine_runs
        self.refine_cut = refine_cut

    def train(self, compiled_dataset):
        train = compiled_dataset["training_data"]
        responses = compiled_dataset["training_labels"]
        index_false_counts = [0] * train.shape[1]
        index_true_count = 0
        for r_run in range(self.refine_runs):
            cur_classifier = RFBreimanClassifier(self.refine_learners)
            c_mat_list, traj_dicts = cur_classifier._fold_predictions(compiled_dataset, 
                                                            self.refine_folds,
                                                            random.randint(0, 1000))
            # traj_dicts is a list of different folds,
            # together they represent a test on all of the data
            for traj_dict in traj_dicts:
                for i, k in enumerate(traj_dict):
                    cur_traj = traj_dict[k]
                    # pt is the result of an instance of data after prediction
                    # in a trajectory
                    for pt in cur_traj:
                        time, label, pred_label, coll_time, index = pt
                        if label != pred_label:
                            index_false_counts[index] += 1
                        else:
                            index_true_count += 1

            print index_true_count, sum(index_false_counts), index_true_count + sum(index_false_counts), (r_run + 1) * train.shape[1]
        indices_to_keep = []
        for ind, count in enumerate(index_false_counts):
            if count <= self.refine_cut:
                indices_to_keep.append(ind)
        print "Old shapes:", train.shape, responses.shape
        train = train[:, indices_to_keep]
        responses = responses[:, indices_to_keep]
        print "New shapes:", train.shape, responses.shape
        dataset = ds.Dataset(train, responses)
        self.rfb.train(dataset)


classifiers_dict = {"small_random_forest" : RFBreimanClassifier(20),
                    "large_random_forest" : RFBreimanClassifier(150),
                    "small_refined_random_forest" : RFBreimanRefinedClassifier(20, 8, 100, 5, 2),
                    "large_refined_random_forest" : RFBreimanRefinedClassifier(100, 8, 100, 5, 2)}
