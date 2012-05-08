import roslib; roslib.load_manifest('trf_learn')
import rospy
import hrl_lib.util as ut
from trf_learn.recognize_3d import *
import sys
import pdb
import hrl_lib.rutils as ru
import os.path as pt
import glob
import numpy as np
from sklearn import cross_validation as cv
import sklearn.svm as svm
import sklearn.metrics as sm 

def trf_select_features(instances, features_to_use, sizes):
    offset = 0
    fset = set(features_to_use)
    selected_fea = []
    for k in ['expected_loc', 'distance', 'fpfh', 'intensity']:
        if not sizes.has_key(k):
            continue
        start_idx = offset 
        end_idx = offset + sizes[k]
        if fset.issuperset([k]):
            selected_fea.append(instances[start_idx:end_idx, :])
        offset = end_idx
    return np.row_stack(selected_fea)

def cache_outputs(afunc, func_args, cache_name):
    #places to save intermediate computation to 
    if not pt.exists(cache_name):
        results = afunc(*func_args)
        ut.save_pickle(results, cache_name)
        print 'cache_outputs: saved to cache named', cache_name
    else:
        print 'cache_outputs: loading cache named', cache_name
        results  = ut.load_pickle(cache_name)
    return results

def calculate_pca(pca_dataset_name, directory):
    ##
    #Calculate PCA and save it.
    pca_dataset = ut.load_pickle(pca_dataset_name)
    pca_dataset_mat = pca_dataset['instances']
    pca_dataset_mat_just_intensities = trf_select_features(pca_dataset_mat, ['intensity'], pca_dataset['sizes'])
    reconstruction_err_toler = .05
    reconstruction_std_lim   = 1.
    variance_keep            = .99
    intensities_index        = 0
    pca = PCAIntensities(intensities_index, reconstruction_std_lim, reconstruction_err_toler)
    pca.calculate_pca_vectors(pca_dataset_mat_just_intensities, variance_keep)
    pca_dataset_projected = pca.partial_pca_project(pca_dataset_mat_just_intensities)
    scale = DataScale()
    scale.scale(pca_dataset_projected)
    return pca, scale

def load_and_project_all_datafiles(directory, ext, pca, scale, exclude=[]):
    data_file_names = glob.glob(pt.join(directory, '*' + ext))
    datamats = []
    datalabels = []
    for data_file in data_file_names:
        if data_file in exclude:
            continue
        print 'load_and_project_all_datafiles: processing', data_file
        dataset = ut.load_pickle(data_file)
        datamat = trf_select_features(dataset['instances'], ['intensity'], dataset['sizes'])
        datamat_projected = pca.partial_pca_project(datamat)
        datamat_scaled = scale.scale(datamat_projected)
        datamats.append(datamat_scaled)
        datalabels.append(dataset['labels'])

    #big_datamat   = np.column_stack(datamats)
    #big_datalabel = np.column_stack(datalabels)
    #return big_datamat, big_datalabel
    return datamats, datalabels

def select_from_list(alist, idx):
    b = []
    for i in idx:
        b.append(alist[i])
    return b

def conf_mat_to_conf_percent(cmat):
    label_sums = np.sum(cmat, 1)
    rmat = np.matrix(cmat.copy(), dtype="float")
    for i in range(cmat.shape[0]):
        rmat[i,:] = cmat[i,:] / float(label_sums[i])
    return rmat

def cross_validate_with_datasets(datamats, datalabels, noise_cost_c, gamma):
    #c     = .5
    #gamma = .5
    class1CONST = 1.
    #model = svm.SVC(gamma=gamma, C=c, scale_C=True)
    #scores = cv.cross_val_score(model, np.column_stack(datamats).T.A,
    #                            np.column_stack(datalabels).T.A1, cv=10)
    #print 'Scores for 10 fold cross validation'
    #print scores
    #print "Accuracy: %0.2f (+/- %0.2f)" % (scores.mean(), scores.std() / 2)

    all_predicted = []
    all_actual = []
    loo = cv.LeaveOneOut(len(datamats))
    i = 0
    correct_percentages = []
    conf_mats = []
    for train, test in loo:
        print "Testing fold", i+1, 'training', train, 'test', test
        #pdb.set_trace()
        xtrain = np.column_stack(select_from_list(datamats, train)).T.A
        ytrain = np.column_stack(select_from_list(datalabels, train)).T.A1

        xtest = np.column_stack(select_from_list(datamats, test)).T.A
        ytest = np.column_stack(select_from_list(datalabels, test)).T.A1

        #pdb.set_trace()
        nneg = np.sum(ytrain == 0)
        npos = np.sum(ytrain == class1CONST)
        neg_to_pos_ratio = float(nneg)/float(npos)

        print 'Training'
        model = svm.SVC(gamma=gamma, C=noise_cost_c, scale_C=True)
        model.fit(xtrain, ytrain, class_weight={0:1, 1:neg_to_pos_ratio})

        ypredict = model.predict(xtest)
        all_predicted.append(ypredict)
        all_actual.append(ytest)

        pcorrect = np.sum(ypredict == ytest)/ float(len(ypredict))
        correct_percentages.append(pcorrect)
        print 'Correct percentage', pcorrect
        conf_mat = sm.confusion_matrix(ypredict, ytest)
        conf_mats.append(conf_mat)
        print 'Confusion matrix\n', conf_mat
        i += 1

    return np.concatenate(all_predicted), np.concatenate(all_actual), \
                          correct_percentages, conf_mats


def eval_performance(actual, predicted):
    cmat = sm.confusion_matrix(actual, predicted)
    conf_percent = conf_mat_to_conf_percent(cmat)
    return cmat, conf_percent, (conf_percent[0,0] + conf_percent[1,1])/2.


def grid_eval(datamats, datalabels, Crange=[-5,5], Grange=[-5,5]):
    C_range     = 10. ** np.arange(Crange[0], Crange[1])
    gamma_range = 10. ** np.arange(Grange[0], Grange[1])
    Cs, Gammas  = np.meshgrid(C_range, gamma_range)

    eval_results = []
    #pdb.set_trace()
    for c, gamma in zip(Cs.tolist()[0], Gammas.tolist()[0]):
        all_predicted, all_actual, correct_percentages, confmats =\
                cross_validate_with_datasets(datamats, datalabels, c, gamma)
        cmat, conf_percent, score = eval_performance(all_actual, all_predicted)
        eval_results.append({'score': score, 'cmat': cmat, 'params': [c,gamma]})

    return eval_results

def scrap():
    cv_pkl_name = pt.join(directory, 'stats_03_cv.pkl')
    all_predicted, all_actual, correct_percentages, confmats = \
            cache_outputs(cross_validate_with_datasets, [datamats, datalabels], cv_pkl_name)

    cmat, conf_percent, class_accuracy = eval_performance(all_actual, all_predicted)
    correct_percentage = np.sum(all_predicted == all_actual)/float(len(all_actual))
    print 'Confusion matrix on training set\n', cmat
    print conf_percent
    print 'correct percentage: ', correct_percentage
    print 'all correct percentages, ', correct_percentages

def subset_selection(percent, X, Y):
    ndatapoints = X.shape[1]
    number_of_points = np.ceil(ndatapoints * percent)
    selected_indices = np.random.permutation(np.array(range(ndatapoints)))[0:number_of_points]
    return X[:, selected_indices], Y[:, selected_indices]

def pca_vector_selection(nvectors, X):
    return X[0:nvectors, :]

def pca_vector_selection_dataset(nvectors, Xs):
    Xpca = []
    for x in Xs:
        Xpca.append(pca_vector_selection(nvectors, x))
    print 'Selecting a subset of dimensions to train on:', nvectors
    return Xpca

def subset_dataset(percent, Xs, Ys):
    Xsub = []
    Ysub = []
    for x, y in zip(Xs, Ys):
        xsub, ysub = subset_selection(percent, x, y)
        Xsub.append(xsub)
        Ysub.append(ysub)
    return Xsub, Ysub
##
# Uses dataset of the format ['points2d', 'sizes', 'points3d', 'image', 'labels', 'instances', 'synthetic_locs3d']
def cross_validate_script(pca_dataset_name, directory, ext='preprocessed_and_has_image_patches.pkl'):
    pca_pkl_name = pt.join(directory, 'stats_03_pca.pkl')
    datamat_pkl_name = pt.join(directory, 'stats_03_datamat.pkl')
    grid_eval_pkl = pt.join(directory, 'stats_03_grid_eval.pkl')
    subset_pkl = pt.join(directory, 'stats_03_subset.pkl')

    ##
    #Designate one dataset as the dataset to load PCA from
    pca, scale = cache_outputs(calculate_pca, [pca_dataset_name, directory], pca_pkl_name)

    ##
    #Load each dataset and project it with PCA, save then all as one big dataset with labels
    #dataset for cross validating
    datamats, datalabels = cache_outputs(load_and_project_all_datafiles, 
                                               [directory, ext, pca, scale, [pca_dataset_name]],
                                               datamat_pkl_name)

    datamats_sub, datalabels_sub = cache_outputs(subset_dataset, [1., datamats, datalabels], 
                                                    subset_pkl)

    datamats_sub_pca = pca_vector_selection_dataset(1, datamats_sub)

    ##
    # Do a grid search
    score_dicts = cache_outputs(grid_eval, [datamats_sub_pca, datalabels_sub, [-5,5], [-5,5]], grid_eval_pkl)
    #score_dicts = cache_outputs(grid_eval, [datamats_sub, datalabels_sub, [0,1], [0,1]], grid_eval_pkl)
    pdb.set_trace()

    scores = []
    for d in score_dicts:
        print 'params:', d['params']
        print 'cmat:\n',   d['cmat']
        print 'score:',  d['score']
        scores.append(d['score'])
    print 'THE MAX SCORE IS:\n', score_dicts[np.argmax(scores)]

    

if __name__ == '__main__':
    cross_validate_script(sys.argv[1], sys.argv[2])


