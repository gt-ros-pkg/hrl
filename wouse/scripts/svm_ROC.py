#!/usr/bin/env python

import pickle
import numpy as np
from scipy import interp
import pylab as pl

from sklearn import preprocessing as pps, svm
from sklearn.metrics import roc_curve, auc
from sklearn.cross_validation import StratifiedKFold, LeaveOneOut

with open('../data/svm_data.pkl', 'rb') as f:
    svm_data = pickle.load(f)
labels = svm_data['labels']
data = svm_data['data']

scaler = pps.Scaler().fit(data)
print "Mean: ", scaler.mean_
print "Std: ", scaler.std_
data_scaled = scaler.transform(data)

classifier = svm.SVC(probability=True)
classifier.fit(data_scaled, labels)

#print "Support Vectors: \r\n", classifier.support_vectors_
print "SV's per class: \r\n", classifier.n_support_


###############################################################################
## Code below modified from http://scikit-learn.org/stable/auto_examples/plot_roc_crossval.html#example-plot-roc-crossval-py
X, y = data_scaled, np.array(labels)
n_samples, n_features = X.shape
print n_samples, n_features

###############################################################################
# Classification and ROC analysis
# Run classifier with crossvalidation and plot ROC curves
cv = StratifiedKFold(y, k=9)

mean_tpr = 0.0
mean_fpr = np.linspace(0, 1, n_samples)
all_tpr = []

for i, (train, test) in enumerate(cv):
    probas_ = classifier.fit(X[train], y[train]).predict_proba(X[test])
    # Compute ROC curve and area the curve
    fpr, tpr, thresholds = roc_curve(y[test], probas_[:, 1])
    mean_tpr += interp(mean_fpr, fpr, tpr)
    mean_tpr[0] = 0.0
    roc_auc = auc(fpr, tpr)
    pl.plot(fpr, tpr, '--', lw=1, label='ROC fold %d (area = %0.2f)' % (i, roc_auc))

pl.plot([0, 1], [0, 1], '--', color=(0.6, 0.6, 0.6), label='Luck')

mean_tpr /= len(cv)
mean_tpr[-1] = 1.0
mean_auc = auc(mean_fpr, mean_tpr)
pl.plot(mean_fpr, mean_tpr, 'k-', lw=3,
        label='Mean ROC (area = %0.2f)' % mean_auc)

pl.xlim([0, 1])
pl.ylim([0, 1])
pl.xlabel('False Positive Rate')
pl.ylabel('True Positive Rate')
pl.title('Receiver Operating Characteristic')
pl.legend(loc="lower right")
pl.show()
