import numpy as np
import sklearn
import sklearn.svm as svm
import pylab as pl
import pdb
from sklearn import cross_validation as cv
import sklearn.metrics as sm 

def plot_decision_boundary(model, X):
    h = .06 # step size in the mesh
    x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1
    y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
    xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))
    Z = model.predict(np.c_[xx.ravel(), yy.ravel()])
    Z = Z.reshape(xx.shape)
    pl.pcolormesh(xx, yy, Z)
    pl.set_cmap(pl.cm.Paired)

def plot_decision_boundary_active(model, X, Y, unknownX, unknownY):
    #plot_decision_boundary(model, np.row_stack((X, unknownX)), 
    #        np.concatenate((Y, 3+np.zeros(unknownX.shape[0]))))
    plot_decision_boundary(model, np.row_stack((X, unknownX)))
    # Plot also the training points
    pl.scatter(unknownX[:,0], unknownX[:,1], marker='o', c=unknownY.A1, linestyle='dotted')
    pl.scatter(X[:, 0], X[:, 1], marker='o', s=30, c=Y.A1)
    pl.axis('tight')
    #pl.show()
    #plot_decision_boundary(model, np.row_stack((X, unknownX)), np.concatenate((Y, unknownY)))

def mat_row_pop(m, index):
    return np.row_stack((m[:index, :], m[index+1:, :])), m[index,:]

def active_learn(dimensions):
    ##########################################################################
    # Generate data from two Gaussians => pool of labeled points
    class1CONST = 1.
    mean0 = np.zeros(dimensions)
    mean1 = np.zeros(dimensions)
    mean1[0] = 6
    
    class0 = np.random.multivariate_normal(mean0, np.matrix(np.eye(dimensions)), 200)
    class1 = np.random.multivariate_normal(mean1, np.matrix(np.eye(dimensions)), 200)
    refX = np.row_stack((class0, class1))
    refY = np.matrix(np.concatenate((np.zeros(class0.shape[0]), np.zeros(class1.shape[0])+class1CONST))).T
    
    ##########################################################################
    # Perform SVM active learning
    
    #Initialize with two data points, positive and negative
    class0cur = class0[1:,:]
    class1cur = class1[1:,:]
    poolX = np.row_stack((class0cur, class1cur))
    poolY = np.matrix(np.concatenate((np.zeros(class0cur.shape[0]), np.zeros(class1cur.shape[0])+class1CONST))).T
    X = np.row_stack((class0[0,:], class1[0,:]))
    Y = np.matrix(np.array([0,class1CONST])).T
    #Train svm
    gamma = .1
    c = 10
    model = svm.SVC(gamma=gamma, C=c, scale_C=True)
    model.fit(X, Y.A1)
    
    iteration = 0
    if dimensions == 2:
        plot_decision_boundary_active(model, X, Y, poolX, poolY)
        pl.savefig('decision_boundary_%3d.png' % iteration)
        pl.clf()
    
    while True:
        iteration += 1
        distances_sv = np.abs(model.decision_function(model.support_vectors_))
        distances = np.abs(model.decision_function(poolX))
    
        min_sv_dist = np.min(distances_sv)
        min_idx = np.argmin(distances)
        min_pool_dist = distances[min_idx,0]
        if min_pool_dist > min_sv_dist:
            break

        if len(poolY.A1) <= 1:
            break
    
        poolX, poppedX = mat_row_pop(poolX, min_idx)
        poolY, poppedY = mat_row_pop(poolY, min_idx)
        X = np.row_stack((X, poppedX))
        Y = np.row_stack((Y, poppedY))
    
        nneg = np.sum(Y == 0)
        npos = np.sum(Y == class1CONST)
        model = svm.SVC(gamma=gamma, C=c, scale_C=True)
        neg_to_pos_ratio = float(nneg)/float(npos)
        #neg_to_pos_ratio = float(npos)/float(nneg)
        model.fit(X, Y.A1, class_weight={0:1, 1:neg_to_pos_ratio})
        if dimensions == 2:
            plot_decision_boundary_active(model, X, Y, poolX, poolY)
            pl.savefig('decision_boundary_%3d.png' % iteration)
            pl.clf()
        print iteration, min_idx#, poppedX, poppedY
    
    loo = cv.LeaveOneOut(len(Y.A1))
    all_predicted = []
    iteration = 0
    for train, test in loo:
        xtrain = X[train,:]
        ytrain = Y[train,:]
        xtest = X[test,:]
        ytest = Y[test,:]
        print train, test
    
        nneg = np.sum(ytrain == 0)
        npos = np.sum(ytrain == class1CONST)
        model = svm.SVC(gamma=gamma, C=c, scale_C=True)
        model.fit(xtrain, ytrain.A1, class_weight={0:1, 1:neg_to_pos_ratio})
        ypredict = model.predict(xtest)
        all_predicted.append(ypredict)
        if dimensions == 2:
            plot_decision_boundary_active(model, xtrain, ytrain, refX, refY)
            pl.plot(xtest[:, 0], xtest[:, 1], marker='x')
            pl.savefig('leave_one_out_%3d.png' % iteration)
            pl.clf()
            iteration += 1
    
    all_predicted = np.concatenate(all_predicted)
    cmat = sm.confusion_matrix(Y.A1, all_predicted)
    correct_percentage = np.sum(all_predicted == Y.A1)/float(len(Y.A1))
    print 'Confusion matrix on training set\n', cmat
    print 'correct percentage: ', correct_percentage
    print 'ground truth', Y.A1
    print 'predicted', all_predicted
    return correct_percentage, cmat, len(Y.A1)

if __name__ == '__main__':
    #dims = [2,4,6,10,20,40,50]
    dims = [2]
    stats = []
    for d in dims:
        p, m, num_points = active_learn(d)
        stats.append([d, p,m, num_points])
    for d,p,m, num_points in stats:
        print 'dim', d, 'num_points', num_points, 'percentage', p, 'm\n', m, '\n'




