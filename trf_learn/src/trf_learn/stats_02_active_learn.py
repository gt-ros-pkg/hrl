import numpy as np
import sklearn
import sklearn.svm as svm
import pylab as pl
import pdb

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
    pl.scatter(unknownX[:,0], unknownX[:,1], marker='D', c=unknownY.A1, linestyle='dotted')
    pl.scatter(X[:, 0], X[:, 1], marker='o', s=30, c=Y.A1)
    pl.axis('tight')
    #pl.show()
    #plot_decision_boundary(model, np.row_stack((X, unknownX)), np.concatenate((Y, unknownY)))

def mat_row_pop(m, index):
    return np.row_stack((m[:index, :], m[index+1:, :])), m[index,:]

##########################################################################
# Generate data from two Gaussians => pool of labeled points
class0 = np.random.multivariate_normal(np.array([0,0]), np.matrix(np.eye(2)), 200)
class1 = np.random.multivariate_normal(np.array([2,0]), np.matrix(np.eye(2)), 200)
class0cur = class0[1:,:]
class1cur = class1[1:,:]
poolX = np.row_stack((class0cur, class1cur))
class1CONST = 100.
poolY = np.matrix(np.concatenate((np.zeros(class0cur.shape[0]), np.zeros(class1cur.shape[0])+class1CONST))).T

##########################################################################
# Perform SVM active learning

#Initialize with two data points, positive and negative
X = np.row_stack((class0[0,:], class1[0,:]))
Y = np.matrix(np.array([0,class1CONST])).T
#Train svm
model = svm.SVC(gamma=.1, C=.1, scale_C=True)
model.fit(X, Y.A1)

iteration = 0
plot_decision_boundary_active(model, X, Y, poolX, poolY)
print model.support_, model.support_.__class__
pl.savefig('%d_decision_boundary.png' % iteration)

while True:
    distances_sv = model.decision_function(model.support_vectors_)
    distances = model.decision_function(poolX)

    min_sv_dist = np.min(distances_sv)
    min_idx = np.argmin(distances)
    min_pool_dist = distances[min_idx,0]
    if min_pool_dist > min_sv_dist:
        break

    poolX, poppedX = mat_row_pop(poolX, min_idx)
    poolY, poppedY = mat_row_pop(poolY, min_idx)
    X = np.row_stack((X, poppedX))
    Y = np.row_stack((Y, poppedY))
    plot_decision_boundary_active(model, X, Y, poolX, poolY)
    pl.savefig('%d_decision_boundary.png' % iteration)
    print min_idx, poppedX, poppedY
    iteration += 1

#Repeat
#while(True):
#	Check to see if there are any points from our pool that is closer than support vectors, 

#	    pick the closest one of them
#   Train our svm again

# Plot results
# Perform cross validation

