
# initially copied from:
# http://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query.html

from scipy.spatial import KDTree
import numpy as np, math

x, y = np.mgrid[0:5, 2:8]
tree = KDTree(zip(x.ravel(), y.ravel()))

tree.data

pt = np.array([0,0])

print 'Querying neibors for ', pt

print 'k=1'
print 'Output of query:', tree.query(pt)
print 'Neighbors:', tree.data[tree.query(pt)[1]]

print 'k=2'
print 'Output of query:', tree.query(pt, 2)
print 'Neighbors:', tree.data[tree.query(pt, 2)[1]]

print ''
print 'Simple query returns (array of distances, array of indices)'
print ''




