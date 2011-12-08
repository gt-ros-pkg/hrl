import numpy as np, math

X,Y = np.meshgrid( np.arange(0.0,11.0), np.arange(0.0,7.0) )
xy = np.column_stack([ X.flatten(), Y.flatten() ])

cap = ''
for i in xy:
    for j in [0.0, 90.0, 180.0, 270.0]:
    # for j in [0.0, 180.0]:
        cap +=  '    - [%6.1f, %6.1f, %6.1f]\n' % (i[0],i[1],j)

f = open( 'captures.yaml', 'w' )
f.write( cap )
f.close()
