import numpy as np, math
import pylab as pl

ar = [[  0, 104 ],
      [  1, 104 ],
      [  2, 104 ],
      [  3, 104 ],
      [  4, 104 ],
      [  8, 100 ],
      [  9,  97 ],
      [ 10,  96 ],
      [ 11,  95 ],
      [ 12,  93 ],
      [ 13,  91 ],
      [ 14,  90 ],
      [ 15,  89 ],
      [ 16,  87 ],
      [ 17,  85 ],
      [ 18,  84 ],
      [ 19,  83 ],
      [ 20,  81 ],
      [ 21,  80 ],
      [ 22,  78 ],
      [ 23,  77 ],
      [ 24,  74 ],
      [ 25,  73 ],
      [ 26,  71 ]]

ar = np.array( ar ).T
ar[0] *= -1.0   # Change attenuation to be negative.

# Linear least squares
a = ar[0,4:] # Start at attn = -4 to avoid including saturation region.
a = np.column_stack([ a, np.ones(len(a)) ])
b = ar[1,4:]
m,b = np.linalg.lstsq( a, b )[0]

xs = np.linspace( -27, -3, 100 )
ys = xs * m + b

if __name__ == '__main__':
    
    pl.plot( ar[0], ar[1], 'bo', linewidth=2.0 )
    pl.hold( True )

    pl.plot( xs, ys, 'g-', linewidth = 2.0 )

    pl.xlabel( 'Attenuator Setting (dB)')
    pl.ylabel( 'RSSI' )
    pl.legend([ 'Measurements', 'Linear Fit' ], loc='upper left')

    pl.savefig( 'Attenuator_RSSI_measurements.png' )               
    pl.show()

