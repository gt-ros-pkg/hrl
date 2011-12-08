import numpy as np, math
import pylab as pl

SENSITIVITY = -80 # dBm
THRESHOLD = -18 # dBm

# P^inc_rdr = P_rdr + 2*alpha > SENSITIVITY  (1)
# P^inc_tag = P_rdr +   alpha > THRESHOLD    (2)

# crossover = (1) - (2)

crossover_alpha = SENSITIVITY - THRESHOLD
crossover_Prdr = THRESHOLD - crossover_alpha

# alpha > 0.5 * (SENSITIVITY - Prdr)   (3)
# alpha >        THRESHOLD   - Prdr    (4)

print 'Crossover Point:\n\talpha: %2.2f\n\tPrdr: %2.2f' % (crossover_alpha, crossover_Prdr)

prdr = np.linspace( 0, 50 )
# alpha = np.linspace( -47, -30 )
alpha_3 = 0.5 * ( SENSITIVITY - prdr )
alpha_4 =         THRESHOLD   - prdr

f = pl.figure( figsize=(12,6) )
pl.axes([0.1,0.1,0.65,0.8])

f3 = pl.plot( prdr, alpha_3, 'g', linewidth = 3.0 )
pl.hold( True )
f4 = pl.plot( prdr, alpha_4, 'r', linewidth = 3.0 )
# x_min, x_max, y_min, y_max = pl.axis()
# pl.axis([-47,-30,y_min,y_max])

pl.legend((f3,f4),
          ('$P^{inc}_{rdr}$ > %d dBm' % (SENSITIVITY), '$P^{inc}_{tag}$ > %d dBm' % (THRESHOLD)),
          loc=(1.03,0.2))
pl.xlabel( '$P_{rdr}$ (dBm)' )
pl.ylabel( '$\\alpha$ (dBm)' )
pl.savefig('forward_link_limit.png')
pl.show()



