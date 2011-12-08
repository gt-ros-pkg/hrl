import numpy as np
from numpy import pi
import time
import transforms as tr
import pylab as pl
import functools as fct
import pickle as pkl
from scipy.special import erf
import prob as pb
import optparse

waveLen = 3e8 / 900e6  # Middle of UHF RFID band

class AntennaGain():
    def __init__(self, RadiationPattern, Gmax = None, GmaxDB = None, front_back_ratio = None):
        if Gmax is not None:
            self.gain = Gmax
        elif GmaxDB is not None:
            self.gain = 10.0 ** (GmaxDB/10.0)
        else:
            self.gain = 1.0

        # If the FBR is 8 dB => fbr = DBToWatts( -8 )
        self.fbr = front_back_ratio 
            
        self.RadiationPattern = RadiationPattern
    
    def G(self, theta, phi):
        rv = self.RadiationPattern( standard_rad(theta), standard_rad(phi)) * self.gain
        # Account for front-back ratio
        if self.fbr:
            rv = np.max([ self.fbr * self.gain, rv ])
        return rv

    def Gdb(self, theta, phi):
        #return np.max([ -80.0, WattsToDB( self.G( theta, phi ))]) # Assume always > -80db
        return WattsToDB( self.G( theta, phi ))
    
def CartToSphere(x, y, z):
    r = np.power( np.power(x,2) + np.power(y,2) + np.power(z,2), 0.5)
    theta = np.arctan2( np.power(np.power(x,2) + np.power(y,2),0.5) , z)
    phi = np.arctan2( y, x )
    return (r,theta,phi)

def CartToSphere2(x, y, z): # Equivalent
    r = np.power( np.power(x,2) + np.power(y,2) + np.power(z,2), 0.5)
    theta = np.arccos( z / r )
    phi = np.arctan2( y, x )
    return (r,theta,phi)

def mCartToSphere(v):
    x = v[0]
    y = v[1]
    z = v[2]
    r = np.power( np.power(x,2) + np.power(y,2) + np.power(z,2), 0.5)
    theta = np.arctan2( np.power(np.power(x,2) + np.power(y,2),0.5) , z)
    phi = np.arctan2( y, x )
    return (r,theta,phi)

def SphereToCart(r, theta, phi):
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return (x,y,z)

def WattsToDBm(pwr):
    return 10.0 * np.log10(pwr) + 30.0

def WattsToDB(pwr):
    return 10.0 * np.log10(pwr)

def DBmToWatts(pwr):
    return np.power(10.0, (pwr - 30.0) / 10.0)

def DBToWatts(pwr):
    return np.power(10.0, (pwr) / 10.0)

def PL( radius ):
    return np.power( waveLen/(4*pi*radius), 2.0 )

def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi

# Dipole Antenna

def rad_dipole(theta, phi):
    return np.power( np.sin(theta), 2.0 )

dipole = AntennaGain(rad_dipole, Gmax=1.5, front_back_ratio = DBToWatts(-8) )
#dipole = AntennaGain(rad_dipole, Gmax=1.5 )

# Isotropic Antenna

isotropic = AntennaGain( lambda theta,phi: 1, Gmax=1.0 )

# Patch Antenna

alpha = 1.0
k0 = 2*pi / waveLen
# See balanis 3-27: k^2 = omega^2 * mu * epsilon
# Free space: mu * epsilon = 1/c^2
# So k0 = w/c = 2pi * freq / c = 2pi / lambda  (since c = f*lambda)

width = waveLen / 2.0
Leff = 1.02*waveLen / 2.0        # This is probably waveLen / 2.0 + epsilon

def rad_patch(theta, phi):
    t1 = np.sin(theta)
    t2 = np.sin(k0*width/2.0 * np.cos(theta)) / np.cos(theta)
    t3 = np.cos(k0*Leff/2.0 * np.sin(theta)*np.sin(phi))
    #return alpha * np.power(t1*t2*t3,2)
    if -pi/2 <= phi <= pi/2:
        return alpha * np.power(t1*t2*t3,2)
    else:
        return alpha * np.power(t1*t2*t3,2) * 0.0001

# S9028?
#patch = AntennaGain( rad_patch, Gmax=5.623/rad_patch(pi/2,0.0)) #7.5dBi ==> 5.623 ==> 18.233 multiplier (since rad_patch_max = 0.30841937174)

# S9025P
#patch = AntennaGain( rad_patch, Gmax=3.548/rad_patch(pi/2,0.0)) # 5.5dBi boresight. => 3.548 [ 10**(5.5/10) ]
# note: we modify Gmax instead of alpha st. Gmax * rad_patch(pi/2) => 5.5dB
patch = AntennaGain( rad_patch,
                     Gmax=3.548/rad_patch(pi/2,0.0), # 5.5dBi boresight. => 3.548 [ 10**(5.5/10) ]
                     front_back_ratio = DBToWatts( -8 ))  # 8dB front-back ratio 


# Friis Forward

def Friis_Inc_Tag( Prdr, CL, waveLen, radius,
                   Grdr_func, theta_rdr, phi_rdr,
                   Gtag_func, theta_tag, phi_tag ):
    return Prdr * CL * Grdr_func(theta_rdr, phi_rdr) * PL(radius) * Gtag_func(theta_tag, phi_tag)

pwr_inc_tag = fct.partial( Friis_Inc_Tag, 1.0, 0.5, waveLen )

def Friis_Inc_Rdr( Prdr, CL, waveLen, AlphaBeta, radius, 
                   Grdr_func, theta_rdr, phi_rdr,
                   Gtag_func, theta_tag, phi_tag ):
    inter = np.power( CL * Grdr_func(theta_rdr, phi_rdr) * PL(radius) * Gtag_func(theta_tag, phi_tag), 2.0 )
    return Prdr * AlphaBeta * inter

pwr_inc_rdr = fct.partial( Friis_Inc_Rdr, 1.0, 0.5, waveLen, 1.0 )


def plot_gain_patterns( gain_func, sup_title, label_psi_plane, label_theta_plane ):
    gf = gain_func
    
    psi = np.linspace( -np.pi, np.pi, 50 )
    theta_front = np.linspace( 0, np.pi, 50 )
    theta_back = theta_front[::-1] * -1.0
    
    fig = pl.figure()
    fig.suptitle( sup_title )
    
    ax1 = fig.add_subplot( 121, polar=True)
    ax1.plot( psi, np.array([ gf( np.pi / 2.0, p ) for p in psi ]), 'bo-' )
    ax1.set_title( label_psi_plane )
    

    # Hacky to get rotated polar...
    ax2 = fig.add_subplot( 122, polar=True )
    ax2.hold( True )
    ax2.plot( theta_front - np.pi / 2, np.array([ gf( t, 0.0 ) for t in theta_front ]), 'ro-' )
    ax2.plot( theta_back - np.pi / 2, np.array([ gf( -1.0*t, -np.pi ) for t in theta_back ]), 'ro-' ) # the negative goes to psi
    pl.thetagrids( [0, 45, 90, 135, 180, 225, 270, 315], [90, 45, 0, -45, -90, -135, 180, 135 ] )
    ax2.set_title( label_theta_plane )

    pl.show()



if __name__ == "__main__":
    p = optparse.OptionParser()
    p.add_option('-a', '--patch', action='store_true', dest='patch', help='Look at patch antenna.')
    opt, args = p.parse_args()
    
    if opt.patch:
        # Patch (verify E/H plane designations?)
        plot_gain_patterns( patch.G, 'Patch Antenna Gains (Absolute)', 'E-Plane', 'H-Plane' )
        #plot_gain_patterns( patch.Gdb, 'Patch Antenna Gains', 'E-Plane', 'H-Plane' )
    else:
        # Dipole (verify E/H plane designations?)
        plot_gain_patterns( dipole.G, 'Dipole Antenna Gains (Absolute)', 'H-Plane', 'E-Plane' )
        #plot_gain_patterns( dipole.Gdb, 'Dipole Antenna Gains', 'H-Plane', 'E-Plane' )
    
