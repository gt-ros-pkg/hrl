#!/usr/bin/env python

import matplotlib
matplotlib.interactive( True )
matplotlib.use( 'WXAgg' )

import numpy as np
import wx
import sys

import roslib
roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'sensor_msgs' )
roslib.load_manifest( 'std_msgs' )


import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

from threading import Thread, Lock

class WXMatPlotLibPanel( wx.Panel ):
    def __init__( self, parent, color=None, dpi=None, **kwargs ):
        from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
        from matplotlib.figure import Figure

        # initialize Panel
        if 'id' not in kwargs.keys():
            kwargs['id'] = wx.ID_ANY
        if 'style' not in kwargs.keys():
            kwargs['style'] = wx.NO_FULL_REPAINT_ON_RESIZE
        wx.Panel.__init__( self, parent, **kwargs )

        # initialize matplotlib stuff
        self.figure = Figure( None, dpi )
        self.canvas = FigureCanvasWxAgg( self, -1, self.figure )
        #self.SetColor( color )

        self._SetSize()
        self.draw()

        self._resizeflag = False

        self.Bind(wx.EVT_IDLE, self._onIdle)
        self.Bind(wx.EVT_SIZE, self._onSize)

    def SetColor( self, rgbtuple=None ):
        """Set figure and canvas colours to be the same."""
        if rgbtuple is None:
            rgbtuple = wx.SystemSettings.GetColour( wx.SYS_COLOUR_BTNFACE ).Get()
        clr = [c/255. for c in rgbtuple]
        self.figure.set_facecolor( clr )
        self.figure.set_edgecolor( clr )
        self.canvas.SetBackgroundColour( wx.Colour( *rgbtuple ) )

    def _onSize( self, event ):
        self._resizeflag = True

    def _onIdle( self, evt ):
        if self._resizeflag:
            self._resizeflag = False
            self._SetSize()

    def _SetSize( self ):
        pixels = tuple( self.parent.GetClientSize() )
        self.SetSize( pixels )
        self.canvas.SetSize( pixels )
        self.figure.set_size_inches( float( pixels[0] )/self.figure.get_dpi(),
                                     float( pixels[1] )/self.figure.get_dpi() )

    def draw(self): pass # abstract, to be overridden by child classes

class JointStateVizPanel( WXMatPlotLibPanel ):
    def __init__( self, parent, **kwargs ):
        self.lock = Lock()
        self.parent = parent
        self.positions = None
        self.velocities = None
        self.efforts = None
        WXMatPlotLibPanel.__init__( self, parent, **kwargs )
        #self.SetColor( (255,255,255) )

    def init_plots( self, names, limits ):
        N = len( names )
        self.axes = []
        self.lines = []
        self.scatter = []
        for j in range( N ):
            axes = self.figure.add_subplot( 3, N, j + 1 )
            axes.set_title( names[j] )
            upper = -np.pi
            lower = np.pi

            if limits[j].has_key( 'upper' ):
                upper = limits[j].get( 'upper' )

            if limits[j].has_key( 'lower' ):
                lower = limits[j].get( 'lower' )

            axes.set_ylim( lower, upper )
            axes.set_xticks( [] )

            self.lines.append( axes.plot( [0], 'r' )[0] )

            axes.autoscale_view( scalex = False, scaley = False )

            self.axes.append( axes )

        for j in range( N ):
            axes = self.figure.add_subplot( 3, N, j + N + 1 )
            effort = 30

            if limits[j].has_key( 'effort' ):
                effort = limits[j].get( 'effort' )

            axes.set_ylim( -effort, effort )
            axes.set_xticks( [] )

            self.lines.append( axes.plot( [0], 'b' )[0] )

            self.axes.append( axes )

        for j in range( N ):
            axes = self.figure.add_subplot( 3, N, j + 2 * N + 1 )
            upper = -2 * np.pi
            lower = 2 * np.pi
            velocity = 2 * np.pi

            if limits[j].has_key( 'upper' ):
                upper = limits[j].get( 'upper' )

            if limits[j].has_key( 'lower' ):
                lower = limits[j].get( 'lower' )

            if limits[j].has_key( 'velocity' ):
                velocity = limits[j].get( 'velocity' )

            axes.set_ylim( -velocity, velocity )
            axes.set_xlim( lower, upper )

            self.scatter.append( axes.scatter( [0], [0], 
                                               c = 'b',
                                               s = 2.0,
                                               marker = 'x') )

            self.axes.append( axes )


        #self.figure.subplots_adjust( wspace = 0.3, hspace = 0.1 )

    def update_plots( self ):
        # self.lock.acquire()
        
        if not hasattr( self, 'axes' ):  
            return
            
        if self.positions is None or len( self.positions ) == 0:
            return

        ( l, N ) = self.positions.shape

        for j in range( N ):
            axes = self.axes[j]
            axes.lines[0].remove()
            axes.plot( self.positions[:,j], 'r')
            
        for j in range( N ):
            axes = self.axes[j + N]
            axes.lines[0].remove()
            axes.plot( self.efforts[:,j], 'b')


        for j in range( N ):
            axes = self.axes[j + N * 2]
            self.scatter[j].remove()
            self.scatter[j] = axes.scatter( self.positions[:,j], 
                          self.velocities[:,j],
                          c = 'b',
                          s = 2.0,
                          marker = 'x')

        # self.lock.release()

    def draw( self ):
        self.update_plots()

    def setData( self, pos, vel, eff, duration ):
        self.positions = pos
        self.velocities = vel
        self.efforts = eff
        self.duration = duration
        self.draw()

class JointStateVizFrame( wx.Frame ) :
    def __init__( self, title, ns, js_topic ):
        wx.Frame.__init__( self, None, wx.ID_ANY, title, size = (2000, 800) )
        self.CreateStatusBar()
        self.SetStatusText( 'waiting for data on %s ...'%js_topic )
        self.panel = JointStateVizPanel( self )
        self.Show()

        self.js_sub = rospy.Subscriber( js_topic, JointState, self.js_callback )
        self.update_sub = rospy.Subscriber( ''.join( (ns, '/update' ) ), Int8, self.update_callback )

        # load parameters
        (self.joint_names, self.limits) = self.load_params( ns )

        self.idx = None
        self.joint_states = []

        self.panel.init_plots( self.joint_names, self.limits )
    
    def load_params( self, ns ):
        params = rospy.get_param( ns )
        joint_names = params.keys()

        limits = []
        for joint_name in joint_names:
            limits.append( params.get( joint_name ) )

        return ( joint_names, limits )

    def js_callback( self, msg ):
        self.joint_states.append( msg )

    def update_callback( self, msg ):
        if msg.data == 0:
            self.joint_states = []
        elif msg.data == 1:
            self.update()

    def update( self ):
        if self.joint_names is None or len( self.joint_names ) == 0 or len( self.joint_states ) == 0:
            return
        if self.idx is None:
            self.idx = map( self.joint_states[0].name.index, self.joint_names )

        positions = []
        velocities = []
        efforts = []

        for js in self.joint_states:
            pos =  map( lambda x: js.position[x], self.idx )
            vel = map( lambda x: js.velocity[x], self.idx )
            effort = map( lambda x: js.effort[x], self.idx )
            positions.append( pos )
            velocities.append( vel )
            efforts.append( effort )

        start_time = self.joint_states[0].header.stamp
        stop_time = self.joint_states[-1].header.stamp
        duration = stop_time - start_time

        self.panel.setData( np.asarray( positions ), 
                            np.asarray( velocities ), 
                            np.asarray( efforts ),
                            duration.to_sec())

        # status = ''.join( ( str( self.joint_states[0].header.stamp ),
        #                     ' ',
        #                     str( duration.to_sec() ) ) )
        
        status = 'ros-time: %s, duration: %5.3f sec'%( self.joint_states[0].header.stamp, duration.to_sec() )
        
        self.SetStatusText( status ) 

        self.joint_states = []




arm_joint_names = ['lr_shoulder_pan_joint',
                   'lr_shoulder_lift_joint',
                   'lr_upper_arm_roll_joint',
                   'lr_elbow_flex_joint',
                   'lr_forearm_roll_joint',
                   'lr_wrist_flex_joint',
                   'lr_wrist_roll_joint']

l_arm_joint_names = map(lambda s: s.replace('lr_', 'l_'), arm_joint_names )
r_arm_joint_names = map(lambda s: s.replace('lr_', 'r_'), arm_joint_names )


if __name__ == '__main__':
    rospy.init_node( 'joint_state_viz_node' )
    app = wx.PySimpleApp( 0 )
    frame = JointStateVizFrame( 'Joint-State-Viz', '/joint_state_viz', '/joint_states' )
    app.MainLoop()
    


    
