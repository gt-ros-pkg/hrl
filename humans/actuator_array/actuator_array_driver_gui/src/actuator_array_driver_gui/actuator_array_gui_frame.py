#! /usr/bin/env python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#* Copyright (c) 2011, A.M.Howard, S.Williams
#* All rights reserved.
#* 
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted provided that the following conditions are met:
#*     * Redistributions of source code must retain the above copyright
#*       notice, this list of conditions and the following disclaimer.
#*     * Redistributions in binary form must reproduce the above copyright
#*       notice, this list of conditions and the following disclaimer in the
#*       documentation and/or other materials provided with the distribution.
#*     * Neither the name of the <organization> nor the
#*       names of its contributors may be used to endorse or promote products
#*       derived from this software without specific prior written permission.
#*  
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#* DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
#* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

import roslib;
roslib.load_manifest('actuator_array_driver_gui')

import wx
import wx.lib.scrolledpanel

import rospy
from std_srvs.srv import *
from sensor_msgs.msg import JointState
from actuator_array_driver_gui.joint_panel import JointPanel


class ActuatorArrayGuiFrame(wx.Frame):
    def __init__(self):
    
        # Call base constructor  
        wx.Frame.__init__(self, None, wx.ID_ANY, title="ActuatorArray Driver GUI", size=(550, 400))

        # Initialize ROS Node
        rospy.init_node('actuator_array_driver_gui', anonymous=True)
        
        # Read from Parameter Server
        self.robot_description_parameter = rospy.get_param("~robot_description", "robot_description")
        self.joint_names = rospy.get_param("~joints")
        # TODO: get this information from the robot_description XML
        self.min_positions = [-3.14159] * len(self.joint_names)
        self.max_positions = [+3.14159] * len(self.joint_names)
        
        # Advertise Commands
        self.command_pub = rospy.Publisher("command", JointState)
        self.command_msg = JointState()
        self.command_msg.name = self.joint_names
        self.command_msg.position = [0] * len(self.joint_names)
        self.command_msg.velocity = [0] * len(self.joint_names)
        self.command_msg.effort = [0] * len(self.joint_names)

        # Subscribe to JointStates
        self.joint_state_sub = rospy.Subscriber("joint_states", JointState, self.joint_states_callback, None, 1)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [None] * len(self.joint_names)
        self.joint_state_msg.velocity = [None] * len(self.joint_names)
        self.joint_state_msg.effort = [None] * len(self.joint_names)
        
        # Create Service Call Proxies
        self.srv_home = rospy.ServiceProxy('home', Empty)
        self.srv_stop = rospy.ServiceProxy('stop', Empty)

        # Update the display every 10th of a second
        self.update_timer = wx.Timer(self, wx.ID_ANY)
        self.Bind(wx.EVT_TIMER, self._on_update_timer, self.update_timer)
        self.update_timer.Start(100, False)


        # Create menu
        self.menubar = wx.MenuBar()
        self.filemenu = wx.Menu()
        self.filemenu.Append(wx.ID_EXIT, 'E&xit', 'Exit the program')
        wx.EVT_MENU(self, wx.ID_EXIT, self.on_exit)
        self.menubar.Append(self.filemenu, '&File')
        self.SetMenuBar(self.menubar)

        # Create main panel
        self.main_panel = wx.Panel(self, wx.ID_ANY)
        

        # Create panel to hold joint controls
        self.joint_panel = wx.lib.scrolledpanel.ScrolledPanel(self.main_panel, wx.ID_ANY)

       
        joint_panel_sizer = wx.BoxSizer(wx.VERTICAL)
        title_font = wx.Font(pointSize=12, family=wx.DEFAULT, style=wx.NORMAL, weight=wx.BOLD)
        
        # Create Joint Controls
        self.joint_command_panels = []
        self.joint_status_panels = []
        for joint_name in self.joint_names:
            # Create label and controls for each joint
            joint_label = wx.StaticText(self.joint_panel, wx.ID_ANY, joint_name)
            joint_label.SetFont(title_font)
            joint_command_label = wx.StaticText(self.joint_panel, wx.ID_ANY, 'Command')
            joint_command_panel = JointPanel(self.joint_panel, joint_name=joint_name, input_mode=True)
            joint_status_label = wx.StaticText(self.joint_panel, wx.ID_ANY, 'Status')
            joint_status_panel = JointPanel(self.joint_panel, joint_name=joint_name, input_mode=False)
            sizer_joint = wx.FlexGridSizer(rows=2, cols=2, vgap=2, hgap=0)
            sizer_joint.SetFlexibleDirection(wx.BOTH)
            sizer_joint.AddGrowableCol(1, 1)
            sizer_joint.Add(joint_command_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)
            sizer_joint.Add(joint_command_panel, 1, wx.EXPAND)
            sizer_joint.Add(joint_status_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)
            sizer_joint.Add(joint_status_panel, 1, wx.EXPAND)
            
            joint_panel_sizer.Add((1,1), 1, wx.CENTER)
            joint_panel_sizer.Add(joint_label, 0, wx.CENTER | wx.ALL, 5)
            joint_panel_sizer.Add(sizer_joint, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)

            self.joint_command_panels.append(joint_command_panel)
            self.joint_status_panels.append(joint_status_panel)

        self.joint_panel.SetSizer(joint_panel_sizer)
        self.joint_panel.SetupScrolling()
        
        
        # Create panel to hold service buttons, etc
        self.button_panel = wx.Panel(self.main_panel, wx.ID_ANY)
        
        # Add buttons
        self.home_button = wx.Button(self.button_panel, wx.ID_ANY, 'Home')
        self.stop_button = wx.Button(self.button_panel, wx.ID_ANY, 'Stop')
        self.send_button = wx.Button(self.button_panel, wx.ID_ANY, 'Send')
        self.auto_button = wx.ToggleButton(self.button_panel, wx.ID_ANY, 'Auto Send')
        
        self.home_button.Bind(wx.EVT_BUTTON, self._on_home_button)
        self.stop_button.Bind(wx.EVT_BUTTON, self._on_stop_button)
        self.send_button.Bind(wx.EVT_BUTTON, self._on_send_button)
        self.auto_button.Bind(wx.EVT_TOGGLEBUTTON, self._on_auto_button)
        
        # Add to button panel sizer
        button_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        button_panel_sizer.Add(self.home_button, 0, wx.CENTER | wx.ALL, 10)
        button_panel_sizer.Add((1,1), 1, wx.CENTER | wx.ALL, 10)
        button_panel_sizer.Add(self.stop_button, 0, wx.CENTER | wx.ALL, 10)
        button_panel_sizer.Add((1,1), 1, wx.CENTER | wx.ALL, 10)
        button_panel_sizer.Add(self.send_button, 0, wx.CENTER | wx.ALL, 10)
        button_panel_sizer.Add((1,1), 1, wx.CENTER | wx.ALL, 10)
        button_panel_sizer.Add(self.auto_button, 0, wx.CENTER | wx.ALL, 10)
        
        self.button_panel.SetSizer(button_panel_sizer)

        # add the two main panels to a sizer
        main_panel_sizer = wx.BoxSizer(wx.VERTICAL)
        main_panel_sizer.Add(self.joint_panel, 1, wx.EXPAND)
        main_panel_sizer.Add(self.button_panel, 0, wx.EXPAND)
        self.main_panel.SetSizer(main_panel_sizer)
        
    def on_exit(self, e):
        self.Close(True)
        self.Refresh()
        
    def _on_home_button(self, event):
        try:
            # Turn off auto-send. Would cause robot to immediately start back
            self.auto_button.SetValue(False)
            self.send_button.Enable(True)
            # Send a 'stop" service request
            resp = self.srv_home()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def _on_stop_button(self, event):
        try:
            # Turn off auto-send. Would cause robot to immediately start moving again
            self.auto_button.SetValue(False)
            self.send_button.Enable(True)
            # Send a 'stop" service request
            resp = self.srv_stop()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def _on_send_button(self, event):
        self.command_publish()

    def _on_auto_button(self, event):
        self.send_button.Enable(not self.auto_button.GetValue())
        
    def _on_update_timer(self, event):
        # Loop over each status panel, update the properties from the message
        for i in range(len(self.joint_names)):
            self.joint_status_panels[i].position = self.joint_state_msg.position[i]
            self.joint_status_panels[i].velocity = self.joint_state_msg.velocity[i]
            self.joint_status_panels[i].effort = self.joint_state_msg.effort[i]
            self.joint_status_panels[i].update_panel()
        
        # If the auto-send flag is set, send out a command as well
        if self.auto_button.GetValue() == True:
            self.command_publish()
        
    def joint_states_callback(self, msg):
        # Copy data about matching joint names into the local joint_state message
        try:
            for i in range(len(msg.name)):
                j = self.joint_names.index(msg.name[i])
                self.joint_state_msg.position[j] = msg.position[i]
                self.joint_state_msg.velocity[j] = msg.velocity[i]
                self.joint_state_msg.effort[j] = msg.effort[i]
        except ValueError:
            # msg joint name not in the list of local joint names. Skip.
            pass
    
    def command_publish(self):
        # Get current values from the joint command panel and save in command message
        for i in range(len(self.joint_names)):
            self.command_msg.position[i] = self.joint_command_panels[i].position
            self.command_msg.velocity[i] = self.joint_command_panels[i].velocity
            self.command_msg.effort[i] = self.joint_command_panels[i].effort
        self.command_msg.header.stamp = rospy.Time.now()
        self.command_pub.publish(self.command_msg)
    