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

import wx

class JointPanel(wx.Panel):
    def __init__(self, parent, joint_name="joint", min_position=-3.14159, max_position=3.14159, max_velocity=100.0, max_effort=100.0, input_mode=True):
        
        # Create Joint Properties and initialize to None
        self.joint_name = joint_name
        self.min_position = min_position
        self.max_position = max_position
        self.max_velocity = max_velocity
        self.max_effort = max_effort
        self.position = (max_position - min_position)/2 + min_position
        self.velocity = max_velocity/2
        self.effort = max_effort
        self.input_mode = input_mode
        
        # Set some control properties
        self.slider_increments = 100
        
        # Create Control Panel
        wx.Panel.__init__(self, parent, wx.ID_ANY, style=wx.BORDER_SUNKEN)
        
        # Create slider: min_label - slider - max_label
        self.min_position_label = wx.StaticText(self, wx.ID_ANY, str(self.min_position))
        self.position_slider = wx.Slider(self, wx.ID_ANY, minValue=0, maxValue=self.slider_increments, style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
        self.position_slider.Enable(self.input_mode)
        self.max_position_label = wx.StaticText(self, wx.ID_ANY, str(self.max_position))
        
        sizer_slider = wx.BoxSizer(wx.HORIZONTAL)
        sizer_slider.Add(self.min_position_label, 0, wx.CENTER | wx.ALL, 5)
        sizer_slider.Add(self.position_slider, 1, wx.CENTER)
        sizer_slider.Add(self.max_position_label, 0, wx.CENTER | wx.ALL, 5)
        
        # Create Joint Properties display boxes
        self.position_label = wx.StaticText(self, wx.ID_ANY, "Position")
        self.position_text = wx.TextCtrl(self, wx.ID_ANY, style=wx.TE_PROCESS_ENTER)
        self.velocity_label = wx.StaticText(self, wx.ID_ANY, "Velocity")
        self.velocity_text = wx.TextCtrl(self, wx.ID_ANY, style=wx.TE_PROCESS_ENTER)
        self.effort_label = wx.StaticText(self, wx.ID_ANY, "Effort")
        self.effort_text = wx.TextCtrl(self, wx.ID_ANY, style=wx.TE_PROCESS_ENTER)
        
        sizer_properties = wx.BoxSizer(wx.HORIZONTAL)
        sizer_properties.Add(self.position_label, 0, wx.CENTER | wx.ALL, 5)
        sizer_properties.Add(self.position_text, 0, wx.CENTER | wx.TOP | wx.BOTTOM | wx.LEFT, 5)
        sizer_properties.Add((1,1), 1, wx.CENTER)
        sizer_properties.Add(self.velocity_label, 0, wx.CENTER | wx.ALL, 5)
        sizer_properties.Add(self.velocity_text, 0, wx.CENTER | wx.TOP | wx.BOTTOM, 5)
        sizer_properties.Add((1,1), 1, wx.CENTER)
        sizer_properties.Add(self.effort_label, 0, wx.CENTER | wx.ALL, 5)
        sizer_properties.Add(self.effort_text, 0, wx.CENTER | wx.TOP | wx.BOTTOM | wx.RIGHT, 5)

        # Stack the Slider and the Properties vertically
        sizer_panel = wx.BoxSizer(wx.VERTICAL)
        sizer_panel.Add(sizer_slider, 0, wx.EXPAND)
        sizer_panel.Add(sizer_properties, 0, wx.EXPAND)
        
        self.SetSizer(sizer_panel)
        
        # Register Callbacks
        if self.input_mode:
            self.position_slider.Bind(wx.EVT_SCROLL, self._on_slider_update)
            self.position_text.Bind(wx.EVT_TEXT_ENTER, self._on_position_update)
            self.position_text.Bind(wx.EVT_KILL_FOCUS, self._on_position_update)
            self.velocity_text.Bind(wx.EVT_TEXT_ENTER, self._on_velocity_update)
            self.velocity_text.Bind(wx.EVT_KILL_FOCUS, self._on_velocity_update)
            self.effort_text.Bind(wx.EVT_TEXT_ENTER, self._on_effort_update)
            self.effort_text.Bind(wx.EVT_KILL_FOCUS, self._on_effort_update)

        # Perform an initial update/redraw of the panel
        self.update_panel()

    def update_panel(self):
        if (self.position is not None):
            self.position_slider.SetValue(self.position2slider(self.position))
            self.position_text.SetValue('%.5f' % self.position)
        else:
            self.position_slider.SetValue(self.position2slider(self.min_position))
            self.position_text.SetValue('')
        if (self.velocity is not None):
            self.velocity_text.SetValue('%.5f' % self.velocity)
        else:
            self.velocity_text.SetValue('')
        if (self.effort is not None):
            self.effort_text.SetValue('%.5f' % self.effort)
        else:
            self.effort_text.SetValue('')

    def position2slider(self, position):
        # Convert a process value into a slider increment
        return int(round(self.slider_increments * (position - self.min_position) / (self.max_position - self.min_position)))
    
    def slider2position(self, slider):
        # Convert a process value into a slider increment
        return (float(slider) / self.slider_increments * (self.max_position - self.min_position) + self.min_position) 

    def _on_slider_update(self, event):
        self.position = self.slider2position(self.position_slider.GetValue())
        self.update_panel()

    def _on_position_update(self, event):
        try:
            self.position = float(self.position_text.GetValue())
            self.position = min(max(self.position, self.min_position), self.max_position)
        except:
            self.position = None
        self.update_panel()

    def _on_velocity_update(self, event):
        try:
            self.velocity = float(self.velocity_text.GetValue())
            if self.velocity > self.max_velocity:
                self.velocity = self.max_velocity
            elif self.velocity < -self.max_velocity:
                self.velocity = -self.max_velocity
        except:
            self.velocity = None
        self.update_panel()

    def _on_effort_update(self, event):
        try:
            self.effort = float(self.effort_text.GetValue())
            if self.effort > self.max_effort:
                self.effort = self.max_effort
        except:
            self.effort = None
        self.update_panel()

