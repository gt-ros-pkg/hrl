#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## @author Hai Nguyen/hai@gatech.edu
from pkg import *
from std_msgs.msg import String
import wx, time, sys

def key_to_command(key):
    ret = None

    if key == u'g':
        ret = 'debug'

    if key == u'd':
        ret = 'display'

    if key == u'v':
        ret = 'verbose'

    if key == u' ':
        ret = 'rebuild'

    if key == u'p':
        ret = 'positive'

    if key == u'c':
        ret = 'clear'

    return ret

class KeyHandler(wx.Window):
    def __init__(self, parent):
        wx.Window.__init__(self, parent, -1, style=wx.WANTS_CHARS, name="sink")
        self.Bind(wx.EVT_KEY_DOWN, self.on_key_down)

        self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)
        self.Bind(wx.EVT_LEFT_UP, self.on_left_up)
        self.Bind(wx.EVT_LEFT_DCLICK, self.on_left_double_click)

        self.Bind(wx.EVT_RIGHT_DOWN, self.on_right_down)
        self.Bind(wx.EVT_RIGHT_UP, self.on_right_up)
        self.Bind(wx.EVT_RIGHT_DCLICK, self.on_right_double_click)

        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SET_FOCUS, self.on_set_focus)
        self.Bind(wx.EVT_KILL_FOCUS, self.on_kill_focus)
        self.Bind(wx.EVT_SIZE, self.on_size)
        self.Bind(wx.EVT_UPDATE_UI, self.on_size)
        wx.UpdateUIEvent.SetUpdateInterval(500)

        self.mouse_click_pub = rospy.Publisher(MOUSE_CLICK_TOPIC, String).publish
        self.mouse_r_click_pub = rospy.Publisher(MOUSE_R_CLICK_TOPIC, String).publish
        self.laser_mode_pub  = rospy.Publisher(LASER_MODE_TOPIC, String).publish
        self.mouse_double_click_pub = rospy.Publisher(MOUSE_DOUBLE_CLICK_TOPIC, String).publish
        self.mouse_r_double_click_pub = rospy.Publisher(MOUSE_R_DOUBLE_CLICK_TOPIC, String).publish
        rospy.init_node('user_interface_node')
        self.SetBackgroundColour(wx.BLACK)
        self.focus = False
        self.text = ''
        self.time = time.time()
        self.interval = 1.0

    def set_text(self, text):
        self.text = text
        self.time = time.time()

    def on_size(self, evt):
        self.Refresh()

    def on_set_focus(self, evt):
        self.focus = True
        self.Refresh()

    def on_kill_focus(self, evt):
        self.focus = False
        self.Refresh()

    def on_left_double_click(self, evt):
        self.mouse_double_click_pub('True')
        self.set_text('DOUBLE CLICKED')

    def on_left_down(self, evt):
        self.mouse_click_pub('True')
        self.set_text('cli....')
        self.Refresh()

    def on_left_up(self, evt):
        self.mouse_click_pub('False')
        self.set_text('...cked')
        self.Refresh()

    def on_right_double_click(self, evt):
        self.mouse_r_double_click_pub('True')
        self.set_text('R DOUBLE CLICKED')

    def on_right_down(self, evt):
        self.mouse_r_click_pub('True')
        self.set_text('right cli....')
        self.Refresh()

    def on_right_up(self, evt):
        self.mouse_r_click_pub('False')
        self.set_text('...cked')
        self.Refresh()

    def on_key_down(self, evt):
        c = unichr(evt.GetRawKeyCode())
        command = key_to_command(c)
        if command is not None:
            #print 'publishing', command
            self.laser_mode_pub(command)
            self.set_text(command)
        else:
            self.set_text(unichr(evt.GetRawKeyCode()))
        self.Refresh()

    def on_paint(self, evt):
        dc   = wx.PaintDC(self)
        font = dc.GetFont()
        font.SetPointSize(20)
        dc.SetFont(font)
        rect = self.GetClientRect()

        if self.focus:
            dc.SetTextForeground(wx.GREEN)
            dc.DrawLabel("Focused.", rect, wx.ALIGN_CENTER)
        else:
            dc.SetTextForeground(wx.RED)
            dc.DrawLabel("Got no focus.", rect, wx.ALIGN_CENTER)

        dc.SetTextForeground(wx.WHITE)
        dc.DrawLabel('g - debug\nd - display\nv - verbose\np - positive\nc - clear\nspace - rebuild', rect, wx.ALIGN_LEFT | wx.ALIGN_TOP)

        if (time.time() - self.time) < self.interval:
            dc.SetFont(wx.Font(20, wx.MODERN, wx.NORMAL, wx.NORMAL))
            dc.SetTextForeground(wx.WHITE)
            dc.DrawLabel(self.text, rect, wx.ALIGN_BOTTOM | wx.ALIGN_CENTER)

app   = wx.PySimpleApp()
frame = wx.Frame(None, wx.ID_ANY, name='Clickable World GUI', size=(800,600))
win   = KeyHandler(frame) 
frame.Show(True)
win.SetFocus()
#rospy.ready(sys.argv[0])
app.MainLoop()


