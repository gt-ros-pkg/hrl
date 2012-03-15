import numpy as np
import matplotlib.pyplot as pb
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import hrl_lib.util as ut

LIGHTBLUE = '#33CCFF'
DARKBLUE = '#008AB8'
LIME = '#CCFF33'
GREY = '#E0E0E0'

##
# gets a random color string
# @return a random color string of format #XXXXXX
def random_color():
    r = '%02X'%np.random.randint(0, 255)
    g = '%02X'%np.random.randint(0, 255)
    b = '%02X'%np.random.randint(0, 255)
    c = '#' + r + g + b
    return c

##
# Selection rectangle for TimeSeriesDisplay
class SelectRect:
    def __init__(self, x_start, x_end, axes_objects, color='#FFCC33', alpha=.3):
        self.rects = []
        for axes_object in axes_objects:
            trans = transforms.blended_transform_factory(axes_object.transData, 
                                                        axes_object.transAxes)
            rect = patches.Rectangle((x_start, 0), width=(x_end-x_start), 
                                        height=1, transform=trans, color=color, alpha=alpha)
            axes_object.add_patch(rect)
            self.rects.append(rect)

        self.x_start = x_start
        self.x_end = x_end
        self.name = None

    def set_color(self, color):
        for r in self.rects:
            r.set_color(color)

    def set_start(self, x):
        self.x_start = x
        for rect in self.rects:
            rect.set_x(self.x_start)
            rect.set_width(self.x_end - self.x_start)

    def set_end(self, x):
        self.x_end = x
        for rect in self.rects:
            rect.set_width(self.x_end - self.x_start)

    def get_start(self):
        return self.x_start

    def get_end(self):
        return self.x_end

    def set_name(self, n):
        self.name = n

    def get_name(self):
        return self.name

##
# plot lists of time series
# allow saving & annotation
class TimeSeriesDisplay:
    #, lists_of_time_series, rows, columns
    def __init__(self, list_of_time_series, labels, limits=None, color='b', save='segments.pkl', name="", marker=''):
        #Display
        self.figure = pb.figure()
        self.figure.set_facecolor('w')
        self.axes = []

        #pdb.set_trace()
        time_idxes = [t for t, d in list_of_time_series]
        min_time = np.min(np.concatenate(time_idxes))
        max_time = np.max(np.concatenate(time_idxes))

        for idx, time_series in enumerate(list_of_time_series):
            tlist, ylist = time_series
            #print len(tlist), len(ylist)
            axis = pb.subplot(len(list_of_time_series), 1, idx+1)
            print labels[idx], len(tlist), len(ylist)
            lines = pb.plot(tlist, ylist, marker+'-', color=color)
            pb.xlim(min_time, max_time)
            if limits != None and limits[idx] != None:
                ymin, ymax = limits[idx] 
                pb.ylim(ymin, ymax)
            pb.ylabel(labels[idx])
            #pdb.set_trace()
            self.axes.append(axis)

        #Interaction
        pb.suptitle(name)
        self.figure.canvas.mpl_connect('button_press_event', self.button_press_handler)
        self.figure.canvas.mpl_connect('key_press_event', self.key_press_handler)

        self.save_filename = save
        self.mode = 'EDIT' #or INTERACT
        self.state = 'START'
        self.rects = []
        self.x = None
        self.active_idx = None
        self.ACTIVE_COLOR = LIME
        self.INACTIVE_COLOR = GREY

        self.load_state()

    def _set_active(self, idx):
        for r in self.rects:
            r.set_color(self.INACTIVE_COLOR)

        self.active_idx = idx % len(self.rects)
        self.rects[self.active_idx].set_color(self.ACTIVE_COLOR)
        print 'TimeSeriesDisplay._set_active(): selected', self.rects[self.active_idx].get_name()

    def _inactivate_current_rect(self):
        if self.active_idx != None:
            self.rects[self.active_idx].set_color(self.INACTIVE_COLOR)

    def key_press_handler(self, event):
        #print event.key
        if self.mode == 'EDIT':
            if event.key == 'escape':
                self.state = 'START'
                self._inactivate_current_rect()

            elif event.key == 'right':
                if self.active_idx != None:
                    self._set_active(self.active_idx + 1)
                    self.state = 'TWO_POINTS'

            elif event.key == 'left':
                if self.active_idx != None:
                    self._set_active(self.active_idx - 1)
                    self.state = 'TWO_POINTS'

            elif event.key == ' ':
                if self.active_idx != None:
                    print 'Current name is', self.rects[self.active_idx].get_name(), '. Enter a new name:'
                    n = raw_input()
                    self.rects[self.active_idx].set_name(n)

            elif event.key == 'w':
                if len(self.rects) > 0:
                    self.save_state()
        elif event.key == 'e':
            if self.mode == 'EDIT':
                self.mode = 'INTERACT'
            else:
                self.mode = 'EDIT'
            print self.mode

        self.figure.canvas.draw()
        #print event.key

    def load_state(self):
        try:
            rects = ut.load_pickle(self.save_filename)
            srects = []
            for r in rects:
                start, end, name = r
                srect = SelectRect(start, end, self.axes)
                srect.set_name(name)
                srects.append(srect)
            self.rects = srects
            self._set_active(0)
            self.state = 'TWO_POINTS'
            self.figure.canvas.draw()
        except IOError, e:
            print 'INFO: was not able to load', self.save_filename

    def save_state(self):
        segments = []
        for r in self.rects:
            segments.append([r.get_start(), r.get_end(), r.get_name()])
        ut.save_pickle(segments, self.save_filename)
        print 'saved state to', self.save_filename

    def button_press_handler(self, event):
        #print 'in state', self.state
        #print event.button
        #print 'dfd'
        if self.mode == 'EDIT':
            #print 'dfd', event.button, event.xdata.__class__, self.state
            if event.button == 1 and event.xdata != None:
                if self.state == 'ONE_POINT': 
                    sm = min(self.x, event.xdata)
                    sx = max(self.x, event.xdata)
                    active_rect = SelectRect(sm, sx, self.axes)
                    self.rects.append(active_rect)
                    self._set_active(len(self.rects) - 1)
                    self.state = 'TWO_POINTS'

                elif self.state == 'TWO_POINTS':
                    if event.xdata > self.rects[self.active_idx].get_end():
                        self.rects[self.active_idx].set_end(event.xdata)
                    elif event.xdata < self.rects[self.active_idx].get_start():
                        self.rects[self.active_idx].set_start(event.xdata)
                    else:
                        dend   = abs(event.xdata - self.rects[self.active_idx].get_end())
                        dstart = abs(event.xdata - self.rects[self.active_idx].get_start())
                        if dstart < dend:
                            self.rects[self.active_idx].set_start(event.xdata)
                        else:
                            self.rects[self.active_idx].set_end(event.xdata)

            elif event.button == 3 and event.xdata != None:
                if self.state == 'START':
                    self.x = event.xdata
                    self.state = 'ONE_POINT'

                elif self.state == 'TWO_POINTS':
                    self.x = event.xdata
                    self.state = 'ONE_POINT'
                    self._inactivate_current_rect()
        
        #print 'out state', self.state
        self.figure.canvas.draw()
