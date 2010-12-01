#!/usr/bin/python
#
# Copyright (c) 2010, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#  \author Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)
#  \author (new edits) Jason Okerman (Healthcare Robotics Lab, Georgia Tech.)

'''
    This source file is not currently supported.
    It was made to help generate classifiers and label the clutter table datasets.

'''

#---------------
# Define Import Location Variables

LOC_DATA_LABELING = '/home/jokerman/svn/robot1_data/usr/martin/laser_camera_segmentation/labeling'

#

import roslib; roslib.load_manifest('clutter_segmentation')
from opencv.highgui import cvLoadImage #unneeded?

from PyQt4 import QtGui, QtCore

import opencv.cv as cv
import opencv.highgui as hg

import sys
import shutil #file operations
import os

import label_object, scan_dataset, scans_database

#take scans:
import  canner 
import processor
import configuration
import util as ut
#Formerly: import hrl_lib.util as ut  


class labeling_tool(QtGui.QWidget):
    
    draw_widget = None
    display_mode = 'image'
    
    display_3d_type = 'height'
    
    def __init__(self, path, parent=None):
        
        self.init_in_progress = True
        
        self.path = path
        
        # load configs for taking scans, etc:
        self.config = configuration.configuration(path)
        #create scanner and processor when needed:
        self.scanner = False
        self.processor = False
        #
        
        # load database:
        self.scans_database = scans_database.scans_database()
        self.scans_database.load(path,'database.pkl')
        
        #get first dataset:
        self.current_dataset = self.scans_database.get_dataset(0)

        QtGui.QWidget.__init__(self, parent)
        self.setWindowTitle('labeling tool')
        

        left_layout = QtGui.QVBoxLayout()
        self.draw_widget = draw_widget(self.current_dataset.polygons, self.scans_database.get_path() + '/' + self.current_dataset.image_filename, self)
        
        
        title_layout = QtGui.QHBoxLayout()
        
        take_scan_button = QtGui.QPushButton('Scan')
        take_scan_button.setMaximumWidth(50)
        title_layout.addWidget(take_scan_button)
        self.connect(take_scan_button, QtCore.SIGNAL('clicked()'), self.slot_take_scan )      
        
        take_artag_image_button = QtGui.QPushButton('ARTag')
        take_artag_image_button.setMaximumWidth(50)
        title_layout.addWidget(take_artag_image_button)
        self.connect(take_artag_image_button, QtCore.SIGNAL('clicked()'), self.slot_take_artag_image )              
        
        button = QtGui.QPushButton('Import Img')
        title_layout.addWidget(button)
        self.connect(button, QtCore.SIGNAL('clicked()'), self.slot_import_image )                 
        
        label = QtGui.QLabel("View: ")
        title_layout.addWidget(label)
        self.display_3d_button = QtGui.QPushButton('3D')
        self.display_3d_button.setMaximumWidth(40)
        title_layout.addWidget(self.display_3d_button)
        self.connect(self.display_3d_button, QtCore.SIGNAL('clicked()'), self.slot_display_3d )
        
        combobox = QtGui.QComboBox()
        combobox.addItem("Height", QtCore.QVariant("height"))
        combobox.addItem("Intensities", QtCore.QVariant("intensities"))
        #combobox.addItem("objects", QtCore.QVariant("objects"))
        combobox.addItem("Labels", QtCore.QVariant("labels"))
        combobox.addItem("Classifier range", QtCore.QVariant("range"))
        combobox.addItem("Classifier color", QtCore.QVariant("color"))
        combobox.addItem("Classifier all", QtCore.QVariant("all"))
        combobox.addItem("Classifier all+post", QtCore.QVariant("all_post"))
        combobox.addItem("Baseline algo", QtCore.QVariant("baseline"))
        combobox.addItem("h", QtCore.QVariant("h"))
        combobox.addItem("s", QtCore.QVariant("s"))
        combobox.addItem("v", QtCore.QVariant("v"))
        self.connect(combobox, QtCore.SIGNAL('currentIndexChanged(int)'), self.slot_update_display_3d_type)  
        title_layout.addWidget(combobox)
        self.display_3d_type_combobox = combobox;        
        
        
        self.display_3d_spheres_button = QtGui.QPushButton('3D_Spheres')
        title_layout.addWidget(self.display_3d_spheres_button)
        self.connect(self.display_3d_spheres_button, QtCore.SIGNAL('clicked()'), self.slot_display_3d_spheres )                  
        self.display_intensity_button = QtGui.QPushButton('Intensity')
        self.display_intensity_button.setMaximumWidth(50)
        title_layout.addWidget(self.display_intensity_button)
        self.connect(self.display_intensity_button, QtCore.SIGNAL('clicked()'), self.slot_display_intensity )   
        self.display_features_button = QtGui.QPushButton('Features')
        title_layout.addWidget(self.display_features_button)
        self.display_features_button.setMaximumWidth(50)
        self.connect(self.display_features_button, QtCore.SIGNAL('clicked()'), self.slot_display_features )   
        self.display_labels_button = QtGui.QPushButton('Labels')
        title_layout.addWidget(self.display_labels_button)
        self.display_labels_button.setMaximumWidth(50)
        self.connect(self.display_labels_button, QtCore.SIGNAL('clicked()'), self.slot_display_labels )   
        ###
        self.display_masks_button = QtGui.QPushButton('Masks')
        title_layout.addWidget(self.display_masks_button)
        self.display_masks_button.setMaximumWidth(50)
        self.connect(self.display_masks_button, QtCore.SIGNAL('clicked()'), self.slot_display_masks )   
        ###
        self.display_stats_button = QtGui.QPushButton('Stats')
        title_layout.addWidget(self.display_stats_button)
        self.display_stats_button.setMaximumWidth(50)
        self.connect(self.display_stats_button, QtCore.SIGNAL('clicked()'), self.slot_display_stats )   
        self.display_global_stats_button = QtGui.QPushButton('Global Stats')
        title_layout.addWidget(self.display_global_stats_button)
        self.display_global_stats_button.setMaximumWidth(50)
        self.connect(self.display_global_stats_button, QtCore.SIGNAL('clicked()'), self.slot_display_global_stats )         
        
        self.line_edits = []
    
        self.add_line_edit('Title:',title_layout,'title')


        first_dataset_button = QtGui.QPushButton('<<')
        first_dataset_button.setMaximumWidth(30)
        title_layout.addWidget(first_dataset_button)
        self.connect(first_dataset_button, QtCore.SIGNAL('clicked()'), self.slot_first_dataset )
        prev_dataset_button = QtGui.QPushButton('<')
        prev_dataset_button.setMaximumWidth(30)
        title_layout.addWidget(prev_dataset_button)
        self.connect(prev_dataset_button, QtCore.SIGNAL('clicked()'), self.slot_prev_dataset )
        next_dataset_button = QtGui.QPushButton('>')
        next_dataset_button.setMaximumWidth(30)
        title_layout.addWidget(next_dataset_button)
        self.connect(next_dataset_button, QtCore.SIGNAL('clicked()'), self.slot_next_dataset )
        last_dataset_button = QtGui.QPushButton('>>')
        last_dataset_button.setMaximumWidth(30)
        title_layout.addWidget(last_dataset_button)
        self.connect(last_dataset_button, QtCore.SIGNAL('clicked()'), self.slot_last_dataset )        
        
        save_button = QtGui.QPushButton('Save')
        title_layout.addWidget(save_button)
        save_button.setMaximumWidth(50)
        self.connect(save_button, QtCore.SIGNAL('clicked()'), self.slot_save )
        
        delete_button = QtGui.QPushButton('Delete')
        title_layout.addWidget(delete_button)
        delete_button.setMaximumWidth(50)
        self.connect(delete_button, QtCore.SIGNAL('clicked()'), self.slot_delete )        
        
        
        self.connect(self.draw_widget, QtCore.SIGNAL('sigPolyChanged'), self.slot_update_polygons)
        self.connect(self.draw_widget, QtCore.SIGNAL('sigPolyLabelChanged'), self.slot_update_polygon_label)
        self.connect(self.draw_widget, QtCore.SIGNAL('sigDefineGroundPlane'), self.slot_define_ground_plane)

        left_layout.addLayout(title_layout)
        
        #second row:
        row2_layout = QtGui.QHBoxLayout()
        left_layout.addLayout(row2_layout)
        

        label = QtGui.QLabel("Id:")
        row2_layout.addWidget(label)        
        self.id_label = QtGui.QLabel("")
        row2_layout.addWidget(self.id_label)        
        
        self.add_line_edit('Surface: ID:',row2_layout,'surface_id')
        self.add_line_edit('Height',row2_layout,'surface_height')
        
        
        label = QtGui.QLabel("Type: ")
        row2_layout.addWidget(label)
        combobox = QtGui.QComboBox()
        combobox.addItem("Table Office", QtCore.QVariant("table_office"))
        combobox.addItem("Table Dorm", QtCore.QVariant("table_dorm"))
        combobox.addItem("Table House", QtCore.QVariant("table_house"))
        combobox.addItem("Shelf Office", QtCore.QVariant("shelf_office"))
        combobox.addItem("Shelf Dorm", QtCore.QVariant("shelf_dorm"))
        combobox.addItem("Shelf House", QtCore.QVariant("shelf_house"))
        self.connect(combobox, QtCore.SIGNAL('currentIndexChanged(int)'), self.slot_update_surface_type)  
        row2_layout.addWidget(combobox)
        self.surface_type_combobox = combobox;
        
        self.add_line_edit('Camera: Height:',row2_layout,'camera_height')
        self.add_line_edit('Camera: Angle:',row2_layout,'camera_angle')
        
        #####################################
        #thrid row:
        row3_layout = QtGui.QHBoxLayout()
        left_layout.addLayout(row3_layout)
        
        #checkboxes:
        button = QtGui.QPushButton("&gen'n'save features")
        row3_layout.addWidget(button)
        self.connect(button, QtCore.SIGNAL('clicked()'), self.slot_generate_save_features )        
        
        checkbox = QtGui.QCheckBox('&Training Set')
        row3_layout.addWidget(checkbox)
        self.connect(checkbox, QtCore.SIGNAL('stateChanged(int)'), self.slot_update_training_set)  
        self.checkbox_training_set = checkbox
        
        checkbox = QtGui.QCheckBox('Te&st Set')
        row3_layout.addWidget(checkbox)
        self.connect(checkbox, QtCore.SIGNAL('stateChanged(int)'), self.slot_update_test_set)
        self.checkbox_test_set = checkbox  
        
        checkbox = QtGui.QCheckBox('Labels, Groundp. checked')
        row3_layout.addWidget(checkbox)
        self.connect(checkbox, QtCore.SIGNAL('stateChanged(int)'), self.slot_update_is_labeled)
        self.checkbox_is_labeled = checkbox                         
        
        button = QtGui.QPushButton("Train'n'save Classifiers (training set)")
        row3_layout.addWidget(button)
        self.connect(button, QtCore.SIGNAL('clicked()'), self.slot_train_and_save_Classifiers )
        
        button = QtGui.QPushButton('Test Classifiers (on current)')
        row3_layout.addWidget(button)
        self.connect(button, QtCore.SIGNAL('clicked()'), self.slot_test_Classifiers )
        
        button = QtGui.QPushButton('Test Classifiers (on testset)')
        row3_layout.addWidget(button)
        self.connect(button, QtCore.SIGNAL('clicked()'), self.slot_test_Classifiers_on_testset )        
        
        button = QtGui.QPushButton('Load Classifiers')
        row3_layout.addWidget(button)
        self.connect(button, QtCore.SIGNAL('clicked()'), self.slot_load_Classifiers )
 
#        button = QtGui.QPushButton('Save Classifier')
#        row3_layout.addWidget(button)
#        self.connect(button, QtCore.SIGNAL('clicked()'), self.slot_save_Classifier )                       
        
        #####################################
        
        left_layout.addWidget(self.draw_widget)
        
        
        self.right_layout = QtGui.QVBoxLayout()
        self.right_layout.setAlignment(QtCore.Qt.AlignTop)
        
        self.outer_layout = QtGui.QHBoxLayout()
        self.outer_layout.addLayout(left_layout)
        self.outer_layout.addLayout(self.right_layout)
        

        
        self.polygon_comboboxes = []
        self.add_polygon_combobox()

        self.slot_update_polygons(self.current_dataset.polygons,0)
        
        
        self.setLayout(self.outer_layout)
        
        
        self.resize(900, 700) 
        self.load_values_from_dataset()
        
        self.init_in_progress = False
        
        #at startup, display newest:
        self.slot_last_dataset()
    
    
    def slot_update_training_set(self, checkState):
        if checkState:
            self.current_dataset.is_training_set = True
        else:
            self.current_dataset.is_training_set = False
            
    def slot_update_test_set(self, checkState):
        if checkState:
            self.current_dataset.is_test_set = True
        else:
            self.current_dataset.is_test_set = False
            
    def slot_update_is_labeled(self, checkState):
        if checkState:
            self.current_dataset.is_labeled = True
        else:
            self.current_dataset.is_labeled = False
    
    def closeEvent(self, x):
        print "Exit: saving database..."
        self.slot_save()      
        
    def slot_import_image(self):
        fileName = QtGui.QFileDialog.getOpenFileName(self,"Open Image", self.path, "Image Files (*.png)")
        print "Import image into new dataset:" + fileName
        
        name = ut.formatted_time()
        
        new_dataset = scan_dataset.scan_dataset()
        new_dataset.id = name
        new_dataset.image_filename = 'data/'+name+'_image.png'
        shutil.copy(fileName,self.path+'/'+new_dataset.image_filename)
        
        self.scans_database.add_dataset(new_dataset)

        #proceed to new dataset: 
        while True == self.slot_next_dataset():
            pass        
        
        
    def add_line_edit(self,label, layout, variable):
        label = QtGui.QLabel(label)
        line_edit = QtGui.QLineEdit()
        line_edit.setMinimumWidth(80)
        self.line_edits.append((line_edit,variable))
        layout.addWidget(label)
        layout.addWidget(line_edit)
        self.connect(line_edit, QtCore.SIGNAL('textEdited (const QString&)'), self.slot_line_edit_changed ) 
        return line_edit      
        
    def slot_line_edit_changed(self,text):
        if True == self.init_in_progress:
            return
        
        for (line_edit, variable) in self.line_edits:
            self.current_dataset.dict[variable] = str(line_edit.text())
        
    def slot_next_dataset(self):
        dataset = self.scans_database.get_next_dataset()
        if False != dataset:
            self.current_dataset = dataset
            self.load_values_from_dataset()
            return True
        return False
        
    def slot_prev_dataset(self):
        dataset = self.scans_database.get_prev_dataset()
        if False != dataset:
            self.current_dataset = dataset
            self.load_values_from_dataset()
            return True
        return False
    
    def slot_first_dataset(self):
        dataset = self.scans_database.get_first_dataset()
        if False != dataset:
            self.current_dataset = dataset
            self.load_values_from_dataset()
            return True
        return False  
         
    def slot_last_dataset(self):
        dataset = self.scans_database.get_last_dataset()
        if False != dataset:
            self.current_dataset = dataset
            self.load_values_from_dataset()
            return True
        return False              
            
            
    def load_values_from_dataset(self):
        self.init_in_progress = True
        
        self.id_label.setText(self.current_dataset.id)
        
        for (line_edit, variable) in self.line_edits:
            line_edit.setText(self.current_dataset.dict[variable])
            
        for index, box in enumerate(self.polygon_comboboxes):
            if index < len(self.current_dataset.polygons):
                print str(index) + " load label:" + self.current_dataset.polygons[index].get_label()
                boxindex = box.findData(QtCore.QVariant(self.current_dataset.polygons[index].get_label()))
                box.setCurrentIndex(boxindex)
            else: #set default to first:
                box.setCurrentIndex(0)
                
                
        box = self.surface_type_combobox
        boxindex = box.findData(QtCore.QVariant(self.current_dataset.surface_type))
        box.setCurrentIndex(boxindex)
        
        print self.current_dataset.is_training_set
        if self.current_dataset.is_training_set:
            self.checkbox_training_set.setCheckState(QtCore.Qt.Checked)
        else:
            self.checkbox_training_set.setCheckState(QtCore.Qt.Unchecked)
            
        if self.current_dataset.is_test_set:
            self.checkbox_test_set.setCheckState(QtCore.Qt.Checked)
        else:
            self.checkbox_test_set.setCheckState(QtCore.Qt.Unchecked)
        
        if self.current_dataset.is_labeled:
            self.checkbox_is_labeled.setCheckState(QtCore.Qt.Checked)
        else:
            self.checkbox_is_labeled.setCheckState(QtCore.Qt.Unchecked)
        
        #hide button if there is no 3d data:
        print self.current_dataset.scan_filename
        if '' == self.current_dataset.scan_filename:
            self.display_3d_button.setEnabled(False)
            self.display_3d_spheres_button.setEnabled(False)
            self.display_intensity_button.setEnabled(False)
        else:
            self.display_3d_button.setEnabled(True)
            self.display_3d_spheres_button.setEnabled(True)
            self.display_intensity_button.setEnabled(True)
    
        self.display_mode = 'image'
        self.draw_widget.set_polygons(self.current_dataset.polygons)
        self.draw_widget.set_image(self.scans_database.get_path() + '/' + self.current_dataset.image_filename)
        
        self.init_in_progress = False
            
    def slot_take_artag_image(self):
        if False == self.scanner:
            self.scanner = scanner.scanner(self.config)
        if False == self.processor:
            self.processor = processor.processor(self.config)
        
        img = self.scanner.take_artag_image()
        self.current_dataset.image_artag_filename = self.scanner.save_artag_image(self.current_dataset.id)
        
        self.slot_save() #save for consistency with files
        
        if self.processor.read_artag(img).any():
            print "SUCCESS in reading ARTag"
        else:
            print "FAILURE in reading ARTag - try again!"
        
        
            
    def slot_take_scan(self):
        
        #save database, let scanner add dataset, reload it then
        self.slot_save()
        
        if False == self.scanner:
            self.scanner = scanner.scanner(self.config)
        if False == self.processor:
            self.processor = processor.processor(self.config)
        
        name = ut.formatted_time()
        self.scanner.capture_and_save(name)
        #self.processor.load_raw_data(name)
        #self.processor.load_metadata(name)
        #self.processor.process_raw_data()
        #self.processor.save_mapped_image(name)
        #self.processor.display_all_data()
        print 'scan ' + name + ' taken'
        
        self.scans_database.load(self.path,'database.pkl')
        
        #proceed to new scan: 
        while True == self.slot_next_dataset():
            pass
        
    def slot_display_intensity(self):
        if self.display_mode != 'intensities':
            if False == self.processor:
                self.processor = processor.processor(self.config)
                
            #reset ground plane:
            self.current_dataset.ground_plane_normal = ''
            self.current_dataset.ground_plane_three_points = ''
            self.slot_save()    
                
            
            self.processor.load_data(self.current_dataset.id)
            self.processor.process_intensities()
            filename = self.processor.save_intensity_image(self.current_dataset.id)
            
            #self.processor.display_intensities()
            
            self.display_mode = 'intensities'
            self.draw_widget.set_image(filename)
        else:
            #display normal image
            
            self.display_mode = 'image'
            self.draw_widget.set_image(self.scans_database.get_path() + '/' + self.current_dataset.image_filename)
            
    def slot_display_features(self):
        if self.display_mode != 'features':
            if False == self.processor:
                self.processor = processor.processor(self.config)
                
            self.processor.load_data(self.current_dataset.id)
            self.processor.process_intensities()
            filename = self.processor.save_intensity_image(self.current_dataset.id)
            self.display_mode = 'features'
            self.draw_widget.set_image(filename)
            
        else:
            #display normal image
            self.display_mode = 'image'
            self.draw_widget.set_image(self.scans_database.get_path() + '/' + self.current_dataset.image_filename)
                 
            
    def slot_display_labels(self):
        if self.display_mode != 'labels':
            if False == self.processor:
                self.processor = processor.processor(self.config)
                
            self.processor.load_data(self.current_dataset.id)
            self.processor.process_labels(self.display_3d_type)
            filename = self.processor.save_labels_image(self.display_3d_type)
            
            self.draw_widget.set_image(filename)
            self.display_mode = 'labels'
        else:
            #display normal image
            self.draw_widget.set_image(self.scans_database.get_path() + '/' + self.current_dataset.image_filename)
            self.display_mode = 'image'       
    ###        
    def slot_display_masks(self):       
        if False == self.processor:
            self.processor = processor.processor(self.config)    
        self.processor.load_data(self.current_dataset.id)
        if self.display_mode != 'labels':     
            self.processor.process_masks(self.display_3d_type)
            self.display_mode = 'labels'
            filename = self.processor.save_masks_image(self.display_3d_type) #saves pic in results
        else:
            self.processor.process_masks(self.display_3d_type, True) #show clutter mask NOT placement mask
            self.display_mode = 'image'
            filename = self.processor.save_masks_image(self.display_3d_type, True) #saves pic in results    
        self.draw_widget.set_image(filename) #loads picture saved previously
    ###               
            
    def slot_display_stats(self):
        if False == self.processor:
            self.processor = processor.processor(self.config)
            
        self.processor.load_data(self.current_dataset.id)
        self.processor.display_stats()
         
    def slot_display_global_stats(self):
        if False == self.processor:
            self.processor = processor.processor(self.config)
            
        self.processor.load_data(self.current_dataset.id)
        self.processor.display_stats(True)      
        
        
    def slot_display_3d_spheres(self):  
        self.slot_display_3d(True)   
        
    def slot_display_3d(self, spheres = False):
        print 'Inside slot_display_3d'
        if False == self.processor:       
            self.processor = processor.processor(self.config)
            
        #save data first so the processor can load it:
        print 'Before slot_save'
        self.slot_save()    
            
        print 'Before load_data'
        self.processor.load_data(self.current_dataset.id)
        #self.processor.create_polygon_images()
        print 'Before process_raw_data'
        self.processor.process_raw_data()
        #pc.save_mapped_image(name)
        print 'Before display_3d'
        self.processor.display_3d(self.display_3d_type, spheres)
        print 'After display_3d'
        
    def slot_train_and_save_Classifiers(self):

        if False == self.processor:       
            self.processor = processor.processor(self.config)
            
        #save data first so the processor can load it:
        self.slot_save()    
            
        self.processor.load_data(self.current_dataset.id)
        self.processor.train_and_save_Classifiers()   
        
    def slot_generate_save_features(self):

        if False == self.processor:       
            self.processor = processor.processor(self.config)
            
        #save data first so the processor can load it:
        self.slot_save()    
            
        self.processor.load_data(self.current_dataset.id)
        self.processor.generate_save_features()   
        
        
    def slot_test_Classifiers(self):
        if False == self.processor:       
            self.processor = processor.processor(self.config)
        self.slot_save()    #save data first so the processor can load it:
        self.processor.load_data(self.current_dataset.id)
        self.processor.train_and_save_Classifiers()
        self.processor.test_Classifiers()   
        
    def slot_test_Classifiers_on_testset(self):
        if False == self.processor:       
            self.processor = processor.processor(self.config)
        self.slot_save()    #save data first so the processor can load it:
        self.processor.load_data(self.current_dataset.id)
        self.processor.train_and_save_Classifiers()
        self.processor.test_classifiers_on_testset()          
      
    def slot_load_Classifiers(self):
        if False == self.processor:       
            self.processor = processor.processor(self.config)
        self.processor.load_Classifiers()   
        
    def slot_save_Classifier(self):
        if False == self.processor:       
            print 'ERROR: no processor object exists -> no Classifier to save!'
            return
        self.processor.save_Classifier()      
            
    def add_polygon_combobox(self):
        combobox = QtGui.QComboBox()
        combobox.addItem("Object", QtCore.QVariant("object"))
        combobox.addItem("Surface", QtCore.QVariant("surface"))
        combobox.addItem("Region of Interest (ROI)", QtCore.QVariant("roi"))
        combobox.addItem("Background", QtCore.QVariant("background"))
        combobox.addItem("Visible Surface-Edge", QtCore.QVariant("edge"))
        combobox.addItem("Wall-Surface-Edge", QtCore.QVariant("edge_up"))
        combobox.addItem("Downward-Surface-Edge", QtCore.QVariant("edge_down"))
        combobox.setCurrentIndex(0)

        self.connect(combobox, QtCore.SIGNAL('currentIndexChanged(int)'), self.slot_update_polygon_labels)
        self.polygon_comboboxes.append(combobox)
        self.right_layout.addWidget(combobox, QtCore.Qt.AlignTop)
        self.slot_update_polygon_labels()
        
        
    def slot_delete(self):
        #delete scan-files:
        if  os.path.isfile(self.current_dataset.scan_filename):
            os.remove(self.path + '/' + self.current_dataset.scan_filename);
        if os.path.isfile(self.current_dataset.image_filename):
            os.remove(self.path + '/' + self.current_dataset.image_filename);
        if os.path.isfile(self.current_dataset.image_artag_filename):
            os.remove(self.path + '/' + self.current_dataset.image_artag_filename);            
        #delete metadata
        self.current_dataset  = self.scans_database.delete_current_dataset()
        self.load_values_from_dataset()
        self.slot_save() #save for consistency with files
        
    def slot_save(self):
        self.scans_database.save()
        
    def slot_update_surface_type(self):
        if True == self.init_in_progress:
            return   
        box = self.surface_type_combobox
        self.current_dataset.surface_type = str(box.itemData(box.currentIndex()).toString())
        
    def slot_update_display_3d_type(self):
        if True == self.init_in_progress:
            return          
        box = self.display_3d_type_combobox
        self.display_3d_type = str(box.itemData(box.currentIndex()).toString())  
        
    def slot_update_polygon_label(self, index, label):
        if True == self.init_in_progress:
            return       
        
        box = self.polygon_comboboxes[index]
        boxindex = box.findData(QtCore.QVariant(label))
        box.setCurrentIndex(boxindex)  
        
        self.draw_widget.update()    
        
    def slot_update_polygon_labels(self):
        if True == self.init_in_progress:
            return       
        
        for index, box in enumerate(self.polygon_comboboxes):
            if index < len(self.current_dataset.polygons):
                self.current_dataset.polygons[index].set_label(str(box.itemData(box.currentIndex()).toString()))
                print str(index) + " xx " + str(box.itemData(box.currentIndex()).toString()) 
                
        self.draw_widget.update()         
    
    def slot_update_polygons(self, polygons, current_index):

        while len(self.polygon_comboboxes) < len(polygons):
            self.add_polygon_combobox()

        #self.polygon_comboboxes[self.current_polygon_index].x()
        for index, box in enumerate(self.polygon_comboboxes):
            if index < len(polygons):
                self.polygon_comboboxes[index].show()
            else:
                self.polygon_comboboxes[index].hide() 
        self.update()
        
    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)
        
        x = self.polygon_comboboxes[self.draw_widget.get_current_polygon_index()].x()    
        y = self.polygon_comboboxes[self.draw_widget.get_current_polygon_index()].y() 
        color = QtGui.QColor(255,0,0)
        painter.setPen(color)
        painter.setBrush(color)
        painter.drawEllipse(QtCore.QRectF(x-8,y+8,6,6))
            
        painter.end()
        
    def get_display_mode(self):
        return self.display_mode
    
    def slot_define_ground_plane(self, ground_plane_points):
        #assumes that intensity image is loaded in processor!
        (self.current_dataset.ground_plane_normal, self.current_dataset.ground_plane_three_points) = self.processor.get_3d_plane_normal(ground_plane_points)
        self.slot_display_intensity() #switch back to image mode

class draw_widget(QtGui.QLabel):
    
    
    ground_plane_points = []
    
    def __init__(self,polygons, image_filename, parent=None):
        QtGui.QWidget.__init__(self, parent)

        self.scaleFactor = False #init is done later

        self.setBackgroundRole(QtGui.QPalette.Base)
        #self.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        self.setSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        self.setScaledContents(True)


        self.set_polygons(polygons)
        self.set_image(image_filename)
        
        self.setScaleFactor(0.8)
       
    def setScaleFactor(self, f):
        self.scaleFactor = f
        self.updateImageSize()
        
    def updateImageSize(self):
        if self.parent().get_display_mode() == 'intensities' or self.parent().get_display_mode() == 'features':
            self.scaleFactor = 1 
        else:
            self.scaleFactor = 0.8      
            self.parent().resize(900, 700) 
        
        self.setMinimumHeight(self.image.height() * self.scaleFactor)
        self.setMinimumWidth(self.image.width() * self.scaleFactor)
        self.setMaximumHeight(self.image.height() * self.scaleFactor)
        self.setMaximumWidth(self.image.width() * self.scaleFactor)
        
        pixmap = QtGui.QPixmap.fromImage(self.image)
        self.resize(self.scaleFactor * pixmap.size());
        self.setPixmap(pixmap);
        
        
    def set_polygons(self, polygons):
        self.polygons = polygons
        self.current_polygon_index = 0
        self.update()
        self.emit(QtCore.SIGNAL("sigPolyChanged"), self.polygons, self.current_polygon_index)         
        
    def set_image(self, filename):
        print filename
        if os.path.isfile(filename):
            self.image = QtGui.QImage(filename)
        else:
            self.image = QtGui.QImage('noimage.png')
        
        self.updateImageSize()
        self.update()
        
    def paintEvent(self, event):
        # draw image as label-pixmap
        QtGui.QLabel.paintEvent(self,event) 
        painter = QtGui.QPainter()
        painter.begin(self)
        if self.parent().get_display_mode() == 'image' or self.parent().get_display_mode() == 'labels':
            color = QtGui.QColor(0,0,255)
            color_surface = QtGui.QColor(0,255,0)
            color_roi = QtGui.QColor(255,255,255)
            color_edge = QtGui.QColor(255,255,0)
            color_edge_up = QtGui.QColor(255,255,255)
            color_edge_down = QtGui.QColor(255,150,255)
            color_background = QtGui.QColor(255,0,255)
            color_current = QtGui.QColor(255,0,0)
    
            for index, polygon in enumerate(self.polygons):
                
                last_point = (-1,-1)
                first = True;
                if self.current_polygon_index != index or self.parent().get_display_mode() != 'image':
                    if polygon.get_label() == 'surface':
                        painter.setPen(color_surface)
                    elif polygon.get_label() == 'roi':
                        painter.setPen(color_roi)                        
                    elif polygon.get_label() == 'edge':
                        painter.setPen(color_edge)
                    elif polygon.get_label() == 'edge_up':
                        painter.setPen(color_edge_up)
                    elif polygon.get_label() == 'edge_down':
                        painter.setPen(color_edge_down)
                    elif polygon.get_label() == 'background':
                        painter.setPen(color_background)                    
                    else:
                        painter.setPen(color)
                else:
                    painter.setPen(color_current)
    
                for point in polygon.get_points():
                    if False == first:
                        painter.drawLine(QtCore.QPointF(point[0],point[1]) * self.scaleFactor, QtCore.QPointF(last_point[0],last_point[1]) * self.scaleFactor)
                    last_point = point
                    first = False
                    
                
                if (self.parent().get_display_mode() != 'image'  or self.current_polygon_index != index ) and polygon.get_type() == 'polygon' and len(polygon.get_points()) :
                    painter.drawLine(QtCore.QPointF(last_point[0],last_point[1]) * self.scaleFactor, QtCore.QPointF(polygon.get_points()[0][0],polygon.get_points()[0][1]) * self.scaleFactor)
                else:
                    for point in polygon.get_points():
                        painter.drawEllipse(QtCore.QRectF(point[0] * self.scaleFactor-3,point[1] * self.scaleFactor-3,6,6))
                
        elif self.parent().get_display_mode() == 'intensities':
            color = QtGui.QColor(255,0,255)
            painter.setPen(color)
            for point in self.ground_plane_points:
                painter.drawEllipse(QtCore.QRectF(point[0] * self.scaleFactor-3,point[1] * self.scaleFactor-3,6,6))
        painter.end()

    
    def mousePressEvent(self,event):
        
        if self.hasFocus():
            if self.parent().get_display_mode() == 'image':
                if event.button() == QtCore.Qt.LeftButton:
                    #print 'coords:', x,'  ',y
                    point = (event.x() / self.scaleFactor, event.y() / self.scaleFactor)
                    self.polygons[self.current_polygon_index].add_point(point)
                    self.update()
                    self.emit(QtCore.SIGNAL("sigPolyChanged"), self.polygons, self.current_polygon_index)
                if  event.button() == QtCore.Qt.RightButton:
                    if False == self.polygons[self.current_polygon_index].is_empty():
                        self.polygons[self.current_polygon_index].delete_last_point()
                        self.update()
                        self.emit(QtCore.SIGNAL("sigPolyChanged"), self.polygons, self.current_polygon_index)
            elif self.parent().get_display_mode() == 'intensities':
               
                point = (event.x() / self.scaleFactor, event.y() / self.scaleFactor)
                print 'point:', point
                if True == self.parent().processor.check_3d_plane_point(point):
                    self.ground_plane_points.append(point)
                    if len(self.ground_plane_points) < 3:
                        self.update()
                    else:
                        self.emit(QtCore.SIGNAL("sigDefineGroundPlane"), self.ground_plane_points)
                        self.ground_plane_points = []
                        
                     
            elif self.parent().get_display_mode() == 'features':   
                point = (event.x() / self.scaleFactor, event.y() / self.scaleFactor)
                if True == self.parent().processor.check_3d_plane_point(point):
                    print 'point:', point
                    point3d = self.parent().processor.get_3d_point(point)
                    print 'point3d',point3d
                    
                    index = self.parent().processor.get_3d_point_index_in_unrotated(point3d)
                    self.parent().processor.load_data(self.parent().current_dataset.id)
                    self.parent().processor.process_raw_data()
                    self.parent().processor.features.prepare([index])
                    self.parent().processor.feature_type = 'gaussian_histograms'
                    fv = self.parent().processor.features.get_featurevector(index,0)
                    print 'fv',fv
                    self.parent().processor.display_featurevector(fv)
                    
                    #reload intensity data for next click
                    self.parent().processor.load_data(self.parent().current_dataset.id)
                    self.parent().processor.process_intensities()
                    
                    #print 'fv:', self.parent().processor.get_point_featurevector(index, self.parent().processor.pts3d_int)
                    #print 'WARNING: THIS IS NOT WORKING YET BECAUSE OF MISSING INTENSITY INDEX MAPPING FOR GRAZEEFFCT REMOVED PTS'
        else:
            self.setFocus()

    def mouseDoubleClickEvent(self,event):
        if self.parent().get_display_mode() == 'image':
            if event.button() == QtCore.Qt.LeftButton:
                self.start_new_polygon()
                self.update()
                self.emit(QtCore.SIGNAL("sigPolyChanged"), self.polygons, self.current_polygon_index)
                
    def start_new_polygon(self):
        if False == self.polygons[self.current_polygon_index].is_empty():
           # if self.current_polygon_index == len(self.polygons) - 1:
            self.polygons.append(label_object.label_object()) #last one, append new
            self.current_polygon_index = len(self.polygons) - 1
            print "new poly index: ", self.current_polygon_index
            
    def delete_empty_polygon(self):
        if True == self.polygons[self.current_polygon_index].is_empty():
            #and it isn't the only one:
            if 1 != len(self.polygons):
                del self.polygons[self.current_polygon_index]
                if 0 != self.current_polygon_index:
                    self.current_polygon_index -= 1
                print "new poly index: ", self.current_polygon_index
                return True
        return False
    
    def keyPressEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key_Right:
            print 'right'
            if self.current_polygon_index < len(self.polygons) - 1:
                self.delete_empty_polygon()
                self.current_polygon_index += 1
                print "czurrent poly index: ", self.current_polygon_index
            else:
                self.start_new_polygon()
                self.parent().slot_update_polygon_labels()
            self.update()
            self.emit(QtCore.SIGNAL("sigPolyChanged"), self.polygons, self.current_polygon_index)
        elif key == QtCore.Qt.Key_Left:
            print 'left'
            if self.current_polygon_index > 0:
                if False == self.delete_empty_polygon():
                    self.current_polygon_index -= 1
                print "current poly index: ", self.current_polygon_index
            self.update()
            self.emit(QtCore.SIGNAL("sigPolyChanged"), self.polygons, self.current_polygon_index)
        elif key == QtCore.Qt.Key_O:
            print 'o'
            self.emit(QtCore.SIGNAL("sigPolyLabelChanged"), self.current_polygon_index, 'object')      
        elif key == QtCore.Qt.Key_S:
            print 's'
            self.emit(QtCore.SIGNAL("sigPolyLabelChanged"), self.current_polygon_index, 'surface')    
        elif key == QtCore.Qt.Key_R:
            print 'r'
            self.emit(QtCore.SIGNAL("sigPolyLabelChanged"), self.current_polygon_index, 'roi')                
        elif key == QtCore.Qt.Key_B:
            print 'b'
            self.emit(QtCore.SIGNAL("sigPolyLabelChanged"), self.current_polygon_index, 'background')    
        elif key == QtCore.Qt.Key_E:
            print 'e'
            self.emit(QtCore.SIGNAL("sigPolyLabelChanged"), self.current_polygon_index, 'edge')  
        elif key == QtCore.Qt.Key_U:
            print 'u'
            self.emit(QtCore.SIGNAL("sigPolyLabelChanged"), self.current_polygon_index, 'edge_up') 
        elif key == QtCore.Qt.Key_D:
            print 'd'
            self.emit(QtCore.SIGNAL("sigPolyLabelChanged"), self.current_polygon_index, 'edge_down')   
        elif key == QtCore.Qt.Key_Plus:
            print '+'
            self.setScaleFactor(self.scaleFactor * 1.25)
            self.update()     
        elif key == QtCore.Qt.Key_Minus:
            print '-'
            self.setScaleFactor(self.scaleFactor * 0.8)
            self.update()                                                               
        else:
            QtGui.QWidget.keyPressEvent(self, event)
    
    def get_polygons(self):
        return self.polygons
    
    def get_current_polygon_index(self):
        return self.current_polygon_index
    
    
    
    
    
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    labeling_tool = labeling_tool(LOC_DATA_LABELING);#

    labeling_tool.show()
    sys.exit(app.exec_())
