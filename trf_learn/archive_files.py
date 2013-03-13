#! /usr/bin/python

import glob
import sys
import os
import time
import shutil

root = sys.argv[1]
name = sys.argv[2]

pca_data_files = glob.glob("%s/.ros/*_pca_data.pkl" % root)
date_string = time.strftime('%Y_%d_%m__%I_%M')
dest_dir = '/removable/haidai/thesis_data/%s_%s' % (date_string, name)
os.mkdir(dest_dir)

for f in pca_data_files:
    fname = os.path.split(f)[1]
    dest_name = os.path.join(dest_dir, fname)
    print 'copying', f, 'to', dest_name
    shutil.copy2(f, dest_name)

critical_pkls = ['action_database.pkl',  'tag_database.pkl',  'trf_learn_db.pkl']
for cpkl in critical_pkls:
    source_name = os.path.join(root, cpkl)
    dest_name = os.path.join(dest_dir, cpkl)
    print 'copying', source_name, 'to', dest_name
    shutil.copy2(source_name, dest_name)

action_dir_files = glob.glob("%s/.ros/*_action*" % root)
for adf in action_dir_files:
    adf_name = os.path.split(adf)[1]
    adf_path = os.path.join(dest_dir, adf_name)
    print 'copying', adf, 'to', adf_path
    if os.path.isdir(adf):
        shutil.copytree(adf, adf_path)
    else:
        shutil.copy2(adf, adf_path)
