#!/usr/bin/env python
import camera_setup_lib as csl
import camera_config as cc

##
# Createss a dictionary mapping camera UUID to OpenCV IDs
def camera_ids():
	uidIndexDict = {}
	numCameras = csl.init_bus1394()
	for i in range(numCameras):
		uidIndexDict[csl.getCameraUID(i)] = i
	csl.endCameraSetup()
	return uidIndexDict

##
# Returns a dictionary mapping from camera names to OpenCV ID for all hooked up
# cameras
def camera_names():
    ids = camera_ids()
    all_ids = {}

    for k in cc.camera_parameters.keys():
        all_ids[cc.camera_parameters[k]['uid']] = k

    name_camera_map = {}
    for k in ids.keys():
        name_camera_map[all_ids[k]] = ids[k]
    return name_camera_map

##
# Returns the OpenCV ID of a named camera
# @param camera_name
def lookup_by_name(camera_name):
    ids = camera_ids()
    print 'ids:', ids
    return ids[cc.camera_parameters[camera_name]['uid']]

if __name__ == '__main__':
    print 'Camera UUIDs', camera_ids()
    print 'Available cameras:', camera_names()
