#!/usr/bin/env python
import rospy

import os
from cob_calibration_msgs.msg._RobotMeasurement import RobotMeasurement
import functools
import rosbag
import numpy
from cv_bridge import CvBridge, CvBridgeError

import cv
import cv2

from calibrator import MonoCalibrator, StereoCalibrator, ChessboardInfo, Patterns
from sensor_msgs import *

NODE = 'monocalibrator'

#############
# TODO:
# - Add stereo calibration support
# - 
#############



def calibrate_mono(bag_file_dir, boards, camera_namespace, pattern=Patterns.Chessboard, directory='/tmp', min_samples=0):
    rospy.init_node(NODE)
    calib_flags = 0
    
    bag = rosbag.Bag(bag_file_dir)
    print bag_file_dir
    cameras = {}
    for topic, msg, t in bag.read_messages(topics=['/robot_measurement', 'robot_measurement']):
        if topic == "robot_measurement" or topic == "/robot_measurement":
            cam = msg.M_cam[0].camera_id
            if not cameras.has_key(cam):
                cameras[cam] = {}
                cameras[cam]['raw_images'] = []
                cameras[cam]['cv_images'] = []
                cameras[cam]['corners'] = []
            cameras[cam]['raw_images'].append(msg.M_cam[0].image)
    bag.close()
    
    bridge = CvBridge()
    for key, value in cameras.iteritems():              # Convert ROS Image msg into CV format
        print key
        for img in cameras[key]['raw_images']:
            i1 = bridge.imgmsg_to_cv2(img, 'bgr8')
            cameras[key]['cv_images'].append(i1)
    
    
    for key, value in cameras.iteritems():
        cameras[key]['calibrator'] = MonoCalibrator(boards,           # Init the calibrator object for each camera
                                                    calib_flags, 
                                                    pattern, 
                                                    name=key)
    
    for key, value in cameras.iteritems():
        print key
        for img in cameras[key]['raw_images']:
            cameras[key]['calibrator'].handle_msg(img)      # Detects the calibration target, if found and provides enough new information, adds it to the sample database
    
    all_good = True
    some_good = False
    for key, value in cameras.iteritems():
        if cameras[key]['calibrator'].goodenough:
            cameras[key]['goodenough'] = True
            some_good = True
        else:
            cameras[key]['goodenough'] = False
            all_good = False
    
    if not all_good and some_good:
        print 'Not enough calibration data for the following camera(s): '
        for key in cameras.keys():
            if cameras[key]['goodenough'] is False:
                print key
        raw_input('Press enter to continue the calibration for rest the of the cameras...')
        
    if not some_good:
        print 'Not enough calibration data. More sample images needed.'
        print 'Current sample image numbers:'
        for key in cameras.keys():
            if cameras[key]['goodenough'] is False:
                print key + ': ' + str(len(cameras[key]['calibrator'].good_corners))
    
    for key in cameras.keys():
        #if cameras[key]['goodenough'] is True:                ### Should we check goodenough?
        print cameras[key]['calibrator'].do_calibration()
        cameras[key]['calibrator'].do_save(directory, key)
        



def main():
    ptrn_size = []
    sqr_size = []
    bag_file_dir = '/tmp/cal/cal_measurements.bag'
    ptrn_size.append(rospy.get_param('/cob_camera_calibration/pattern_size'))
    sqr_size.append(rospy.get_param('/cob_camera_calibration/square_size'))
    
    result_directory = '/tmp/cal/'
    camera_namespace = '###'        ### Change this later (needed for uploading the calibration results to camera)

    if len(ptrn_size) != len(sqr_size):
        parser.error("Number of size and square inputs must be the same!")

    boards = []
    for (sz, sq) in zip(ptrn_size, sqr_size):
        size = tuple([int(c) for c in sz.split('x')])
        boards.append(ChessboardInfo(size[0], size[1], float(sq)))

    pattern = Patterns.Chessboard

    calibrate_mono(bag_file_dir, boards, camera_namespace, pattern, result_directory)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
