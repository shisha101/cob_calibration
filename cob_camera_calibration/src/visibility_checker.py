#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_calibration
# \note
#   ROS package name: cob_image_capture
#
# \author
#   Author: Jannik Abbenseth, email:jannik.abbenseth@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: December 2011
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################
PKG = 'cob_camera_calibration'
NODE = 'visibility_check'
import roslib
roslib.load_manifest(PKG)
import rospy

from cob_camera_calibration import Checkerboard, CheckerboardDetector, cv2util
from sensor_msgs.msg import Image
from cob_calibration_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError

import pdb

class VisibilityCheckerNode():
    '''
    @summary: Captures Images from one or more cameras (Image message topics) to files.

    Number of cameras, output folder and file names are configurable via ROS parameters.
    After startup call "~capture_images" ROS service to save images of all
    cameras to output folder.
    '''

    def __init__(self):
        '''
        Initializes storage, gets parameters from parameter server and logs to rosinfo
        '''
        self.bridge = CvBridge()

        # set up Checkerboard and CheckerboardDetector
        self.board = Checkerboard((9, 6), 0.03)                 ### Take the parameters from the parameter server
        self.detector = CheckerboardDetector(self.board)
        rospy.init_node(NODE)
        self.counter = 0

        # Get params from ros parameter server or use default
        self.cams = rospy.get_param("~cameras")
        self.camera = [self.cams["reference"]["topic"]]
        if self.cams.has_key("further"):
            rospy.loginfo("FURTHER cameras specified")
            for cam in self.cams["further"]:
                 self.camera.append(cam["topic"])
        else:
            rospy.logwarn("No FURTHER cameras specified")
        self.numCams = len(self.camera)

        # Init images
        self.image = []
        self.image_received = [False] * self.numCams
        for id in range(self.numCams):
            self.image.append(Image())  # wait for a message from each camera this ensures that all cameras are working

        # Subscribe to images
        self.imageSub = []
        for id in range(self.numCams):
            self.imageSub.append(rospy.Subscriber(
                self.camera[id], Image, self._imageCallback, id))

        # Wait for image messages
        for id in range(self.numCams):
            print "now checking for incoming frames on the %s topic" %self.camera[id]
#             pdb.set_trace()
            rospy.wait_for_message(self.camera[id], Image, 5)

    def _imageCallback(self, data, id):
        '''
        Copy image message to local storage

        @param data: Currently received image message
        @type  data: ROS Image() message
        @param id: Id of camera from which the image was received
        @type  id: integer
        '''
        #print "cb executed"
        self.image[id] = data
        self.image_received[id] = True

    def _checkHandle(self, req):
        '''
        Service handle for "Capture" Service
        Grabs images, converts them and saves them to files

        @param req: service request
        @type  req: CaptureRequest() message

        @return: CaptureResponse() message
        '''
        self.image_received = [False] * self.numCams
        visible = []# this list contians the boolean information concerning the detection of a CB pattern in each image
        visible_cam_id = []
        while not all(self.image_received):
            print "waiting for images"
            rospy.sleep(1)
        if all(self.image_received):
            print "=== ALL IMAGES RECEIVED ==="
        self.images_received = [False] * self.numCams
        for id in range(self.numCams):
            image = self.image[id]
            cvImage = self.bridge.imgmsg_to_cv2(image, 'rgb8') # changed from imgmsg_to_cv to cv2 (update)

#             img_raw = cv2util.cvmat2np(cvImage)
            try:
                self.detector.detect_image_points(cvImage, False, True)
                visible.append(True)
                visible_cam_id.append(self.camera[id])
            except self.detector.NoPatternFoundException:
                rospy.logwarn("No cb found")
                visible.append(False)
                visible_cam_id.append(self.camera[id])
            # grab image messages
        #print '%s checkerboards found --> return %s'%(sum(visible),all(visible))
        response = VisibleResponse()
        pdb.set_trace()
        response.every = False
        response.master = False
        response.visibleImages = visible
        response.cameraTopicID = visible_cam_id
        if all(visible):
            response.every = True
            response.master = True

        #elif visible[0]:
        #   response.master = True
        return response

    def run(self):
        # Start service
        srv = rospy.Service(
            '/image_capture/visibility_check', Visible, self._checkHandle)
        rospy.loginfo("service '/image_capture/visibility_check' started, waiting for requests...")
        rospy.spin()


if __name__ == '__main__':
    node = VisibilityCheckerNode()
    node.run()
