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
#   ROS package name: cob_calibration_executive
#
# \author
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: January 2012
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
PKG = 'cob_calibration_executive'
NODE = 'generate_robot_calibration_data_node'
""" The strings here regarding group names should be accounted for later.
The number of groups should be dynamic"""
import roslib
roslib.load_manifest(PKG)
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import tf
import numpy as np
import yaml
import os
import math
from simple_script_server import simple_script_server
from random import shuffle  #shuffle links to look at
# from cob_calibration_executive.torso_ik import TorsoIK

import pdb # debugger

#board       = Checkerboard(self.pattern_size, self.square_size)
#checkerboard_detector=CheckerboardDetector()
#latest_image=Image()

def get_cb_pose_center(listener, base_frame):
    return listener.lookupTransform(base_frame, '/chessboard_center', rospy.Time(0))


def get_cb_pose(listener, base_frame):
    return listener.lookupTransform(base_frame, '/chessboard_position_link', rospy.Time(0))

def get_position(listener, tip):
    return listener.lookupTransform('/torso_3_link', tip, rospy.Time(0))
def generate_calibration_trajectory(moveGroup,limits,discritization,orientation_mean,reference_frame,gazebo_simulation, cb_publisher):
    '''
    @param moveGroup: Is the move MoveGroupCommander of the group
    @type moveGroup: moveit_commander.MoveGroupCommander
    @param limits: Are the limits of the cube wrt to the @reference_frame
    @type limits: a list of lists [[x1,y1,z1],[x2,y2,z2]]
    @param discritization: the number of times for which the range is discretized [x,y,z]
    @type discritization: A list []
    @param orientation_mean: The mean Orientation before noise addition [x,y,z,w]
    @type orientation_mean: A list []
    @param reference_frame: the frame of refrence in which the planning will take place using the pose given by @limits and @orientation_mean
    @type reference_frame: String (name of reference frame)
    @param gazebo_simulation: Flag to signal if we want gazebo to simulate the path while planning or not
    @param gazebo_simulation: Boolean
    @param cb_publisher: Chess board publisher
    '''
    listener = tf.TransformListener()
    
    next_pose = geometry_msgs.msg.PoseStamped()  #init of the geometry_msgs pose
    current_joint_angles = moveGroup.get_current_joint_values()
    # making a state message and filling it
    group_state = moveit_msgs.msg.RobotState()
    group_state.joint_state.name = moveGroup.get_joints()
    group_state.joint_state.position = current_joint_angles
    # define cuboid for positions
    # limits from base_link frame
    limits = {'x': (limits[0][0], limits[1][0]),
              'y': (limits[0][1], limits[1][1]),
              'z': (limits[0][2], limits[1][2])}

    sample_density = {'x': discritization[0],
                      'y': discritization[1],
                      'z': discritization[2]}

    sample_positions = {'x': [],# initialization 
                        'y': [],
                        'z': []}
    for key in limits.keys():
        limits[key] = sorted(limits[key])
        sample_positions[key].append(limits[key][0])
        diff = limits[key][1] - limits[key][0]
        step = 1.0 * diff / (sample_density[key] - 1)

        while sample_positions[key][-1] + step <= (limits[key][1] + 0.01):
            sample_positions[key].append(sample_positions[key][-1] + step)

    group_joint_states = []
    group_joint_trajectory = []
    '''
@todo: Check this cb_links and find out if it should be removed
    '''
    cb_links = ["/chessboard_center","/chessboard_lu_corner",
                "/chessboard_ru_corner", "/chessboard_ll_corner",
                "/chessboard_rl_corner"]

    for x in sample_positions['x']:
        for y in sample_positions['y']:
            for z in sample_positions['z']:
                #for q in quaternion:
                for cb_link in cb_links:
                    """
@todo: Check why we need this cb_link loop
                    """
                    print "\033[1;34mNew Position\033[1;m"
                    next_pose.header.frame_id = '/torso_3_link' ## BASE LINK ??
                    next_pose.pose.position.x = x
                    next_pose.pose.position.y = y
                    next_pose.pose.position.z = z
                    '''
@todo: Check this part again for the addition of random noise
                    '''
                    # (0,0,0,1) for cob3-6
                    next_pose.pose.orientation.x = orientation_mean[0]
                    next_pose.pose.orientation.y = orientation_mean[1]
                    next_pose.pose.orientation.z = orientation_mean[2]
                    next_pose.pose.orientation.w = orientation_mean[3]

                    cb_publisher.publish(next_pose)  # is this publish still needed ??
                    rospy.sleep(0.2)
                    transformation_base_cb = get_position(
                        listener, '/chessboard_center')
#                     [x1, y1, z1] = transformation_base_cb[0]
#                     (dx, dy, dz) = (x1 - xhead, y1 - yhead, z1 - zhead)
#                     roll = 0
#                     pitch = math.atan2(dz, -dx)
#                     yaw = -math.atan2(dy, -dx)
#                     '''
#                     Add noise to roll pitch and yaw values of cb
#                     '''
#                     std_dev = 0.3
#                     roll = np.random.normal(roll,std_dev,1)[0]
#                     pitch = np.random.normal(pitch,std_dev,1)[0]
#                     yaw = np.random.normal(yaw,std_dev,1)[0]

#                     q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                    #print q
#                     next_pose.pose.orientation.x = q[0]
#                     next_pose.pose.orientation.y = q[1]
#                     next_pose.pose.orientation.z = q[2]
#                     next_pose.pose.orientation.w = q[3]
                    next_pose.pose.orientation.x = orientation_mean[0]
                    next_pose.pose.orientation.y = orientation_mean[1]
                    next_pose.pose.orientation.z = orientation_mean[2]
                    next_pose.pose.orientation.w = orientation_mean[3]
                    cb_publisher.publish(next_pose)
                    rospy.sleep(0.2)
                    transformation_base_cb = get_position(
                        listener, cb_link)
                #Set starting pose for trajectory to continue planning from the last achieved point 
                group_state.joint_state.position = current_joint_angles
                moveGroup.set_start_state(group_state)
                #Set end pose for trajectory
                moveGroup.set_pose_target(next_pose.pose)
                #Plant paths for groups
                group_path_planned = moveGroup.plan()
                if group_path_planned.joint_trajectory.header.frame_id: # check if planning succeeded
                    print "\033[1;32mA solution has been found\033[1;m"
                    #update next iterations starting pose
                    if(gazebo_simulation):
                        moveGroup.go()
                    group_joint_trajectory.append(group_path_planned)
                    current_joint_angles = group_path_planned.joint_trajectory.points[-1].positions #update the robots joint angles for next planning iteration
                    group_joint_states.append({'joint_position': group_path_planned.joint_trajectory.points[-1].positions}) # append the attainable pose\s joint angles to the list then YAML 
                else:
                    print "\033[1;31mNo solution was found\033[1;m"
    return [group_joint_trajectory, group_joint_states]
    
def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    rospy.wait_for_service("/move_group/plan_execution/set_parameters",timeout=30.0)
    chessboard_pose = rospy.Publisher('/cob_calibration/chessboard_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
    print 'chessboard_pose publisher activated'
    moveit_commander.roscpp_initialize(sys.argv) # init of moveit
    robot = moveit_commander.RobotCommander()# init of moveit wrt robot
#     scene = moveit_commander.PlanningSceneInterface() # init if scene if collisiton is to be avoided and for the addition of the cb pattern later
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============"
    listener = tf.TransformListener()
    rospy.sleep(1.5)
    arm_left_group = moveit_commander.MoveGroupCommander("arm_left") # define groups
    arm_right_group = moveit_commander.MoveGroupCommander("arm_right")
    arm_left_group.set_pose_reference_frame('torso_3_link')#***
    arm_right_group.set_pose_reference_frame('torso_3_link')#***
#     torso_ik = rospy.ServiceProxy('/lookat_get_ik', GetPositionIK) # no Torso link
    # init
    print "--> initializing sss"
    sss = simple_script_server()
    sss.init("base")
    sss.init("torso")
    sss.init("head")
    sss.recover("base")
    sss.recover("torso")
    sss.recover("head")
    sss.move("arm_left","home")
    sss.move("arm_right","home")
    print "--> components initialized"
    camera_link_left = "/torso_cam3d_left_link" #renamed
    camera_link_right = "/torso_cam3d_right_link" #new addition

    [xhead, yhead, zhead] = get_position(listener, camera_link_left)[0]
    print "Positon wrt torso_cam3d_left"
    print xhead, yhead, zhead
    
    [xhead, yhead, zhead] = get_position(listener, camera_link_right)[0]
    print "Positon wrt torso_cam3d_left"
    print xhead, yhead, zhead

    print "--> setup care-o-bot for capture"
#     next_pose = geometry_msgs.msg.PoseStamped()  #init of the geometry_msgs pose
#     current_pose_left = arm_left_group.get_current_joint_values()
#     current_pose_right = arm_right_group.get_current_joint_values()
#     # making a state message and filling it
#     arm_left_state = moveit_msgs.msg.RobotState()
#     arm_left_state.joint_state.name = arm_left_group.get_joints()
#     arm_right_state = moveit_msgs.msg.RobotState()
#     arm_right_state.joint_state.name = arm_right_group.get_joints()
#     arm_left_state.joint_state.position = current_pose_left
#     arm_right_state.joint_state.position = current_pose_right
#     # Debugging
# #     ref_frame_left = arm_right_group.get_pose_reference_frame()
# #     ref_frame_right = arm_left_group.get_pose_reference_frame()
# #     pdb.set_trace()
#      
# #     torso = TorsoIK()
#  
#     # define cuboid for positions
#     # limits from base_link frame Old values
# #     limits = {'x': (-0.4, -1.0),
# #               'y': (-0.3, 0.3),
# #               'z': (0.5, 1.5)}
#     #New values
#     limits = {'x': (0.57567, 0.34636),
#               'y': (-0.021282,  0.542),
#               'z': (0.62907-0.485,  1.1114-0.485)}
#  
#     sample_density = {'x': 6,
#                       'y': 6,
#                       'z': 6}#originally 6
#  
#     sample_positions = {'x': [],
#                         'y': [],
#                         'z': []}
#     for key in limits.keys():
#         limits[key] = sorted(limits[key])
#         sample_positions[key].append(limits[key][0])
#         diff = limits[key][1] - limits[key][0]
#         step = 1.0 * diff / (sample_density[key] - 1)
#      #   print key, ' ',diff,' ',step
#  
#         while sample_positions[key][-1] + step <= (limits[key][1] + 0.01):
#             sample_positions[key].append(sample_positions[key][-1] + step)
#  
#     joint_states_left = []
#     joint_states_right= []
#     joint_trajectory_left = []
#     joint_trajectory_right = []
# #     torso.get_torso_limits() #limits should have alerady been loaded in the launch files
#  
#     cb_links = ["/chessboard_center","/chessboard_lu_corner",
#                 "/chessboard_ru_corner", "/chessboard_ll_corner",
#                 "/chessboard_rl_corner"]
#  
#     for x in sample_positions['x']:
#         for y in sample_positions['y']:
#             for z in sample_positions['z']:
#                 #for q in quaternion:
#                 for cb_link in cb_links:
#                     print "\033[1;34mNew Position\033[1;m"
#                     next_pose.header.frame_id = '/torso_3_link' ## BASE LINK ??
#                     next_pose.pose.position.x = x
#                     next_pose.pose.position.y = y
#                     next_pose.pose.position.z = z
#  
#                     # (0,0,0,1) for cob3-6
#                     next_pose.pose.orientation.x = 0
#                     next_pose.pose.orientation.y = 0
#                     next_pose.pose.orientation.z = 0
#                     next_pose.pose.orientation.w = 1
#  
#                     chessboard_pose.publish(next_pose)  # is this publish still needed ??
#                     rospy.sleep(0.2)
#                     transformation_base_cb = get_position(
#                         listener, '/chessboard_center')
#                     [x1, y1, z1] = transformation_base_cb[0]
#                     (dx, dy, dz) = (x1 - xhead, y1 - yhead, z1 - zhead)
#                     roll = 0
#                     pitch = math.atan2(dz, -dx)
#                     yaw = -math.atan2(dy, -dx)
#                     '''
#                     Add noise to roll pitch and yaw values of cb
#                     '''
#                     std_dev = 0.3
#                     roll = np.random.normal(roll,std_dev,1)[0]
#                     pitch = np.random.normal(pitch,std_dev,1)[0]
#                     yaw = np.random.normal(yaw,std_dev,1)[0]
#  
# #                     q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
#                     #print q
# #                     next_pose.pose.orientation.x = q[0]
# #                     next_pose.pose.orientation.y = q[1]
# #                     next_pose.pose.orientation.z = q[2]
# #                     next_pose.pose.orientation.w = q[3]
#                     q = [0.0049228, 0.01897, -0.70297, 0.71095]
#                     next_pose.pose.orientation.x = q[0]
#                     next_pose.pose.orientation.y = q[1]
#                     next_pose.pose.orientation.z = q[2]
#                     next_pose.pose.orientation.w = q[3]
#                     chessboard_pose.publish(next_pose)
#                     rospy.sleep(0.2)
#                     transformation_base_cb = get_position(
#                         listener, cb_link)
#                 #Set starting pose for trajectory to continue planning from the last achieved point 
# #                 arm_left_group.set_start_state(current_pose_left) # not compatible since the pose is not a state check arm_left_state and right
# #                 arm_right_group.set_start_state(current_pose_right)
# #                 pdb.set_trace()
#                 arm_left_state.joint_state.position = current_pose_left
#                 arm_right_state.joint_state.position = current_pose_right
#                 arm_left_group.set_start_state(arm_left_state)
#                 arm_right_group.set_start_state(arm_right_state)
#                 #Set end pose for trajectory
#                 arm_left_group.set_pose_target(next_pose.pose)
#                 arm_right_group.set_pose_target(next_pose.pose)
#                 #Plant paths for groups
#                 path_left = arm_left_group.plan()
#                 path_right = arm_right_group.plan()
#                 if path_left.joint_trajectory.header.frame_id: # check if planning succeeded
#                     print "\033[1;32mA solution for left has been found\033[1;m"
#                     #update next iterations starting pose
# #                     pdb.set_trace()
#                     arm_left_group.go()
#                     joint_trajectory_left.append(path_left)
#                     current_pose_left = path_left.joint_trajectory.points[-1].positions
# #                     joint_states_left.append(str('\njoint_position: \n'))
#                     joint_states_left.append({'joint_position': path_left.joint_trajectory.points[-1].positions}) # append the attainable pose to the list then YAML 
#                 else:
#                     print "\033[1;31mNo solutuin for left was found\033[1;m"
#                 if path_right.joint_trajectory.header.frame_id:# check if planning succeeded
#                     print "\033[1;32mA solution for right has been found\033[1;m"
# #                     pdb.set_trace()
#                     arm_right_group.go()
#                     joint_trajectory_right.append(path_right)
#                     current_pose_right = path_right.joint_trajectory.points[-1].positions
# #                     joint_states_right.append('\njoint_position: \n')
#                     joint_states_right.append({'joint_position': path_right.joint_trajectory.points[-1].positions}) # append the attainable pose to the list then YAML 
#                 else:
#                     print "\033[1;31mNo solutuin for right was found\033[1;m"
#  
#     file_path = rospy.get_param('~output_path', None)
#     directory = os.path.dirname(file_path)
#  
#     if file_path is not None:
#         if not os.path.exists(directory):
#             os.makedirs(directory)
#         file_path_joint_angles_left = file_path+"calibration_positions_left.yaml"
#         file_path_joint_angles_right = file_path+"calibration_positions_right.yaml"
#         with open(file_path_joint_angles_left, 'w') as f:
#             f.write('# autogenerated: Do not edit #\n')
#             f.write(yaml.dump(joint_states_left))
#         with open(file_path_joint_angles_right, 'w') as f:
#             f.write('# autogenerated: Do not edit #\n')
#             f.write(yaml.dump(joint_states_right))
#         file_path_joint_trajectory_left = file_path+"calibration_trajectory_left.yaml"
#         file_path_joint_trajectory_right = file_path+"calibration_trajectory_right.yaml"
#         with open(file_path_joint_trajectory_left, 'w') as f:
#             f.write('# autogenerated: Do not edit #\n')
#             f.write(yaml.dump(joint_trajectory_left))
#         with open(file_path_joint_trajectory_right, 'w') as f:
#             f.write('# autogenerated: Do not edit #\n')
#             f.write(yaml.dump(joint_trajectory_right))
#          
# #     else:
# #         print yaml.dump(joint_states)
    arm_list = [arm_left_group,arm_right_group]
    arm_palnning_refrence_frames = ['torso_3_link','torso_3_link']
    orientation_mean = [[0.0049228, 0.01897, -0.70297, 0.71095],
                        [0.0049228, 0.01897, -0.70297, 0.71095]]
    limits = [    
              [[0.57567,-0.021282,0.62907-0.485],[0.34636,0.542,1.1114-0.485]],
              [[0.57567,-0.021282,0.62907-0.485],[0.34636,-0.542,1.1114-0.485]]
              ]
    discritization = [[2,2,2],
                      [2,2,2]]
    Trajectory_Joint_angles = [] # the first element of each entry is the Trajectory list for all successful calibration points while the second is the list of finla joint angles corresponding to that list
    
    file_path = rospy.get_param('~output_path', None)
    directory = os.path.dirname(file_path)
    if file_path is not None:
        if not os.path.exists(directory):
            os.makedirs(directory)
    
    for i in xrange(len(arm_list)):
        Trajectory_Joint_angles.append(generate_calibration_trajectory(
                                                    arm_list[i], 
                                                    limits[i],
                                                    discritization[i],
                                                    orientation_mean[i],
                                                    arm_palnning_refrence_frames[i],
                                                    False,
                                                    chessboard_pose))
  
        file_path_group_angels = file_path+arm_list[i].get_name()+"calibration_positions.yaml"
        with open(file_path_group_angels, 'w') as f:
            f.write('# autogenerated: Do not edit #\n')
            f.write(yaml.dump(Trajectory_Joint_angles[i][1]))# this is the second entry of the element in the list corresponding to the joint angles for all successful calibration points
        file_path_group_trajectories = file_path+arm_list[i].get_name()+"calibration_trajectories.yaml"
        with open(file_path_group_trajectories, 'w') as f:
            f.write('# autogenerated: Do not edit #\n')
            f.write(yaml.dump(Trajectory_Joint_angles[i][0])) #this is the first entry of the element in the list corresponding to the joint trajectories for all successful calibration points
            print '%s ik solutions found for left arm' % len(Trajectory_Joint_angles[i][0])
            rospy.sleep(5)
def calculate_angles(t):
    '''
    computes pan and tilt angles for camera like translations
    z-axis: optical axis
    y-axis: vertical
    x-axis: horizontal
    '''
    angles = {}
    angles['p'] = np.arctan2(t[0], t[2])
    angles['t'] = np.arctan2(t[1], t[2])
    return angles

if __name__ == '__main__':
    main()
    print "==> done exiting"
