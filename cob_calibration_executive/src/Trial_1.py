#!/usr/bin/env python
PKG = 'cob_calibration_executive'
NODE = 'InformationParserNode'
import sys
import moveit_commander
import rospy
import pdb
import tf
from urdf_parser_py.urdf import URDF
import os
import yaml
# import roslib
# import yaml
def get_position_euler(listener,base , tip):
    now = rospy.Time.now()
    listener.waitForTransform(base, tip, now, rospy.Duration(4.0))
    translation, rotation_quat = listener.lookupTransform(base, tip, now)
    rotation_euler = tf.transformations.euler_from_quaternion(rotation_quat)
#     pdb.set_trace()
    return translation, rotation_euler


def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    listener = tf.TransformListener()
    moveit_commander.roscpp_initialize(sys.argv)
#     rospy.sleep(10)
    robot_moveit = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    robot_urdf = URDF.load_from_parameter_server()
    print "============ Robot Groups:"
    group_names = robot_moveit.get_group_names()
    print group_names
    print "============"
#     pdb.set_trace()
    left_arm_group = moveit_commander.MoveGroupCommander(group_names[1]) # Rename the group
    right_arm_group = moveit_commander.MoveGroupCommander(group_names[2])
    sensor_chain_groups = [left_arm_group,right_arm_group]# sensor chains are groups that can hold the CB 
    ## robot_moveit.
    print "============ Printing robot_moveit state"
    print robot_moveit.get_current_state()
    print "============"
#     pdb.set_trace()
    print "============ \nPrinting information about the %s group" %left_arm_group.get_name()
    print "============ \nReference frame: %s" % left_arm_group.get_planning_frame()
    print "============ \nEnd effector frame: %s" % left_arm_group.get_end_effector_link()
    print "============ \nCurrent joint angles: %s \n" % left_arm_group.get_current_joint_values()
    print "============ \nCurrent joints are: %s \n" % left_arm_group.get_joints()
    left_arm_group_joint_names = robot_moveit.get_joint_names(left_arm_group.get_name())
    left_arm_group_links_names = robot_moveit.get_link_names(left_arm_group.get_name())
    system_dict = dict()  # this is the entry corresponding to the system yaml file
    dh_chains_dict = dict() # this corresponds to the dh_chains in the system yaml
    transfomations_dict = dict()
    sensors_dict = dict() # this is the complete system that will be saved
    '''
    @see: The following parameters should be given by the user
    '''
    base_frame = "odom_combined" # the frame to which the group basses will be transformed to
    camera_frames = ["torso_cam3d_left_link","torso_cam3d_right_link","torso_cam3d_down_link"]# the sensor links 
    cb_frames = ["cb_6x9_base_link_left","cb_6x9_base_link_right"] # CB links fixed to end effector
    ## population of dh_chains in the format that is understood by the optimizer not that willow garage use real DH parameters here we are using xyz and rpy 
    
    for group_current in sensor_chain_groups:
        current_chain_joint_names = group_current.get_joints()
        sensor_list = []#this corresponds to the list inside system for dh parameters for each group
        for joint_name_current in current_chain_joint_names:
            current_joint = robot_urdf.joints[joint_name_current]
            type_of_joint = ""
            # print joint.name
            if current_joint.joint_type == current_joint.REVOLUTE:
                type_of_joint += "rot"
            elif current_joint.joint_type == current_joint.PRISMATIC:
                type_of_joint += "trans"
            else:
                print"The type of %s is not prismatic nor revolute please check"% current_joint.name
#             pdb.set_trace()
            if current_joint.axis == "1 0 0":
                type_of_joint += "x"
            elif current_joint.axis == "0 1 0":
                type_of_joint += "y"
            elif current_joint.axis == "0 0 1":
                type_of_joint += "z"
            sensor_list.append({"name":current_joint.name,
                                "type": type_of_joint,
                                "xyzrpy": current_joint.origin.position + current_joint.origin.rotation})# here we append a dictionary for each joint
        
        dh_chains_dict[group_current.get_name()] = {"cov":[0.01]*len(sensor_list),
                                                    "dh":sensor_list,
                                                    "gearing":[1]*len(sensor_list)
                                                    }#appending the chain to DH chains
    system_dict["dh_chains"] = dh_chains_dict
#     pdb.set_trace()
    #construction of transfomation part of the system discription
    
    
    all_group_links = dict()
    all_group_joints = dict() # not needed atm but does not hurt
    for group_current in sensor_chain_groups:
        links_of_current_group = []
        first_joint_of_group = group_current.get_joints()[0] #this returns the first element of the list of joints
        base_link_of_currnet_group = robot_urdf.joints[first_joint_of_group].parent # the parent of the 1st joint is the base link for the group
        links_of_current_group.append(base_link_of_currnet_group)
        links_of_current_group.extend(robot_moveit.get_link_names(group_current.get_name()))
        all_group_links[group_current.get_name()] = links_of_current_group #the dictionary that contains an entry for each group and under each entry has the list of links
        all_group_joints[group_current.get_name()] = group_current.get_joints()
        
    
    all_camera_links_dict = dict()
    link_parent_map_dict = robot_urdf.parent_map # a dict with a key for each link in the URDF each entry shows the links joint and the parent of that joint
    for camera_frame in camera_frames:
        if robot_urdf.links.has_key(camera_frame):
            parent_link_tree = []
            current_frame = camera_frame
            print current_frame
            pdb.set_trace()
            while (current_frame not in all_group_links and current_frame != base_frame):# the last element of the tuple given by the dict entry is the parent of the current links joint
                parent_link_tree.append(current_frame)
                try:
                    current_frame = link_parent_map_dict[current_frame][-1] # if we cant link it to something
                except KeyError:
                    print "the current frame for %s has no parent"%camera_frame
                    sys.exit("")
            parent_link_tree = parent_link_tree.reverse() # proper arrangment of the order
            all_camera_links_dict[camera_frame] = parent_link_tree
        else:
            print "the link %s does not exist inside the urdf" %camera_frame
    
    
    """
    @todo: Write a method that takes this dict and produces the proper sensor.yaml
    """
    
    
    
#     pdb.set_trace()
    for group_current in sensor_chain_groups:# iterate through all the groups for kinematics which also act as sensors
        first_joint_of_group = group_current.get_joints()[0] #this returns the first element of the list of joints
        base_link_of_currnet_group = robot_urdf.joints[first_joint_of_group].parent # the parent of the 1st joint is the base link for the group
#         pdb.set_trace()
        translation, rotation_euler = get_position_euler(listener, base_frame, base_link_of_currnet_group)
        transfomations_dict[base_link_of_currnet_group] = list(translation) + list(rotation_euler) # list of rotations
#         print transfomations_dict
    for cb in cb_frames:
        if robot_urdf.links.has_key(cb): #we have a CB in the left hand
            translation, rotation_euler = get_position_euler(listener, base_frame, cb)
            transfomations_dict[cb] = list(translation) + list(rotation_euler) # list of rotations
        else:
            print "the link %s does not exist inside the urdf" %cb
    print transfomations_dict
    for camera_frame in camera_frames:
        if robot_urdf.links.has_key(camera_frame): #we have a CB in the left hand
            translation, rotation_euler = get_position_euler(listener, base_frame, camera_frame)
            transfomations_dict[camera_frame] = list(translation) + list(rotation_euler) # list of rotations
        else:
            print "the link %s does not exist inside the urdf" %camera_frame
    print transfomations_dict
#       translation, rotation_quat = get_position(listener, base_frame, left_arm_group.get_end_effector_link())
    system_dict['checkerboards'] = rospy.get_param('checkerboards', None)
    if system_dict['checkerboards'] is None:
        print '[ERROR]: Parameter checkerboards not found. Make sure it is set and try again'
        return
    system_dict['dh_chains'] = dh_chains_dict
    system_dict["transforms"] =   transfomations_dict
    
#     output_system = '~/Home/Refrences/SystemTrial.yaml'
    pdb.set_trace()
    output_system = os.path.abspath("IDEcatkin_ws/src/cob_calibration/cob_calibration_executive/test/SystemTrial.yaml")
    with open(output_system, 'w') as f:
        f.write('####### This file is autogenerated. Do not edit #######\n')
        f.write(yaml.dump(system_dict))  
    pdb.set_trace()
    #appending the the dict of dh_chains to the system dict under 
    #         print "DH is %s: "  %sensors_dict[joint_name_current]["dh"]
    #         print "cov is %s: " %[sensors_dict[joint_name_current]["cov"]
    #         print "gearing is %s: " %[sensors_dict[joint_name_current]["gearing"]
    left_arm_group_joint_3 = robot_moveit.get_joint(left_arm_group_joint_names[4])
    left_arm_group_link_3 = robot_moveit.get_link(left_arm_group_links_names[4])
    left_arm_group_link_3.pose()
    joint_dic_urdf = robot_urdf.joints
    """
    @attention: This are the functions and parameters of the URDF
    """
# robot_urdf.add_joint                   robot_urdf.load_from_parameter_server
# robot_urdf.add_link                    robot_urdf.load_xml_file
# robot_urdf.child_map                   robot_urdf.materials
# robot_urdf.elements                    robot_urdf.name
# robot_urdf.get_chain                   robot_urdf.parent_map
# robot_urdf.get_root                    robot_urdf.parse_xml_string
# robot_urdf.joints                      robot_urdf.to_xml
# robot_urdf.links                       
# 
    """
    @attention: This are the functions and parameters of the URDF
    """
# robot_urdf.joints.clear       robot_urdf.joints.iteritems   robot_urdf.joints.setdefault
# robot_urdf.joints.copy        robot_urdf.joints.iterkeys    robot_urdf.joints.update
# robot_urdf.joints.fromkeys    robot_urdf.joints.itervalues  robot_urdf.joints.values
# robot_urdf.joints.get         robot_urdf.joints.keys        robot_urdf.joints.viewitems
# robot_urdf.joints.has_key     robot_urdf.joints.pop         robot_urdf.joints.viewkeys
# robot_urdf.joints.items       robot_urdf.joints.popitem     robot_urdf.joints.viewvalues
    """
    @attention: This are the functions and parameters of the URDF joints
    """
# robot_urdf.joints['key'].CONTINUOUS   robot_urdf.joints['key'].PRISMATIC    robot_urdf.joints['key'].calibration  robot_urdf.joints['key'].limits       robot_urdf.joints['key'].parent       
# robot_urdf.joints['key'].FIXED        robot_urdf.joints['key'].REVOLUTE     robot_urdf.joints['key'].child        robot_urdf.joints['key'].mimic        robot_urdf.joints['key'].parse        
# robot_urdf.joints['key'].FLOATING     robot_urdf.joints['key'].UNKNOWN      robot_urdf.joints['key'].dynamics     robot_urdf.joints['key'].name         robot_urdf.joints['key'].safety       
# robot_urdf.joints['key'].PLANAR       robot_urdf.joints['key'].axis         robot_urdf.joints['key'].joint_type   robot_urdf.joints['key'].origin       robot_urdf.joints['key'].to_xml    
    """
    @attention: This are the functions and parameters of the URDF jpints keys origin
    """
# robot_urdf.joints['key'].origin.parse     robot_urdf.joints['key'].origin.position  robot_urdf.joints['key'].origin.rotation  robot_urdf.joints['key'].origin.to_xml

# this has been added as is from generate_config due to it's simplicity
    
    pdb.set_trace()
    
    print "============ \nPrinting information about the %s group" %right_arm_group.get_name()
    print "============ \nReference frame: %s" % right_arm_group.get_planning_frame()
    print "============ \nEnd effector frame: %s" % right_arm_group.get_end_effector_link()
    print "============ \nCurrent joint angles: %s\n" % right_arm_group.get_current_joint_values()
    pdb.set_trace()





if __name__ == '__main__':
    main()
    print "==> done exiting"