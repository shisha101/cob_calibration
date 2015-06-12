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

'''
@summary: This is a script/node that is used to generate the System and sensor YAML files, as well as a template step based on user input, srdf(via moveit) and urdf
@note: It is important to note that any mechanism that moves that is an intermediate between the sensors or checker boards or base frame should be defined as a DH parameter group
since this affects the static transformations defined in system.yaml
@note: note that this script does not account for the possibility of one link having multiple parents
'''
def get_position_euler(listener,base , tip):
    now = rospy.Time.now()
    listener.waitForTransform(base, tip, now, rospy.Duration(4.0))
    translation, rotation_quat = listener.lookupTransform(base, tip, now)
    rotation_euler = tf.transformations.euler_from_quaternion(rotation_quat)
#     pdb.set_trace()
    return translation, rotation_euler

def get_parents():
    return 

def get_children(origin_frame,link_child_map_dict):
    '''
    @summary: this function returns the children of a givien link
    @param origin_frame: the frame for which we will get the children
    @param link_child_map_dict: the complete child map from the urdf 
#     @param all_camera_children_links_dict: this is a storage dictionary for all cameras and is passed to be populated
#     @param transformations_dict_debug: a debugging dict that is used for debugging
#     @param Flag: this determines if the frame is a chain or a sensor
    @return: returns a list of linkswhich are the children of the parent link 
    '''
    children_link_tree = []
    current_link = origin_frame
    while (link_child_map_dict.has_key(current_link)):# as long as the current frame has children append the children to children_link_tree
        childe_of_current_frame = link_child_map_dict[current_link][-1][-1]# the last entry contains the child link the
        children_link_tree.append(childe_of_current_frame)
        current_link = childe_of_current_frame
    return children_link_tree
    
def get_dh_chains_for_acctuated_sensors():
    return
def generate_dh_parameters_for_dh_chains(sensor_chain_groups,robot_urdf):
    """
    @summary: this function depends on the move it group and the urdf of the robot to extract the xyzrpy and return a dictionary that is the subdict of dh_chains found in system.yaml
    @param sensor_chain_groupsparam: this is the list of moveitcommandgroups for which the dh parameters will be extracted 
    @param robot_urdfparam: this is the robot urdf reader that supplies the information 
    """
#     population of dh_chains in the format that is understood by the optimizer not that willow garage use real DH parameters here we are using xyz and rpy
    dh_chains_dict = dict() # this corresponds to the dh_chains in the system yaml
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
    return dh_chains_dict

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    listener = tf.TransformListener()
    # init of move it 
    moveit_commander.roscpp_initialize(sys.argv)
    robot_moveit = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    #init of robot urdf reader 
    robot_urdf = URDF.load_from_parameter_server()
    print "============ Robot Groups:"
    group_names = robot_moveit.get_group_names()
    print group_names
    print "============"
#     pdb.set_trace()
    left_arm_group = moveit_commander.MoveGroupCommander(group_names[1]) # Rename the group
    right_arm_group = moveit_commander.MoveGroupCommander(group_names[2])
    right_gripper_group = moveit_commander.MoveGroupCommander(group_names[4])
    left_gripper_group = moveit_commander.MoveGroupCommander(group_names[3])
    sensor_chain_groups = [left_arm_group,right_arm_group]#,right_gripper_group,left_gripper_group]# sensor chains are groups that can hold the CB 
    ## robot_moveit.
#     print "============ Printing robot_moveit state"
#     print robot_moveit.get_current_state()
#     print "============"
# #     pdb.set_trace()
#     print "============ \nPrinting information about the %s group" %left_arm_group.get_name()
#     print "============ \nReference frame: %s" % left_arm_group.get_planning_frame()
#     print "============ \nEnd effector frame: %s" % left_arm_group.get_end_effector_link()
#     print "============ \nCurrent joint angles: %s \n" % left_arm_group.get_current_joint_values()
#     print "============ \nCurrent joints are: %s \n" % left_arm_group.get_joints()
#     left_arm_group_joint_names = robot_moveit.get_joint_names(left_arm_group.get_name())
#     left_arm_group_links_names = robot_moveit.get_link_names(left_arm_group.get_name())
    
    
    system_dict = dict()  # this is the entry corresponding to the system yaml file
    transfomations_dict = dict()
    sensors_dict = dict() # this is the complete system that will be saved
    '''
    @note: The following parameters should be given by the user
    @todo: The following parameters should be given by the user via a yaml file for instance
    '''
    sensor_chain_groups = [left_arm_group,right_arm_group]#,right_gripper_group,left_gripper_group]# sensor chains are groups that can hold the CB 
    base_frame = "base_link" # the frame to which the group basses will be transformed to
    camera_frames = ["torso_cam3d_left_link","torso_cam3d_right_link","torso_cam3d_down_link","gripper_left_camera_link","gripper_right_camera_link"]# the sensor links 
    cb_frames = ["cb_6x9_base_link_left","cb_6x9_base_link_right"] # CB links fixed to end effector
    
    
    link_parent_map_dict = robot_urdf.parent_map # a dict with a key for each link in the URDF each entry shows the links joint and the parent of that joint
    link_child_map_dict = robot_urdf.child_map  # a dict with a key for each link in the URDF each entry shows the links joint and the children of that joint
    '''
    @todo: the code below should be changed into functions divided into 
    - generate parents
    - generate children
    - extract DH chains for cameras and sensors
    - generate DH_chains for system yaml (done)
    '''

    system_dict["dh_chains"] = generate_dh_parameters_for_dh_chains(sensor_chain_groups,robot_urdf)# this corresponds to the dh_chains in the system yaml
#     pdb.set_trace()
    #construction of transformation part of the system description
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
        
        
        
        
#------------------------------------------------------------------------------ 
    """
            @note: this is only used for debugging 
    """
    Dump_dict_debug = dict()# temp dict used to visualize the data coming out of the function
    Dump_dict_debug["camera_chains"] = []# initalize a list of dicts
    transformations_dict_debug = dict()
#===============================================================================
# construction of the sensor yaml   and extraction of transofmations needed for the system yaml 
#===============================================================================
    all_camera_parent_links_dict = dict()
    all_camera_children_links_dict = dict()
    all_camera_dh_chains = dict()
    all_camera_dh_chain_link_number  = dict()
    link_parent_map_dict = robot_urdf.parent_map # a dict with a key for each link in the URDF each entry shows the links joint and the parent of that joint
    link_child_map_dict = robot_urdf.child_map  # a dict with a key for each link in the URDF each entry shows the links joint and the children of that joint
    for camera_frame in camera_frames:# for each sensor(camera in this case)
        if robot_urdf.links.has_key(camera_frame):# check if we have the camera frame in the robot URDF
            parent_link_tree = []
            children_link_tree = []
            transformations_dict_debug[camera_frame] = dict() #used for debuging
#this part corresponds to the extraction of the children of the camera frame the children are used to fully define the robots kinematics (since the order of the joints is specified in the sensors.yaml)
            print "starting child extraction"
            current_frame = camera_frame
            children_link_tree = get_children(current_frame, link_child_map_dict)
            all_camera_children_links_dict[camera_frame] = children_link_tree
            transformations_dict_debug[camera_frame]["children"] = children_link_tree
#this part corresponds to the extraction of the parents of the camera frame it does not append the last parent as it is either the base_frame or one of the links in the DH chains
            current_frame = camera_frame
            print "starting parent extraction"
            print current_frame
#             pdb.set_trace()
            while (not any(current_frame in group_links for group_links in all_group_links.itervalues()) and current_frame != base_frame):# the current parent is an the base frame or a part of a DH chain
                parent_link_tree.append(current_frame)
                print parent_link_tree
                if(link_parent_map_dict.has_key(current_frame)):
                    current_frame = link_parent_map_dict[current_frame][-1] # the last entry contains the parent link
                else:
                    print "the current frame for %s has no parent"%camera_frame
                    print "the parent tree found for %s is:\n %s"%(camera_frame,parent_link_tree)
                    sys.exit("")
            parent_link_tree.reverse() # proper arrangement of the order
            all_camera_parent_links_dict[camera_frame] = parent_link_tree
            transformations_dict_debug[camera_frame]["parents"] = parent_link_tree 
            print parent_link_tree
            print "checking if %s is fixed to a DH group or static"%camera_frame
#             pdb.set_trace()
            # this part determines the DH chain that should correspond to a sensor
            if (any(current_frame in group_links for group_links in all_group_links.itervalues())): #if one of the group_links (movable parts) is a parent of the camera_frame then there it should be in the chain dict entry for the camera_frame in sensors.yaml
                for key, value in all_group_links.items():# iterate through all the DH groups
                    if current_frame in value:# check for the current frame link in the current DH group
                        all_camera_dh_chains[camera_frame] = key # stores the name of the DH group that is a parent of the camera_frame
                        '''
                        @todo: check the index of this function
                        '''
                        all_camera_dh_chain_link_number[camera_frame] = value.index(current_frame)# store the number of the link in the parent DH gorup to which the camera_frame is fixed
                print "the %s link is actuated and therefore should have a DHchain, it's tree has been linked to the (%s)"%(camera_frame,all_camera_dh_chains[camera_frame])
            else:
                print "the %s link is not actuated and is thus static, it's tree has been linked to the base frame (%s)"%(camera_frame,base_frame)
                all_camera_dh_chains[camera_frame] = "" # stores an empty frame to denote that there is not DHframe between the camera/sensor and base
            
            """
            @note: this is only used for debugging 
            """
            
            temp_merge_dict = dict()
            temp_merge_dict["sensor_id"] = camera_frame
            temp_merge_dict["camera_id"] = camera_frame
            temp_merge_dict["chain"] = {"before_chain":parent_link_tree,
                                        "chain":all_camera_dh_chains[camera_frame],
                                        "after_chain":children_link_tree}
            
            
            Dump_dict_debug["camera_chains"].append(temp_merge_dict) # dict entry here for specific camera_sensor_chain
        else:
            print "the link %s does not exist inside the urdf\n System Exiting" %camera_frame
            sys.exit("")
    
      
    for group_current in sensor_chain_groups: # for each movable group it should be transformable to the base frame
        group_current_links = robot_moveit.get_link_names(group_current.get_name())
        first_link_of_group = group_current_links[0]#first link
        last_link_of_group = group_current_links[-1]#last link of group
        transformations_dict_debug[group_current.get_name()] = dict()
#------------------------------------------------------------------------------ 
# starting the extraction of children for the current DH group
        current_link_in_grp = last_link_of_group
        if robot_urdf.links.has_key(last_link_of_group):
            transformations_dict_debug[group_current.get_name()]["children"] = get_children(last_link_of_group, link_child_map_dict)
# starting the extraction of parents for the current DH group 
        current_link_in_grp = first_link_of_group           
        if robot_urdf.links.has_key(first_link_of_group):
            dh_parents = []
            while(current_link_in_grp != base_frame): # as long as the current frame has children append the children to dh_children
                if(link_parent_map_dict.has_key(current_link_in_grp)):
                    parent_of_current_link = link_parent_map_dict[current_link_in_grp][-1]# the last entry contains the parent link the
                    dh_parents.append(parent_of_current_link)
                    current_link_in_grp = parent_of_current_link
                else:
                    print "the current frame for %s has no parent and could not be linked to the base frame %s"%(group_current.get_name(),base_frame)
                    print "the parent tree found for %s is:\n %s"%(group_current.get_name(),dh_parents)
                    sys.exit("")
            dh_parents.reverse()# proper arrangement of the order
            transformations_dict_debug[group_current.get_name()]["parents"] = dh_parents
#------------------------------------------------------------------------------ 
# file dumping
    output_system = os.path.abspath("IDEcatkin_ws/src/cob_calibration/cob_calibration_executive/test/sensor_trial.yaml")
    with open(output_system, 'w') as f:
        f.write('####### This file is autogenerated. Do not edit #######\n')
        f.write(yaml.dump(Dump_dict_debug))  
    output_system = os.path.abspath("IDEcatkin_ws/src/cob_calibration/cob_calibration_executive/test/transfomations_trial.yaml")
    with open(output_system, 'w') as f:
        f.write('####### This file is autogenerated. Do not edit #######\n')
        f.write(yaml.dump(transformations_dict_debug))
#         i += 1 
    """
    @todo: Write a method that takes these dicts and produces the proper sensor.yaml
    a fixed camera can be treated as an actuated camera with an empty linkage
    
    @todo: remember to organize the names of the chains and sensors such that they match the bag file
    
    @todo: check the situation in which 2 DHcains are linked to each other as in the DH chain of a sensor should be a list and not just one single chain
    """
#===============================================================================
# generate the transofmation dict list needed for the DH groups
#===============================================================================
    
    print "this is the end of the camera tree loop"
    pdb.set_trace()
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
    #appending the the dict of dh_chains to the system dict under 
    #         print "DH is %s: "  %sensors_dict[joint_name_current]["dh"]
    #         print "cov is %s: " %[sensors_dict[joint_name_current]["cov"]
    #         print "gearing is %s: " %[sensors_dict[joint_name_current]["gearing"]
#     left_arm_group_joint_3 = robot_moveit.get_joint(left_arm_group_joint_names[4])
#     left_arm_group_link_3 = robot_moveit.get_link(left_arm_group_links_names[4])
#     left_arm_group_link_3.pose()
#     joint_dic_urdf = robot_urdf.joints
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
    

    
#     print "============ \nPrinting information about the %s group" %right_arm_group.get_name()
#     print "============ \nReference frame: %s" % right_arm_group.get_planning_frame()
#     print "============ \nEnd effector frame: %s" % right_arm_group.get_end_effector_link()
#     print "============ \nCurrent joint angles: %s\n" % right_arm_group.get_current_joint_values()
    pdb.set_trace()





if __name__ == '__main__':
    main()
    print "==> done exiting"