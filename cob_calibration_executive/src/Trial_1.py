#!/usr/bin/env python
from mercurial.commands import summary
from bonobo._bonobo import Listener
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
import copy
# import roslib
# import yaml

'''
@summary: This is a script/node that is used to generate the System and sensor 
YAML files, as well as a template step based on user input, srdf(via moveit) and urdf
@note: It is important to note that any mechanism that moves that is an 
intermediate between the sensors or checker boards or base frame should be 
defined as a DH parameter group since this affects the static transformations defined in system.yaml
@note: note that this script does not account for the possibility of one link having multiple parents
@note: This code has not been implemented to perform as fast as possible but to be as modular and easy to trace/edit as possible
'''
def extract_transformations(extraction_tree,listener,sanity_check_dict, parent_of_extraction_tree, endframe_of_extraction_tree):
    '''
    @summary: This method extracts the transformations from the end of a given list to the start, 
    this however does not yet extract the transformation of the last element of 
    the list[a,b,c,d] would give d in c, c in b, b in a but not a in its parent
    @param extraction_treeparam: this parameter is the list of frames that 
    we which to transform from back to front
    @param listener: a tf.listner()
    @param sanity_check_dict: this is an input output dict that checks and stops 
    the program if a frame tries to be transformed to two or more different parents
    @param parent_of_extraction_tree: this is the name of the parent link of the 
    extraction tree for which a will be transformed to relative to the example given in the summary
#     @param child_of_extraction_tree: This is the transformation from a
    @return: returns a dict of the transformation relative to one another
    '''
    transformation_sub_dict = dict()
    extraction_tree_copy = list(extraction_tree)
#     pdb.set_trace()
    if extraction_tree_copy != []:# if the extraction tree has atleast one entry aka not empty
        if parent_of_extraction_tree != None:
            extraction_tree_copy.insert(0, parent_of_extraction_tree) # add the parent of the extraction tree to the tree
        if endframe_of_extraction_tree != None:
            extraction_tree_copy.append(endframe_of_extraction_tree) # add the parent of the extraction tree to the tree
        for i in xrange(len(extraction_tree_copy)-1, 0, -1): # decremental loop from (start, step, increment)
#                 if(i==1):
#                     print "I is equal to 1"
                current_frame = extraction_tree_copy[i]
                parent_frame = extraction_tree_copy[i-1]
                translation, rotation_euler = get_position_euler(listener, parent_frame, current_frame)
                
                if sanity_check_dict.has_key(current_frame):
                    if sanity_check_dict[current_frame] != parent_frame:
                        print 'the frame %s has been previously transformed to %s and now requests to be transformed to %s'%(current_frame,sanity_check_dict[current_frame],parent_frame)
#                         pdb.set_trace()
                else:
                    sanity_check_dict[current_frame] = parent_frame # this denotes that this frame has been transformed to the frame before
                transformation = list(translation) + list(rotation_euler) # list of rotations
                transformation_sub_dict[current_frame] = transformation
    return transformation_sub_dict, sanity_check_dict

def get_position_euler(listener,base , tip):
    now = rospy.Time.now()
    listener.waitForTransform(base, tip, now, rospy.Duration(4.0))
    translation, rotation_quat = listener.lookupTransform(base, tip, now)
    rotation_euler = tf.transformations.euler_from_quaternion(rotation_quat)
#     pdb.set_trace()
    return translation, rotation_euler

def get_parents(origin_frame,link_parent_map_dict,all_group_links,base_frame,Actuated_Flag):
    '''
    @summary: this function returns the parents of a given link and stops depending on the condition
    either at the base link for a non actuated frame or at a link of one of the moving DH chains
    @param origin_frame: the frame for which we will get the children
    @param link_parent_map_dict: the complete parent map from the urdf
    @param all_group_links: this is a dict containing an entry for each DH group where each DH group contains a list of its links
    @param base_frame: The name of the base frame
    @return: returns a list of links which are the parents of the origin link
    @return: returns a flag that denotes whether the link is fixed to a DH group or not 
    '''
    parents_link_tree = []
    current_frame = origin_frame
#     parents_link_tree.append(current_frame)
    if Actuated_Flag:
        while (not any(current_frame in group_links for group_links in all_group_links.itervalues()) and current_frame != base_frame):# the current parent is an the base frame or a part of a DH chain
            if(link_parent_map_dict.has_key(current_frame)):
                parent_of_current_link = link_parent_map_dict[current_frame][-1]# the last entry contains the parent link the
                if(not any(parent_of_current_link in group_links for group_links in all_group_links.itervalues())):# check if the link to be appended is part of the DH groups if not then append
                    parents_link_tree.append(parent_of_current_link)
                current_frame = parent_of_current_link
            else:
                print "the current frame for %s has no parent"%origin_frame
                print "the parent tree found for %s is:\n %s"%(origin_frame,parents_link_tree)
                sys.exit("")
        if (any(current_frame in group_links for group_links in all_group_links.itervalues())):
            linked_to_DH_chain = True
        else:
            linked_to_DH_chain = False
    else:
        while(current_frame != base_frame): # as long as the current frame has children append the children to dh_children
            if(link_parent_map_dict.has_key(current_frame)):
                parent_of_current_link = link_parent_map_dict[current_frame][-1]# the last entry contains the parent link the
                parents_link_tree.append(parent_of_current_link)
                current_frame = parent_of_current_link
            else:
                print "the current frame for %s has no parent and could not be linked to the base frame %s"%(origin_frame,base_frame)
                print "the parent tree found for %s is:\n %s"%(origin_frame,parents_link_tree)
                sys.exit("")
        linked_to_DH_chain = False
    parents_link_tree.reverse()
    return parents_link_tree,linked_to_DH_chain,current_frame

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
#     children_link_tree.append(current_link)
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

def generate_camera_chains_dict_entry(name,before_chain,after_chain,DH_fixture_chain_id,DH_link_num):
    """
    @summary:  This function generates the sub dictionaries needed to construct 
    the dict eintry camera_chains in the sensors.yaml file
    @param name: The name of the chain that will be used as the sensor_id and camera_id
    @param before_chain: The static chain (set of transformations) from the base 
    uptill the DH_fixture_chain (only available if the camera is a fixed to a moving chain)
    @param DH_fixture_chain_id: the chain that is movable and fixed to the end of the before_chain
    @param after_chain: the rest of the chain until the camera has been fixed
    it is assumed that the camera is fixed at the end of the after chain 
    @param DH_link_num: if the camera is not fixed to the end of the DHchain 
    then we must give the number of the link in this parameter
    """
    camera_currnet_dict = dict()
    camera_currnet_dict["sensor_id"] = name
    camera_currnet_dict["camera_id"] = name
    camera_currnet_dict["chain"] = dict()
    camera_currnet_dict["chain"]["before_chain"] = before_chain
    camera_currnet_dict["chain"]["chains"] = DH_fixture_chain_id
    camera_currnet_dict["chain"]["after_chain"] = after_chain
    camera_currnet_dict["chain"]["dh_link_num"] = DH_link_num
    return camera_currnet_dict

def generate_chains_dict_entry(name,parent_frame,before_chain,after_chain,links,topic):
    """
    @summary:  This function generates the sub dictionaries needed to construct 
    the dict eintry chains in the sensors.yaml file
    @param name: The name of the chain that will be used as the sensor_id and chain_id
    @param before_chain: The static chain (set of transformations) from the base 
    uptill the DH_fixture_chain
    @param links: the links that make up the chain that is movable
    @param after_chain: the rest of the chain until the camera has been fixed
    it is assumed that the camera is fixed at the end of the after chain 
    @param parent_frame: the base frame of the group in this case the base frame of the robot
    then we must give the number of the link in this parameter
    @param topic: the controller state topic (this normally should not be needed)
    """
    chain_currnet_dict = dict()
    chain_currnet_dict["chain_id"] = name
    chain_currnet_dict["sensor_id"] = name
    chain_currnet_dict["parent_frame"] = parent_frame
    chain_currnet_dict["before_chain"] = before_chain
    chain_currnet_dict["after_chain"] = after_chain # at the end should be a CB
    chain_currnet_dict["links"] = links
    chain_currnet_dict["topic"] = topic
    return chain_currnet_dict

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

    '''
    @note: The following parameters should be given by the user
    @todo: The following parameters should be given by the user via a yaml file for instance
    '''
    sensor_chain_groups = [left_arm_group,right_arm_group]#,right_gripper_group,left_gripper_group]# sensor chains are groups that can hold the CB
    base_frame = "base_link" # the frame to which the group basses will be transformed to
    camera_frames = ["torso_cam3d_left_link","torso_cam3d_right_link","torso_cam3d_down_link","gripper_left_camera_link","gripper_right_camera_link"]# the sensor links 
    cb_frames = ["cb_6x9_base_link_left","cb_6x9_base_link_right"] # CB links fixed to end effector
    
    
    # initialization     
    system_dict = dict()  # this is the entry corresponding to the system yaml file
    transfomations_dict = dict()
    sensors_dict = dict() # this is the complete system that will be saved
    link_parent_map_dict = robot_urdf.parent_map # a dict with a key for each link in the URDF each entry shows the links joint and the parent of that joint
    link_child_map_dict = robot_urdf.child_map  # a dict with a key for each link in the URDF each entry shows the links joint and the children of that joint

    system_dict["dh_chains"] = generate_dh_parameters_for_dh_chains(sensor_chain_groups,robot_urdf)# this corresponds to the dh_chains in the system yaml
    #construction of transformation part of the system description
    all_group_links = dict()
    all_group_joints = dict() # not needed atm but does not hurt
    for group_current in sensor_chain_groups:
        links_of_current_group = []
        first_joint_of_group = group_current.get_joints()[0] #this returns the first element of the list of joints
#         base_link_of_currnet_group = robot_urdf.joints[first_joint_of_group].parent # the parent of the 1st joint is the base link for the group
#         links_of_current_group.append(base_link_of_currnet_group)
        links_of_current_group.extend(robot_moveit.get_link_names(group_current.get_name()))
        all_group_links[group_current.get_name()] = links_of_current_group #the dictionary that contains an entry for each group and under each entry has the list of links
        all_group_joints[group_current.get_name()] = group_current.get_joints()
        
        
        
        
#------------------------------------------------------------------------------ 
    """
            @note: this is only used for debugging 
    """
#     Dump_dict_debug = dict()# temp dict used to visualize the data coming out of the function
#     Dump_dict_debug["camera_chains"] = []# initalize a list of dicts
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
            all_camera_parent_links_dict[camera_frame],linked_to_DH_chain,current_frame = get_parents(current_frame, link_parent_map_dict, all_group_links, base_frame, True)
            transformations_dict_debug[camera_frame]["parents"] = all_camera_parent_links_dict[camera_frame]
            print "checking if %s is fixed to a DH group or static"%camera_frame
            
# this part determines the DH chain that should correspond to a sensor
            if (linked_to_DH_chain): #if one of the group_links (movable parts) is a parent of the camera_frame then there it should be in the chain dict entry for the camera_frame in sensors.yaml
                for key, value in all_group_links.items():# iterate through all the DH groups
                    if current_frame in value:# check for the current frame link in the current DH group
                        all_camera_dh_chains[camera_frame] = key # stores the name of the DH group that is a parent of the camera_frame
                        '''
                        @todo: check the index of this function
                        '''
                        all_camera_dh_chain_link_number[camera_frame] = value.index(current_frame)# store the number of the link in the parent DH gorup to which the camera_frame is fixed
                        print "found the DH parent of %s"%camera_frame
                        break
                print "the %s link is actuated and therefore should have a DHchain, it's tree has been linked to the (%s)"%(camera_frame,all_camera_dh_chains[camera_frame])
            else:
                print "the %s link is not actuated and is thus static, it's tree has been linked to the base frame (%s)"%(camera_frame,base_frame)
                all_camera_dh_chains[camera_frame] = []#"" or []  # stores an empty frame to denote that there is not DHframe between the camera/sensor and base
                all_camera_dh_chain_link_number[camera_frame] =[]#"" or [] 
            """
            @note: this is only used for debugging 
            """
            
#             temp_merge_dict = dict()
#             temp_merge_dict["sensor_id"] = camera_frame
#             temp_merge_dict["camera_id"] = camera_frame
#             temp_merge_dict["chain"] = {"before_chain":parent_link_tree,
#                                         "chain":all_camera_dh_chains[camera_frame],
#                                         "after_chain":children_link_tree}         
#             Dump_dict_debug["camera_chains"].append(temp_merge_dict) # dict entry here for specific camera_sensor_chain
        else:
            print "the link %s does not exist inside the urdf\n System Exiting" %camera_frame
            sys.exit("")
#=========================================================================
# extracting information relevant to the DH chains
#=========================================================================
    all_dh_chains_details = dict()
    for group_current in sensor_chain_groups: # for each movable group it should be transformable to the base frame
        group_current_links = robot_moveit.get_link_names(group_current.get_name())
        first_link_of_group = group_current_links[0]#first link
        last_link_of_group = group_current_links[-1]#last link of group
        transformations_dict_debug[group_current.get_name()] = dict()
        all_dh_chains_details[group_current.get_name()] = dict()
# starting the extraction of children for the current DH group
        current_link_in_grp = last_link_of_group
        if robot_urdf.links.has_key(last_link_of_group):
            all_dh_chains_details[group_current.get_name()]["children"] = get_children(last_link_of_group, link_child_map_dict)
            transformations_dict_debug[group_current.get_name()]["children"] = all_dh_chains_details[group_current.get_name()]["children"]
# starting the extraction of parents for the current DH group 
        current_link_in_grp = first_link_of_group           
        if robot_urdf.links.has_key(first_link_of_group):
            all_dh_chains_details[group_current.get_name()]["parents"],linked_to_DH_chain,current_link_in_grp = get_parents(current_link_in_grp, link_parent_map_dict, all_group_links, base_frame, False)
            transformations_dict_debug[group_current.get_name()]["parents"] = all_dh_chains_details[group_current.get_name()]["parents"]

#===============================================================================
# Generation of the sensors.yaml file
#===============================================================================

# Generation of the camera_chains entry
    sensors_dict["camera_chains"] = []
    '''
    @note: watch out with appends as they affect all the lists , pointers therfore use a new list 
    '''
    for camera_current in camera_frames:
        # the before chain is the transformation from the base of the DH_group to the base
        #base->base_of_DH_chain = before_chain, base_of_DH_chain-> tip_of_DH_chain = DH_chain (variable), tip_of_DH_chain -> camera = after_chain
        if all_camera_dh_chains[camera_current] != []: # if the camera has a DH_chain
            name_of_dh_group = all_camera_dh_chains[camera_current]
            before_chain = all_dh_chains_details[name_of_dh_group]["parents"]#then set the before chain to be the transformation from the base base->base_of_DH_chain
#             before_chain = before_chain + all_camera_parent_links_dict[camera_current] # appending the lists together
#             after_chain = []#all_camera_children_links_dict[camera_current]#[]
            after_chain = list(all_camera_parent_links_dict[camera_current]) # the after chain are the parents of the camera link up to the start of the DH_chain
            after_chain.append(camera_current)# add the frame at the end of the chain
        else:
            before_chain = all_camera_parent_links_dict[camera_current] # if there is no DH chain then the parent of the camera link should be the base
            after_chain = [camera_current]#all_camera_children_links_dict[camera_current]#[]
#         pdb.set_trace()
        camera_currnet_dict = generate_camera_chains_dict_entry(camera_current,
                                                                before_chain,
                                                                after_chain,
                                                                all_camera_dh_chains[camera_current],
                                                                all_camera_dh_chain_link_number[camera_current])
        sensors_dict["camera_chains"].append(camera_currnet_dict)
        
# Generation of the sensor_chains entry, chains that can hold  CB


# Generation of the chains entry, chains that can move  
#in willow garage chains exist but not sensor chains as they assume that each chain can hold a CB
    sensors_dict["chains"] = []
    for chain in sensor_chain_groups:
        topic = '' # left empty for now
#         before_chain = copy.deepcopy(all_dh_chains_details[chain.get_name()]) Used to look at the yaml file without memory locations
        chain_currnet_dict = generate_chains_dict_entry(chain.get_name(),
                                                        base_frame,
                                                        all_dh_chains_details[chain.get_name()]["parents"],#before_chain["parents"],
                                                        all_dh_chains_details[chain.get_name()]["children"],
                                                        all_group_links[chain.get_name()],
                                                        topic)
        sensors_dict["chains"].append(chain_currnet_dict) 

    """
    @todo: Write a method that takes these dicts and produces the proper sensor.yaml
    a fixed camera can be treated as an actuated camera with an empty linkage
    
    @todo: remember to organize the names of the chains and sensors such that they match the bag file
    
    @todo: check the situation in which 2 DHcains are linked to each other as in the DH chain of a sensor should be a list and not just one single chain
    """
#===============================================================================
# generate the transofmation dict list needed for the DH groups this is used to define to robot
#===============================================================================
    system_dict["transforms"] = dict()
    sanity_check_dict = dict()
    for sensor_chain in sensor_chain_groups:# for each sensor chain
        name_of_sensor_chain = sensor_chain.get_name()

#extract transformations for children
        extraction_chain = all_dh_chains_details[name_of_sensor_chain]["children"]
        end_link_of_group = all_group_links[sensor_chain.get_name()][-1]
        transformations_of_current_chain,sanity_check_dict = extract_transformations(extraction_chain, listener, sanity_check_dict,end_link_of_group,None)# list of rotations
        system_dict["transforms"].update(transformations_of_current_chain)
        
#extract transformations for parents
        extraction_chain = all_dh_chains_details[name_of_sensor_chain]["parents"]
        transformations_of_current_chain,sanity_check_dict = extract_transformations(extraction_chain, listener, sanity_check_dict,None,None)# list of rotations
        system_dict["transforms"].update(transformations_of_current_chain)    
    for camera_frame in camera_frames: # for each camera frame 

#extract transformations for children
        extraction_chain = all_camera_children_links_dict[camera_frame]
        transformations_of_current_chain,sanity_check_dict = extract_transformations(extraction_chain, listener, sanity_check_dict,None,None)# list of rotations
        system_dict["transforms"].update(transformations_of_current_chain)

#extract transformations for parents   
        extraction_chain = all_camera_parent_links_dict[camera_frame]
        transformations_of_current_chain,sanity_check_dict = extract_transformations(extraction_chain, listener, sanity_check_dict,None,camera_frame)# list of rotations
        system_dict["transforms"].update(transformations_of_current_chain)
        
#------------------------------------------------------------------------------ 
# final check for verification check that all transformations mentioned on the sensor.yaml are present in the system.yaml
    for key, array_of_dicts in sensors_dict.iteritems(): #for each entry in sensors
        not_present_transforamtions = []
        for dictt in array_of_dicts:
            if key == 'camera_chains':
                dict_to_work_on = dictt['chain']
            elif key == 'chains':
                dict_to_work_on = dictt
            elif key == 'sensor_chains':
                dict_to_work_on = dictt
            else:
                print 'There is a key (%s) inside the sensor_dict which corresponds to the sensor.yaml which is not defined'%key
                sys.exit("")
            for key_work_on, list_in_key in dict_to_work_on.iteritems():
                if key_work_on == 'after_chain' or key_work_on == 'before_chain' :
                    for value in list_in_key:
                        if value != base_frame and value not in system_dict["transforms"]:
                            not_present_transforamtions.append(value)
    if not_present_transforamtions == []:
        print "All transformations were found"
    else:
        print "the following transformations were not fount: "
        print not_present_transforamtions
#------------------------------------------------------------------------------ 
# file dumping
    output_system = os.path.abspath("IDEcatkin_ws/src/cob_calibration/cob_calibration_executive/test/sensor_trial.yaml")
    with open(output_system, 'w') as f:
        f.write('####### This file is autogenerated. Do not edit #######\n')
        f.write(yaml.dump(sensors_dict))  
    output_system = os.path.abspath("IDEcatkin_ws/src/cob_calibration/cob_calibration_executive/test/debug_tree.yaml")
    with open(output_system, 'w') as f:
        f.write('####### This file is autogenerated. Do not edit #######\n')
        f.write(yaml.dump(transformations_dict_debug))  
    output_system = os.path.abspath("IDEcatkin_ws/src/cob_calibration/cob_calibration_executive/test/system_trial.yaml")
    with open(output_system, 'w') as f:
        f.write('####### This file is autogenerated. Do not edit #######\n')
        f.write(yaml.dump(system_dict))
    output_system = os.path.abspath("IDEcatkin_ws/src/cob_calibration/cob_calibration_executive/test/sanity_check_transformations.yaml")
    with open(output_system, 'w') as f:
        f.write('####### This file is autogenerated. Do not edit #######\n')
        f.write(yaml.dump(sanity_check_dict))

    print '==============================================================================='
    print "this is the end of the camera tree loop"
    print "note that there is a pdb at the end of the code preventing termination"
    print '==============================================================================='
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
    @attention: This are the functions and parameters of the URDF joints keys origin
    """
# robot_urdf.joints['key'].origin.parse     robot_urdf.joints['key'].origin.position  robot_urdf.joints['key'].origin.rotation  robot_urdf.joints['key'].origin.to_xml


    pdb.set_trace()





if __name__ == '__main__':
    main()
    print "==> done exiting"