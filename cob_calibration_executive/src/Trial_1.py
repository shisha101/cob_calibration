#!/usr/bin/env python
PKG = 'cob_calibration_executive'
NODE = 'InformationParserNode'
import sys
import moveit_commander
import rospy
import pdb
from urdf_parser_py.urdf import URDF
# import roslib
# import yaml

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.sleep(10)
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
    sensor_chain_groups = [left_arm_group,right_arm_group]
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
    system_dict = dict()  # this is the entry corresponding to dh_chains
    sensors_dict = dict() # this is the complete system that will be saved
    pdb.set_trace()
    
    
    
    
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
            if current_joint.joint_type.axis == "1 0 0":
                type_of_joint += "x"
            elif current_joint.joint_type.axis == "0 1 0":
                type_of_joint += "y"
            elif current_joint.joint_type.axis == "0 0 1":
                type_of_joint += "z"
            sensor_list.append({"name":current_joint.name,
                                "type": type_of_joint,
                                "xyzrpy": current_joint.position + current_joint.orientation
                                                       })
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
    system_dict['checkerboards'] = rospy.get_param('~checkerboards', None)
    if system_dict['checkerboards'] is None:
        print '[ERROR]: Parameter checkerboards not found. Make sure it is set and try again'
        return

#     robot_urdf.joints['arm_right_4_joint'].joint_type
#     robot_moveit.get_current_variable_values()
#     left_arm_group.get_current_pose() works with any link
    
    pdb.set_trace()
    
    print "============ \nPrinting information about the %s group" %right_arm_group.get_name()
    print "============ \nReference frame: %s" % right_arm_group.get_planning_frame()
    print "============ \nEnd effector frame: %s" % right_arm_group.get_end_effector_link()
    print "============ \nCurrent joint angles: %s\n" % right_arm_group.get_current_joint_values()
    pdb.set_trace()





if __name__ == '__main__':
    main()
    print "==> done exiting"