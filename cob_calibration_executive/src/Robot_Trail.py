#!/usr/bin/env python
PKG = 'cob_calibration_executive'
NODE = 'Robot Test node'
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    print "============ Robot Groups:"
    print robot.get_group_names()
    #group = moveit_commander.MoveGroupCommander("left_arm") # Rename the group
    ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    
    group.set_pose_target(pose_target) # Set the end target for the movement
    

if __name__ == '__main__':
    main()
    print "==> done exiting"
