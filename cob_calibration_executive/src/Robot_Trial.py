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
    rospy.sleep(10)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============"
    group = moveit_commander.MoveGroupCommander("arm_left") # Rename the group
    ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    
    
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
    
    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ End effector frame: %s" % group.get_end_effector_link()
    print "============ Current joiny angles: %s" % group.get_current_joint_values()
#     group.set_pose_target() # Set the end target for the movement
    # angle calculation
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = 0.712
    pose_target.position.y = 0.347
    pose_target.position.z = 1.159
    pose_target.orientation.x = 0.092
    pose_target.orientation.y = -0.0619
    pose_target.orientation.z = -0.550
    pose_target.orientation.w = 0.827
    group.set_pose_target(pose_target)
    group.set_start_state_to_current_state()
#     #angle calculation
#      group_variable_values = group.get_current_joint_values()
#      group_variable_values[5] = 0.5
#      group.set_joint_value_target(group_variable_values)
    plan1 = group.plan()
    rospy.sleep(5)
    print "============ executing Path"
    group.go(wait=True)
#     print "============ Visualizing plan1"
#     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#     display_trajectory.trajectory_start = robot.get_current_state()
#     display_trajectory.trajectory.append(plan1)
#     display_trajectory_publisher.publish(display_trajectory)

if __name__ == '__main__':
    main()
    print "==> done exiting"
