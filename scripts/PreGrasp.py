#! /usr/bin/env python
import roslib; roslib.load_manifest('amigo_whole_body_controller')
import rospy
import tf
import actionlib
from amigo_whole_body_controller.msg import *
from arm_navigation_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
#from amigo_arm_navigation.msg._grasp_precomputeGoal import grasp_precomputeGoal
#from amigo_arm_navigation.msg._grasp_precomputeAction import grasp_precomputeAction

def euler_z_to_quaternion(roll, pitch, yaw):
    
    rospy.logwarn("These should be euler angles")
    orientation_goal = geometry_msgs.msg.Quaternion()
    
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    orientation_goal.x = quaternion[0]
    orientation_goal.y = quaternion[1]
    orientation_goal.z = quaternion[2]
    orientation_goal.w = quaternion[3]
    
    return orientation_goal

if __name__ == '__main__':

    rospy.init_node('add_motion_objective')

    move_arm = actionlib.SimpleActionClient("add_motion_objective", ArmTaskAction)
    #move_arm.wait_for_server()
    #move_arm.wait_for_server()
    rospy.loginfo("Connected to action server")
    
    marker_pub = rospy.Publisher('/visualization_marker', Marker)
    
    if (sys.argv[1] == "free"):
        do_constrain = False
    else:
       do_constrain = True

    goal = ArmTaskGoal()
    #rospy.loginfo(goal)
    goal.goal_type = "grasp"
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "base_link"
    position_constraint.link_name = "grippoint_left"
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0
    position_constraint.position.x = 0.5
    position_constraint.position.y = 0.2
    position_constraint.position.z = 0.8
    position_constraint.target_point_offset.x = 0.2
    goal.position_constraint = position_constraint
    rospy.logwarn("Position constraint region shapes etc. not yet defined")
    
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "base_link"
    orientation_constraint.link_name = "grippoint_left"
    orientation_constraint.orientation = euler_z_to_quaternion(0, 0, 0)
    goal.orientation_constraint = orientation_constraint
    rospy.loginfo("Type link or header not yet taken into account")
    rospy.logwarn("Orientation constraint tolerances etc not yet defined")

    goal.stiffness.force.x = 70.0
    goal.stiffness.force.y = 60.0
    goal.stiffness.torque.x = 5.0
    goal.stiffness.torque.y = 5.0
    if (do_constrain):
        goal.stiffness.force.z = 50.0
        goal.stiffness.torque.z = 0.0
    else:
        goal.stiffness.force.z = 0.0
        goal.stiffness.torque.z = 0.0
    
    rospy.loginfo(goal)
    
    goal_marker = Marker()
    goal_marker.header = goal.position_constraint.header
    goal_marker.id = 5432
    goal_marker.type = 0 # Arrow
    goal_marker.pose.position = goal.position_constraint.position
    goal_marker.pose.orientation = goal.orientation_constraint.orientation
    goal_marker.scale.x = 0.5
    goal_marker.scale.y = 0.5
    goal_marker.scale.z = 0.2
    goal_marker.color.r = 1.0
    goal_marker.color.g = 0.0
    goal_marker.color.b = 0.0
    goal_marker.color.a = 1.0
    goal_marker.lifetime = rospy.Duration(5.0)

    rospy.loginfo(goal_marker)

    ctr = 0;
    while (not rospy.is_shutdown() and ctr < 10):
        marker_pub.publish(goal_marker)
        rospy.sleep(rospy.Duration(0.1))
        ctr = ctr + 1
        rospy.sleep(0.02)
        
    #actionClients.move_arm.send_goal_and_wait(goal, rospy.Duration(time_out))
    result = move_arm.send_goal(goal)
    rospy.loginfo("Result = {0}".format(result))
    
