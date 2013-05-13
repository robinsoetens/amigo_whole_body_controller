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

    #move_arm = actionlib.SimpleActionClient("add_motion_objective")
    #move_arm.wait_for_server()
    rospy.loginfo("Connected to action server")
    
    marker_pub = rospy.Publisher('/visualization_marker', Marker)

    goal = ArmTaskGoal()
    rospy.loginfo(goal)
    goal.goal_type = "grasp"
    #goal.goal_constraints.position_constraints[0].header.frame_id = "/base_link"
    #goal.goal_constraints.position_constraints[0].link_name = "/grippoint_left"
    #goal.goal_constraints.position_constraints[0].target_point_offset.x = 0.0
    #goal.goal_constraints.position_constraints[0].target_point_offset.y = 0.0
    #goal.goal_constraints.position_constraints[0].target_point_offset.z = 0.0
    #goal.goal_constraints.position_constraints[0].position.x = sys.argv[1]
    #goal.goal_constraints.position_constraints[0].position.y = sys.argv[2]
    #goal.goal_constraints.position_constraints[0].position.z = sys.argv[3]
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "/base_link"
    position_constraint.link_name = "/grippoint_left"
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0
    position_constraint.position.x = float(sys.argv[1])
    position_constraint.position.y = float(sys.argv[2])
    position_constraint.position.z = float(sys.argv[3])
    goal.goal_constraints.position_constraints.append(position_constraint)
    rospy.logwarn("Position constraint region shapes etc. not yet defined")
    
    #goal.goal_constraints.orientation_constraints[0].header.frame_id = "/base_link"
    #goal.goal_constraints.orientation_constraints[0].link_name = "/grippoint_left"
    #goal.goal_constraints.orientation_constraints[0].orientation = euler_z_to_quaternion(sys.argv[4],sys.argv[5],sys.argv[6])
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "/base_link"
    orientation_constraint.link_name = "/grippoint_left"
    orientation_constraint.orientation = euler_z_to_quaternion(float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6]))
    goal.goal_constraints.orientation_constraints.append(orientation_constraint)
    rospy.loginfo("Type link or header not yet taken into account")
    rospy.logwarn("Orientation constraint tolerances etc not yet defined")

    goal.stiffness.force.x = 1.0
    goal.stiffness.force.y = 1.0
    goal.stiffness.force.z = 1.0
    goal.stiffness.torque.x = 1.0
    goal.stiffness.torque.y = 1.0
    goal.stiffness.torque.z = 1.0
    
    rospy.loginfo(goal)
    
    goal_marker = Marker()
    goal_marker.header = goal.goal_constraints.position_constraints[0].header
    goal_marker.id = 5432
    goal_marker.type = 0 # Arrow
    goal_marker.pose.position = goal.goal_constraints.position_constraints[0].position
    goal_marker.pose.orientation = goal.goal_constraints.orientation_constraints[0].orientation
    goal_marker.scale.x = 0.1
    goal_marker.scale.y = 0.1
    goal_marker.scale.z = 0.1
    goal_marker.color.r = 1.0
    goal_marker.color.g = 0.0
    goal_marker.color.b = 0.0
    goal_marker.color.a = 1.0
    goal_marker.lifetime = rospy.Duration(5.0)

    rospy.loginfo(goal_marker)

    while not rospy.is_shutdown():
        marker_pub.publish(goal_marker)
        rospy.sleep(rospy.Duration(0.1))
    marker_pub.publish(goal_marker)
    '''
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing target pose")
        targetPubLeft.publish(goalPoseLeft)
        
        rospy.loginfo("Published target pose")
        rospy.sleep(0.1)
        
    rospy.loginfo("Finished")
    '''
    
    

    
