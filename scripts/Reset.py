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
    move_arm.wait_for_server()
    rospy.loginfo("Connected to action server")
    
    goal = ArmTaskGoal()
    goal.goal_type = "reset"
    goal.remove_tip_frame = "grippoint_left"
    goal.remove_root_frame = "base_link"
    
    result = move_arm.send_goal(goal)
    
    goal = ArmTaskGoal()
    goal.goal_type = "reset"
    goal.remove_tip_frame = "grippoint_right"
    goal.remove_root_frame = "base_link"
    
    result = move_arm.send_goal(goal)
    
