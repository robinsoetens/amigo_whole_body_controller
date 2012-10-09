#! /usr/bin/env python
import roslib; roslib.load_manifest('amigo_whole_body_controller')
import rospy
import tf
#import actionlib
#from arm_navigation_msgs.msg import *
#from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':

    rospy.init_node('publish_target_pose_left')
    
    targetPubLeft = rospy.Publisher('/arm_left_target_pose', PoseStamped)
    
    ''' Left '''
    targetx = 0.45
    targety = 0.3
    targetz = 0.46
    targetRoll = 0
    targetPitch = 0
    targetYaw = 0
    # Not really correct, should be roll-pitch-yaw instead of Euler
    [rx,ry,rz,rw] = tf.transformations.quaternion_from_euler(targetRoll,targetPitch,targetYaw) 
    
    goalPose = PoseStamped();
    goalPose.header.frame_id = "/base_link"
    goalPose.pose.position.x = targetx
    goalPose.pose.position.y = targety
    goalPose.pose.position.z = targetz
    goalPose.pose.orientation.x = rx;
    goalPose.pose.orientation.y = ry;
    goalPose.pose.orientation.z = rz;
    goalPose.pose.orientation.w = rw; 
    
    rospy.loginfo(goalPose)
    
    goalPoseLeft = goalPose
    
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing target pose")
        targetPubLeft.publish(goalPoseLeft)
        
        rospy.loginfo("Published target pose")
        rospy.sleep(0.1)
        
    rospy.loginfo("Finished")
    
    
    #side = rospy.get_param("~side", "left")

    
