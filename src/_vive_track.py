#!/usr/bin/env python3
import _openvr_wrapper
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion



if __name__ == '__main__':
    #initial ros node
    rospy.init_node('vive_pose', anonymous=True)
    pub = rospy.Publisher("/vive_pose", PoseStamped, queue_size=10)

    #set refresh rate
    rate = rospy.Rate(120)

    #initial vive
    vive = _openvr_wrapper.OpenvrWrapper()
    rospy.loginfo("=== vive ros started ===")
    vive.print_discovered_objects()
    vive_poseStamped = PoseStamped()
    vive_poseStamped.header.frame_id = 'world'

    # get the poseStamped from pose
    while not rospy.is_shutdown():
        translation,quaternion = vive.devices["tracker_1"].get_pose_quaternion()
        vive_poseStamped.pose.position = Point(*translation)
        vive_poseStamped.pose.orientation = Quaternion(*quaternion)
        vive_poseStamped.header.stamp = rospy.Time.now()
        pub.publish(vive_poseStamped)

    rate.sleep()
