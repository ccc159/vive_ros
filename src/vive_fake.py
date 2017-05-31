#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


def callback(msg):
    global pub,poseStamp
    pose = msg.data
    poseStamp.pose = pose
    poseStamp.header.stamp = rospy.Time.now()
    pub.publish(poseStamp)


def main():
    global pub
    rospy.init_node('vive_fake', anonymous=True)
    pub = rospy.Publisher("/mavros/mocap/pose",PoseStamped,queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub.publish(poseStamp)
    rate.sleep()



if __name__ == '__main__':
    pub = None
    poseStamp = PoseStamped()
    poseStamp.header.frame_id = 'world'

    poseStamp.pose.position.x = 1
    poseStamp.pose.position.y = 2
    poseStamp.pose.position.z = 3
    poseStamp.pose.orientation.w = 1

    main()
