#!/usr/bin/env python2
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
    rospy.init_node('vive_bridge', anonymous=True)
    pub = rospy.Publisher("/mavros/mocap/pose",PoseStamped,queue_size=10)
    rospy.Subscriber("/vive/pose", Pose, callback)
    rospy.spin()


if __name__ == '__main__':
    pub = None
    poseStamp = PoseStamped()
    poseStamp.header.frame_id = 'world'
    main()
