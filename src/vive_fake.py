#!/usr/bin/env python2
import rospy

from geometry_msgs.msg import PoseStamped




def main():
    global pub1
    rospy.init_node('vive_fake', anonymous=True)
    pub1 = rospy.Publisher("/mavros/mocap/pose",PoseStamped,queue_size=10)


    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        poseStamp.header.stamp = rospy.Time.now()
        pub1.publish(poseStamp)

        rate.sleep()


if __name__ == '__main__':
    pub1 = None
    poseStamp = PoseStamped()
    poseStamp.header.frame_id = 'world'

    poseStamp.pose.position.x = 1
    poseStamp.pose.position.y = 2
    poseStamp.pose.position.z = 3
    poseStamp.pose.orientation.w = 1

    main()
