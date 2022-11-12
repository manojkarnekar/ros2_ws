#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, Pose

def position(x,y,o_z, w):
    somePose = Pose()
    somePose.position.x = x
    somePose.position.y = y
    somePose.position.z = 0.0

    somePose.orientation.x = 0.0
    somePose.orientation.y = 0.0
    somePose.orientation.z = o_z
    somePose.orientation.w = w
    poseArray.poses.append(somePose)

if __name__ == '__main__':
    rospy.init_node('pose_array')
    r = rospy.Rate(60.0)
    pub = rospy.Publisher("/poseArrayTopic", PoseArray, queue_size = 2)
    while not rospy.is_shutdown():
        poseArray = PoseArray()
        poseArray.header.stamp = rospy.Time.now()
        poseArray.header.frame_id = "map"
        for i in range(0, 5):
            position(i, 0.0, 0.0, 1)

        pub.publish(poseArray)
        r.sleep()