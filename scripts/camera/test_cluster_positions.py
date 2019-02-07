#!/usr/bin/env python
import rospy
from move_prediction.msg import PointArr


def callback(data):
    rospy.loginfo(len(data.Array))


def init():
    rospy.Subscriber("camera/depth/color/cluster_positions", PointArr, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("test_points_node")
    init()