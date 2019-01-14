#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2 as pc2
import numpy as np
import struct
from math import sqrt
from itertools import izip, count
from multiprocessing import Pool, cpu_count
import time
import os

pub = rospy.Publisher('camera/depth/color/filtered_points', PointCloud2, queue_size=1)
rospy.init_node('distance_filter', anonymous=False)

def callback(data):
    global init_timer
    global loop_timer
    global ends_timer

    #built-in function to read points from a cloud
    generator = pc2.read_points(data, skip_nans=True)

    #generate a list of points if each axis is less than 1 meter from the camera
    new_array = [point for point in generator if abs(point[0]) < 1 and abs(point[1]) < 1 and abs(point[2]) < 1]

    #generate a new cloud with the old header and fields but with the new array
    new_cloud = pc2.create_cloud(data.header, data.fields, new_array)

    pub.publish(new_cloud)



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)
    # rospy.Subscriber('/camera/depth/image_rect_raw', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
