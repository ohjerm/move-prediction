#!/usr/bin/env python

import rospy
import os

if __name__ == '__main__':
    os.seteuid(0)
    rospy.init_node('test', anonymous=True)
    rospy.loginfo('adsfdsaf')
    rospy.loginfo(os.geteuid())
