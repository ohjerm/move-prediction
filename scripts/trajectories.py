#!/usr/bin/env python
"""
we need to keep the user's path seperate, as the robot can otherwise enter a
loop wherein it's movement is based on its own previous movement
also, we need to remap user input to the robot to legible positions
"""

import rospy
from geometry_msgs.msg import Vector3, Point


current_position = Vector3()
list_10_sec = [Vector3()] * 600


def add_vectors(vec1, vec2):
    ret = Vector3()
    ret.x = vec1.x + vec2.x
    ret.y = vec1.y + vec2.y
    ret.z = vec1.z + vec2.z    
    return ret


def get_vector(current_pos, old_pos):
    ret = Vector3()
    ret.x = current_pos.x - old_pos.x
    ret.y = current_pos.y - old_pos.y
    ret.z = current_pos.z - old_pos.z
    return ret


def callback(data):
    global current_position
    global list_10_sec
    
    # remap the data to z = forw/back, x = right/left, and y is up/down
    remap = Vector3()
    remap.x = -data.y
    remap.z = data.x
    remap.y = data.z
    
    # the current position is last frame + old position
    current_position = add_vectors(current_position, remap)
    
    # four vectors are created resembling the movement from 1,2,5, and 10 sec
    vec_1_sec = get_vector(current_position, list_10_sec[60])
    vec_2_sec = get_vector(current_position, list_10_sec[120])
    vec_5_sec = get_vector(current_position, list_10_sec[300])
    vec_10_sec = get_vector(current_position, list_10_sec.pop())

    #create a clone of the current position (as it is global) to add to list
    clone = Vector3()
    clone.x = current_position.x
    clone.y = current_position.y
    clone.z = current_position.z
    list_10_sec.insert(0, clone)
    
    # rospy.loginfo(clone)
    rospy.loginfo("X: \t" + str(vec_1_sec.x) + "\t" + str(vec_2_sec.x) + "\t" + str(vec_5_sec.x) + "\t" + str(vec_10_sec.x) + "\t")
    rospy.loginfo("Y: \t" + str(vec_1_sec.y) + "\t" + str(vec_2_sec.y) + "\t" + str(vec_5_sec.y) + "\t" + str(vec_10_sec.y) + "\t")
    rospy.loginfo("Z: \t" + str(vec_1_sec.z) + "\t" + str(vec_2_sec.z) + "\t" + str(vec_5_sec.z) + "\t" + str(vec_10_sec.z) + "\t")


if __name__ == '__main__':
    rospy.init_node('trajectories', anonymous=False)
    rospy.Subscriber('keyboard/input', Vector3, callback)
    rospy.spin()
    