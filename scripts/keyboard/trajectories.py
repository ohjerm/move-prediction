#!/usr/bin/env python
"""
we need to keep the user's path seperate, as the robot can otherwise enter a
loop wherein it's movement is based on its own previous movement
also, we need to remap user input to the robot to legible positions
"""

import rospy
from geometry_msgs.msg import Vector3, Point
from move_prediction.msg import VectorArr

current_position = Vector3()

pub = None
start_pos = Point()

# list_10_sec = [start_pos] * 1039
list_10_sec = [start_pos] * 520
 

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
    """
    called as often as keyboard/input is published, meaning at constant 60hz
    there is no need to keep a loop running
    """
    global current_position
    global list_10_sec
    
    # remap the data to z = forw/back, x = right/left, and y is up/down
    remap = Point()
    remap.x = -data.y * 0.000333333
    remap.z = data.x * 0.000333333
    remap.y = data.z * 0.000333333
    
    # the current position is last frame + old position
    current_position = add_vectors(current_position, remap)
    
    # five vectors are created based on an internal timing test
    vec_1_sec = get_vector(current_position, list_10_sec[55])
    vec_2_sec = get_vector(current_position, list_10_sec[97])
    vec_5_sec = get_vector(current_position, list_10_sec[135])
    vec_10_sec = get_vector(current_position, list_10_sec[267])
    vec_14_sec = get_vector(current_position, list_10_sec.pop())

    #create a clone of the current position (as it is global) to add to list
    clone = Vector3()
    clone.x = current_position.x
    clone.y = current_position.y
    clone.z = current_position.z
    list_10_sec.insert(0, clone)
    
    #ship it
    to_ret=VectorArr()
    to_ret.Array=[vec_1_sec,vec_2_sec,vec_5_sec,vec_10_sec,vec_14_sec]
    pub.publish(to_ret)

if __name__ == '__main__':
    rospy.init_node('trajectories', anonymous=False)
    pub = rospy.Publisher("keyboard/trajectories", VectorArr, queue_size=1)
    rospy.Subscriber('keyboard/input', Vector3, callback)
    rospy.spin()
    
