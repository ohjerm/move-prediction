#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Vector3
from move_prediction.msg import Goal
from std_msgs.msg import String

pub = None
mode = ''

def flatten(value, minimum, maximum):
    if value < minimum:
        return 0.
    else:
        return clamp(value, minimum, maximum)

def clamp(value, minimum, maximum):
    return min(max(value, minimum), maximum)

def cb_prediction(data):
    global pub
    global mode
    
    #linear arbitration ( no difference )
    lin_goal = Goal()
    lin_goal.point = data.point
    lin_goal.confidence = data.confidence
    
    timid_goal = Goal()
    timid_goal.point = data.point
    timid_goal.confidence = clamp(data.confidence - 0.5, 0., 0.5)

    aggressive_goal = Goal()
    aggressive_goal.point = data.point
    aggressive_goal.confidence = flatten(data.confidence, 0.5, 1.)
    
    if mode == 'timid' or mode == 't':
        pub.publish(timid_goal)
    elif mode == 'aggressive' or mode == 'a':
        pub.publish(lin_goal)
    
def cb_mode(data):
    global mode
    mode = data.data

if __name__ == '__main__':
    # global pub
    rospy.init_node('arbitration_node', anonymous=False)
    # mode = sys.argv[1]
    # rospy.loginfo(sys.argv)
    pub = rospy.Publisher("arbitration/final_goal", Goal, queue_size=1)
    rospy.Subscriber("arbitration/prediction", Goal, cb_prediction)
    rospy.Subscriber("keyboard/arbitration_mode", String, cb_mode)
    rospy.spin()
