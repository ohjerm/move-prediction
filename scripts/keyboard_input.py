#!/usr/bin/env python

import keyboard
import rospy
import getch
from geometry_msgs.msg import Vector3

INTERP_FACTOR = 1. / 30.  # publish at 60, so going from 1 to 0 takes .5 sec

interp_vec = Vector3()


def clamp(val, _min, _max):
    return max(min(val, _max), _min)


def publisher(interpolate):
    global interp_vec
    global INTERP_FACTOR
    
    pub = rospy.Publisher('keyboard/input', Vector3, queue_size=1)
    rate = rospy.Rate(60)
    
    out = Vector3()
    
    while not rospy.is_shutdown():
        if keyboard.is_pressed('esc'):
            rospy.signal_shutdown("Hooked keyboard is no longer needed")
        
        if keyboard.is_pressed(72):
            out.x = 1
        elif keyboard.is_pressed(76):
            out.x = -1
        else:
            out.x = 0
            
        if keyboard.is_pressed(75):
            out.y = 1
        elif keyboard.is_pressed(77):
            out.y = -1
        else:
            out.y = 0
        
        if keyboard.is_pressed(73):
            out.z = 1
        elif keyboard.is_pressed(71):
            out.z = -1
        else:
            out.z = 0
            
        if interpolate:
            out.x = clamp(out.x, interp_vec.x - INTERP_FACTOR, interp_vec.x + INTERP_FACTOR)
            out.y = clamp(out.y, interp_vec.y - INTERP_FACTOR, interp_vec.y + INTERP_FACTOR)
            out.z = clamp(out.z, interp_vec.z - INTERP_FACTOR, interp_vec.z + INTERP_FACTOR)
            interp_vec.x = out.x
            interp_vec.y = out.y
            interp_vec.z = out.z
        
        pub.publish(out)
        rate.sleep()
            
if __name__ == '__main__':
    try:
        rospy.init_node('keyboard_input', anonymous=False)
        publisher(True)
    except rospy.ROSInterruptException:
        pass
    
