#!/usr/bin/env python

import keyboard
import rospy
import getch
from geometry_msgs.msg import Vector3

def publisher():
    pub = rospy.Publisher('keyboard/input', Vector3, queue_size=10)
    rospy.init_node('keyboard_input', anonymous=False)
    rate = rospy.Rate(60)
    
    out = Vector3()
    out.x = 0
    out.y = 0
    out.z = 0
    
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
        
        pub.publish(out)
        rate.sleep()
            
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
    
