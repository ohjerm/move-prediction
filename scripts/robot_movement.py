#!/usr/bin/env python

import socket
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

robot = None
scene = None
group = None

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def send_starting_pos():
    global client
    g = MoveBaseAction()
    g.target_pose.header.frame_id = "base_link"
    g.target_pose.header.stamp = rospy.Time.now()
    
    q = Quaternion(quaternion_from_euler(0,0,0, axes="sxyz"))
    g.target_pose.pose = Pose(0.314,0.120,0.144, q)
        
    rospy.loginfo("greetings")
    
    try:
        client.send_goal(g)
        client.wait_for_server()
        
        rospy.loginfo("ready for input")
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def callback(data):
    rospy.loginfo(data)
    

def main():
    global robot
    global scene
    global group
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_movement', anonymous=False, disable_signals=True)
        
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("left_arm")
        
        send_starting_pos()
        
        rospy.Subscriber('keyboard/input', Vector3, callback)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
        raise
        
    
    

if __name__ == '__main__':
    main()