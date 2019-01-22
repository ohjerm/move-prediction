#!/usr/bin/env python

import socket
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

HOST = "169.254.178.76"     # remote host
PORT = 30002                # port used by server

client = None

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST, PORT))

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def send_starting_pos():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
    
        g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=[0.25,0.197,0.149,0,0,0], velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
        client.send_goal(g)
        client.wait_for_server()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def callback(data):
    rospy.loginfo(data)
    

def main():
    global client
    try:
        rospy.init_node('robot_movement', anonymous=False, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        
        send_starting_pos()
        
        rospy.Subscriber('keyboard/input', Vector3, callback)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
        raise
        
    
    

if __name__ == '__main__':
    main()