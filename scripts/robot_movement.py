#!/usr/bin/env python

"""
This is a variation of stefan's code, keyboard-control.py
The original repo is found here: https://bitbucket.org/phdstefan/ur3-keyboard-control/src/master/
Since our needs and the previous work we have are a bit different, we remake some here
"""

import numpy as np
import rospy
import sys
import moveit_commander
import tf.transformations
import time
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from geometry_msgs.msg import PoseStamped, Vector3
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# Global vars
prev_state = None
mode = "position"
mode_timer = time.time()
ik_srv = None
pub = None
robot = None
scene = None
group = None
req = None



def init():
    global pub
    global robot
    global scene
    global group
    global ik_srv
    global req
    global prev_state
    # Setup moveit stuff
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('keyboard-control_node')
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = 'manipulator'
    group = moveit_commander.MoveGroupCommander(group_name)
    
    prev_state = robot.get_current_state().joint_state.position
    
    # Insert table in planning scene
    rospy.sleep(1)
    scene.remove_world_object('ground_plane')
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.0
    p.pose.position.y = 0.0
    p.pose.position.z = -0.035
    scene.add_box('ground_plane', p, (2.0, 2.0, 0.02))
    rospy.sleep(1)
    
    # Setup services and publishers
    ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

    # Prepare IK req
    req = GetPositionIKRequest()
    req.ik_request.timeout = rospy.Duration(0.01)
    req.ik_request.attempts = 2
    req.ik_request.avoid_collisions = True
    req.ik_request.group_name = group_name
    

def publish_pose(pose, speed):
    global prev_state
    global req
    global pub
    
    # Send request for new pose to IK (inverse kinematics) service
    req.ik_request.pose_stamped = pose
    resp = ik_srv.call(req)
    
    # Check if valid robot configuration can be found
    if(resp.error_code.val == -31):
        print('No IK found!')
        return
    
    # Check if we are near a singularity
    # Skip movement if that is the case
    diff = np.array(resp.solution.joint_state.position)-np.array(prev_state)
    max_diff = np.max(diff)
    if(max_diff > 1.0):
        print('Singularity detected. Skipping!')
        return
    
    # Publish new pose as JointTrajectory 
    traj = JointTrajectory()
    point = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = pose.header.frame_id
    traj.joint_names = resp.solution.joint_state.name
    point.positions = resp.solution.joint_state.position
    point.time_from_start = rospy.Duration(speed)
    traj.points.append(point)
    pub.publish(traj)
    
    # Update previous state of robot to current state
    prev_state = resp.solution.joint_state.position
    
    
    
    
def callback(data):
    global prev_state
    
    execution_time = 0.15
    new_pose = PoseStamped()
    new_pose.header.stamp = rospy.Time.now()
    new_pose.header.frame_id = 'ee_link'
    
    new_pose.pose.position.x = data.x / 100.
    new_pose.pose.position.y = data.y / 100.
    new_pose.pose.position.z = data.z / 100.
    new_pose.pose.orientation.w = 1.0
    
    publish_pose(new_pose, execution_time)

if __name__ == '__main__':
    init()
    rospy.Subscriber('keyboard/input', Vector3, callback)