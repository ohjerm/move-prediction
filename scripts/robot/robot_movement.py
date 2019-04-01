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
import copy
from math import sqrt
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Point
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from move_prediction.msg import PointArr, Goal
from visualization_msgs.msg import Marker


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
start_pub = None
curr_pub = None
marker_pub = None
tf_listener = None
predicted_goal = None
confidence = 0.
time_since_prediction = time.time()

positions_list = [Point()] * 520



def init():
    global pub
    global robot
    global scene
    global group
    global ik_srv
    global req
    global prev_state
    global start_pub
    global positions_list
    global curr_pub
    global marker_pub
    global tf_listener
    # Setup moveit stuff
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_control_node')
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = 'manipulator'
    group = moveit_commander.MoveGroupCommander(group_name)
    
    prev_state = robot.get_current_state().joint_state.position

    tf_listener = tf.TransformListener()
    
    # real start position at start time
    pos = group.get_current_pose().pose.position
    fixed_pos = Point()
    fixed_pos.z = -pos.x
    fixed_pos.y = pos.z
    fixed_pos.x = pos.y
    positions_list = [fixed_pos] * 520
    
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
    pub = rospy.Publisher('/vel_based_pos_traj_controller/command', JointTrajectory, queue_size=1)
    start_pub = rospy.Publisher("keyboard/robot_start_position", PointArr, queue_size=1)
    curr_pub = rospy.Publisher('/robot/current_position', Point, queue_size=1)
    marker_pub = rospy.Publisher('/robot/marker_pos', Marker, queue_size=1)
    
    # Prepare IK req
    req = GetPositionIKRequest()
    req.ik_request.timeout = rospy.Duration(1.)
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
    
    

def add_points(old, new):
    p = Point()
    p.x = old.x + new.x
    p.y = old.y + new.y
    p.z = old.z + new.z
    return p
    
    

def publish_current_pose():
    global start_pub
    global curr_pub
    pos = group.get_current_pose().pose.position
    
    fixed_pos = Point()
    fixed_pos.z = -pos.x
    fixed_pos.y = pos.z
    fixed_pos.x = pos.y
    
    positions_list.insert(0, fixed_pos)
    
    to_pub = PointArr()
    to_copy = [positions_list[55], positions_list[97], 
                    positions_list[135], positions_list[267],  
                    positions_list.pop()]
    to_pub.Array = copy.deepcopy(to_copy)
    start_pub.publish(to_pub)

    curr_pub.publish(fixed_pos)

    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = 'camera_link'
    marker.pose = group.get_current_pose().pose
    # marker.pose.position = fixed_pos

    scale = Vector3()
    scale.x = 0.02
    scale.y = 0.02
    scale.z = 0.02

    marker.color.a = 1
    marker.color.r = 1

    marker.scale = scale

    marker_pub.publish(marker)

    
    
def get_magnitude(vector):
    if type(vector) is Vector3 or type(vector) is Point:
        return sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)
    elif type(vector) is list:
        return sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)


def get_normalized_vector(vector):
    if type(vector) is Vector3 or type(vector) is Point:
        if get_magnitude(vector) == 0. or get_magnitude(vector) == 1.:
            return vector
        else:
            to_ret = Vector3()
            mag = get_magnitude(vector)
            to_ret.x = vector.x / mag
            to_ret.y = vector.y / mag
            to_ret.z = vector.z / mag
            return to_ret
    elif type(vector) is list:
        if get_magnitude(vector) == 0:
            return vector
        else:
            to_ret = Vector3()
            mag = get_magnitude(vector)
            to_ret.x = vector[0] / mag
            to_ret.y = vector[1] / mag
            to_ret.z = vector[2] / mag
            return to_ret
    
    
def get_vector(target, current):
    to_ret = Vector3()
    to_ret.x = target.x - current.x
    to_ret.y = target.y - current.y
    to_ret.z = target.z - current.z
    return to_ret


def convert_to_ur_coord(vector):
    new_coords = Point()
    
    new_coords.x = -vector.z
    new_coords.y = vector.x
    new_coords.z = -vector.y
    
    return new_coords
    
    
def callback(data):
    global prev_state
    global predicted_goal
    global time_since_prediction
    global confidence
    global tf_listener
    
    publish_current_pose()  # we always want to publish this and update it
    
    # first check if it has been a second since the last update
    if predicted_goal is not None and time.time() - time_since_prediction > 1.:
        predicted_goal = None
        confidence = 0.
        
    execution_time = 0.5
    new_pose = PoseStamped()
    new_pose.header.stamp = rospy.Time.now()
    new_pose.header.frame_id = 'ee_link'

    
    
    if abs(data.x) < 0.05 and abs(data.y) < 0.05 and abs(data.z) < 0.05:
        return
    
    if predicted_goal is not None:
        """
        # rospy.loginfo(predicted_goal)
        # rospy.loginfo(group.get_current_pose().pose.position)
        optimal_goal_direction = get_vector(convert_to_ur_coord(predicted_goal),
                                            group.get_current_pose().pose.position)
        optimal_goal_direction = get_normalized_vector(optimal_goal_direction)
        keyboard_magnitude = get_magnitude(data)
        optimal_goal_direction.x *= keyboard_magnitude
        optimal_goal_direction.y *= keyboard_magnitude
        optimal_goal_direction.z *= keyboard_magnitude

        rospy.loginfo("predicted")
        rospy.loginfo(convert_to_ur_coord(predicted_goal))
        rospy.loginfo("current pos")
        rospy.loginfo(group.get_current_pose())

        # rospy.loginfo("The goal is in x: " + str(predicted_goal.x))
        # rospy.loginfo("Current pos is x: " + str(group.get_current_pose().pose.position.y))
        # rospy.loginfo("Current fixed  x: " + str(convert_to_ur_coord(predicted_goal).x))
        
        new_pose.pose.position.x = data.x / 100. * (1 - confidence) + optimal_goal_direction.x / 100. * confidence
        new_pose.pose.position.y = data.y / 100. * (1 - confidence) + optimal_goal_direction.y / 100. * confidence
        new_pose.pose.position.z = data.z / 100. # * (1 - confidence) + optimal_goal_direction.z / 100. * confidence

        new_pose.pose.orientation.w = 1.0
        """

        # predicted goal pos in ee_link space
        try:
            # now = rospy.Time.now()
            seconds = rospy.get_time()
            seconds -= 0.1
            now = rospy.Time.from_sec(seconds)
            (trans, rot) = tf_listener.lookupTransform("/camera_depth_optical_frame", "/ee_link", now)
            
            #position is
            point = PointStamped()
            point.header.frame_id = '/camera_depth_optical_frame'
            point.header.stamp = now
            point.point = predicted_goal
            target_pos = tf_listener.transformPoint('/ee_link', point).point
            optimal_goal_direction = get_normalized_vector(target_pos)

            keyboard_magnitude = get_magnitude(data)
            optimal_goal_direction.x *= keyboard_magnitude
            optimal_goal_direction.y *= keyboard_magnitude
            optimal_goal_direction.z *= keyboard_magnitude

            new_pose.pose.position.x = data.x / 100. * (1 - confidence) + optimal_goal_direction.x / 100. * confidence
            new_pose.pose.position.y = data.y / 100. * (1 - confidence) + optimal_goal_direction.y / 100. * confidence
            new_pose.pose.position.z = data.z / 100. # * (1 - confidence) + optimal_goal_direction.z / 100. * confidence

            new_pose.pose.orientation.w = 1
            publish_pose(new_pose, execution_time)
        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("Exception on lookupTransform")
    else:
        new_pose.pose.position.x = data.x / 100.
        new_pose.pose.position.y = data.y / 100.
        new_pose.pose.position.z = data.z / 100.
        new_pose.pose.orientation.w = 1.0
        
        publish_pose(new_pose, execution_time)
    
    
def cb_goal(data):
    global predicted_goal
    global confidence
    global time_since_prediction
    predicted_goal = data.point
    predicted_goal.z -= 0.1
    confidence = data.confidence
    time_since_prediction = time.time()
    
    

if __name__ == '__main__':
    init()
    rospy.Subscriber('keyboard/input', Vector3, callback)
    rospy.Subscriber('arbitration/final_goal', Goal, cb_goal)
    rospy.spin()
