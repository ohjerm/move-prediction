#!/usr/bin/env python

"""
this code will guess a most likely goal from a set of possible goals
goals are gathered from "camera/depth/color/cluster_positions"
trajectories are gathered from "keyboard/trajectories"
"""

#1,0.46,0.22,0.11,0.08
import rospy
import copy
import numpy as np
from geometry_msgs.msg import Point, Vector3
from move_prediction.msg import VectorArr, PointArr
from math import sqrt

goal_list = []
robot_positions_list = []


def cb_robot_positions(data):
    global robot_positions_list
    robot_positions_list = data.Array


def get_distance(point1, point2):
    return sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)


def cb_goal_positions(data):
    """
    edits the list of goals based on new information
    :data: a list of Points
    :returns: nothing
    """
    global goal_list
    
    for goal in data.Array:
        add_to_list = True  # to determine whether to add this goal as new
        for existing_goal in goal_list:  
            # check every point against existing goals
            if get_distance(goal, existing_goal[0]) < 0.02:  # it exists
                existing_goal[1] = 0
                add_to_list = False
                break
        
        if add_to_list:
            goal_list.append([goal,0])
            
    # check "age" of all existing goals
    for existing_goal in goal_list:
        if existing_goal[1] > 5:  # remove if 5 frames (~1 sec) old
            goal_list.remove(existing_goal)
        else:
            existing_goal[1] += 1  # it has existed for one more frame

    
    
def calc_fit(best, taken):
    """
    calculates the fit (dot product) of two normalized vectors
    :best: a vector
    :taken: a vector
    :returns: a float
    """
    return best.x * taken.x + best.y * taken.y + best.z * taken.z
    
    
def normalize(vector):
    v = Vector3()
    mag = sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)
    if mag == 0:
        return v
    
    v.x = vector.x/mag
    v.y = vector.y/mag
    v.z = vector.z/mag
    return v
    
    

def calc_trajectory_normalized(point_start, point_end):
    v = Vector3()
    v.x = point_end.x - point_start.x
    v.y = point_end.y - point_start.y
    v.z = point_end.z - point_start.z
    
    mag = sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
    if mag == 0:
        return v
    
    v.x /= mag
    v.y /= mag
    v.z /= mag
    
    return v


def no_length(vector):
    return vector.x == 0 and vector.y == 0 and vector.z == 0


def cb_trajectories(data):
    global goal_list
    global robot_positions_list
    
    #kind of slow (deepcopying) but likely necessary
    copy_goal_list = copy.deepcopy(goal_list)  # ensure no race conditions
    copy_posi_list = copy.deepcopy(robot_positions_list)
    
    best_guess = np.zeros((len(data.Array), 2)) # to contain guess and confidence
    
    for i, trajectory in enumerate(data.Array):
        # check if we have everything we need
        if len(copy_goal_list) == 0 or len(copy_posi_list) == 0:
            return
        #initialize array to store probabilities
        probabilities = np.zeros(len(copy_goal_list))
        #iterate through each goal
        for j, (goal, frames) in enumerate(copy_goal_list):
            optimal_path = calc_trajectory_normalized(copy_posi_list[i], goal)
            taken_path = normalize(trajectory)
            if no_length(optimal_path) or no_length(taken_path):
                continue  # there is now 0 probabilities for this goal
            probabilities[j] = max(0, calc_fit(optimal_path, taken_path))
        
        # now we have the fits for each of these, we sum them 
        # and create our probability distribution
        total = sum(probabilities)
        if total == 0:
            continue
        probabilities /= total
        
        #the guess is the highest probability, the confidence is prob_1-prob_2
        #np.partition ensures -2 is the pivot in at most O(n) time
        guess = probabilities.argmax()
        conf = 0.
        if len(probabilities) > 1:
            conf = probabilities[guess] - np.partition(probabilities, -2)[-2]
        else:
            conf = 1.
        
        # the best guess and its confidence is added to the best_guess array
        best_guess[i] = np.array([guess, conf])
     
    """
    now we have a list of best guesses and their confidence
    we modify the confidence based on a weight for each trajectory
    these are calculated as traj_time[0]/traj_time[i] normalized over the sum
    1, 0.46, 0.22, 0.11, 0.08 = 0.53, 0.25, 0.12, 0.06, 0.04
    """
    weights = np.array([0.53, 0.25, 0.12, 0.06, 0.04])
    total = 0.
    for i, confidence in enumerate(best_guess[:,1]):
        weighted_confidence = weights[i] * confidence
        total += weighted_confidence
        best_guess[i,1] = weighted_confidence
        
    if total == 0:
        return  # we should publish 0 confidence here
    best_guess[:,1] /= total
    
    #new we have weighted confidences, time to do a weighted majority vote
    votes = np.zeros(len(copy_goal_list))
    for guess, weighted_confidence in best_guess:
        votes[int(guess)] += weighted_confidence
        
    final_goal = copy_goal_list[votes.argmax()][0]
    confidence_of_goal = max(votes) - np.partition(votes, -2)[-2]
    goal_position_and_confidence = (final_goal, confidence_of_goal)
    
    rospy.loginfo("I believe it to be the goal located at " + 
                  str(final_goal.x) + ", " + str(final_goal.y)
                  + ", " + str(final_goal.z) + 
                  " with a confidence of " + str(confidence_of_goal))


def init():
    rospy.init_node('intent_prediction_node', anonymous=False)
    rospy.Subscriber('keyboard/robot_start_position', 
                     PointArr, cb_robot_positions)
    rospy.Subscriber('camera/depth/color/cluster_positions', 
                     PointArr, cb_goal_positions)
    rospy.Subscriber('keyboard/trajectories', VectorArr, cb_trajectories)
    rospy.spin()


if __name__ == '__main__':
    init()