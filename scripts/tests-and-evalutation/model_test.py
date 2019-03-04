import numpy as np
from math import sqrt, exp

goal1 = (3,8)
goal2 = (9,2)
goal3 = (5,6)
goal4 = (4,3)
positions_list = [(0,0),(1,2),(1,4),(3,5),(4,4),(6,3),(7,5),(7,7)]

def vec_minus(v1, v2):
    return (v1[0] - v2[0], v1[1] - v2[1])

def vec_mag(vec):
    return sqrt(vec[0] ** 2 + vec[1] ** 2)

def cost(vec_list):
    c = 0.
    for i, vec in enumerate(vec_list):
        if i == 0:
            continue
        c += vec_mag(vec_minus(vec, vec_list[i - 1])) ** 2
    return c

def get_list_to_goal(goal, lis):
    return [vec_minus(goal, lis[x]) for x in range(len(lis))]
        
def prob(stu, utg, stg):
    return (exp(-stu -utg) / exp(-stg)) * 0.5

def d_s(lis):
    global goal1
    global goal2
    
    cost_start_to_state = cost(lis[1:])
    
    list_optimal_state_to_goal_1 = get_list_to_goal(goal1, lis[1:])
    cost_optimal_state_to_goal_1 = cost(list_optimal_state_to_goal_1)
    
    list_optimal_start_to_goal_1 = get_list_to_goal(goal1, lis[:-1])
    cost_optimal_start_to_goal_1 = cost(list_optimal_start_to_goal_1)
    
    P_G1 = prob(cost_start_to_state, 
                cost_optimal_state_to_goal_1, cost_optimal_start_to_goal_1)
    
    list_optimal_state_to_goal_2 = get_list_to_goal(goal2, lis[1:])
    cost_optimal_state_to_goal_2 = cost(list_optimal_state_to_goal_2)
    
    list_optimal_start_to_goal_2 = get_list_to_goal(goal2, lis[:-1])
    cost_optimal_start_to_goal_2 = cost(list_optimal_start_to_goal_2)
    
    P_G2 = prob(cost_start_to_state,
                cost_optimal_state_to_goal_2, cost_optimal_start_to_goal_2)
    
    return (P_G1, P_G2)

def dot(vec1, vec2, normalised=False):
    if normalised:
        vec1_total = sqrt(vec1[0] ** 2 + vec1[1] ** 2)
        vec2_total = sqrt(vec2[0] ** 2 + vec2[1] ** 2)
        v1 = (vec1[0] / vec1_total, vec1[1] / vec1_total)
        v2 = (vec2[0] / vec2_total, vec2[1] / vec2_total)
        return v1[0] * v2[0] + v1[1] * v2[1]
    return vec1[0] * vec2[0] + vec1[1] * vec2[1]

def naive(lis):
    global goal1
    global goal2
    
    pos = lis[len(lis) - 1]
    
    guess_list = [(0,0.)] * 5
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        
        dot_1 = max(0, dot(move_traj, goal_1_traj))
        dot_2 = max(0, dot(move_traj, goal_2_traj))
        total = dot_1 + dot_2
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        
        guess = 0
        
        if prob_1 < prob_2:
            guess = 1
            
        guess_list[j - 1] = (guess, max(prob_1, prob_2) - min(prob_1, prob_2))
    
    guesses = [0.] * 2
    for g, conf in guess_list:
        guesses[g] += conf
    total = guesses[0] + guesses[1]
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses


def norm(lis):
    global goal1
    global goal2
    
    pos = lis[len(lis) -1]
    guess_list = [(0,0.)] * 5
    weights = [0.44, 0.25, 0.18, 0.09, 0.05]
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        
        dot_1 = max(0, dot(move_traj, goal_1_traj, True))
        dot_2 = max(0, dot(move_traj, goal_2_traj, True))
        total = dot_1 + dot_2
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        
        guess = 0
        
        if prob_1 < prob_2:
            guess = 1
            
        guess_list[j - 1] = (guess, max(prob_1, prob_2) - min(prob_1, prob_2))
    
    # print(guess_list)
    guesses = [0.] * 2
    for i in range(len(weights)):
        g = guess_list[i][0]
        conf = guess_list[i][1]
        weight = weights[i]
        guesses[g] += conf * weight
    total = sum(guesses)
    
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses

def quad(lis):
    global goal1
    global goal2
    
    pos = lis[len(lis) -1]
    guess_list = [(0,0.)] * 5
    weights = [0.44, 0.25, 0.18, 0.09, 0.05]
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        
        dot_1 = max(0, dot(move_traj, goal_1_traj, True)) ** 2
        dot_2 = max(0, dot(move_traj, goal_2_traj, True)) ** 2
        total = dot_1 + dot_2
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        
        guess = 0
        
        if prob_1 < prob_2:
            guess = 1
            
        guess_list[j - 1] = (guess, max(prob_1, prob_2) - min(prob_1, prob_2))
    
    # print(guess_list)
    guesses = [0.] * 2
    for i in range(len(weights)):
        g = guess_list[i][0]
        conf = guess_list[i][1]
        weight = weights[i]
        guesses[g] += conf * weight
    total = sum(guesses)
    
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses


def naive_prob(lis):
    global goal1
    global goal2
    global goal3
    global goal4
    
    pos = lis[len(lis) -1]
    probs = [(0,0.)] * 5
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        goal_3_traj = vec_minus(goal3, start)
        goal_4_traj = vec_minus(goal4, start)        
        
        dot_1 = max(0, dot(move_traj, goal_1_traj))
        dot_2 = max(0, dot(move_traj, goal_2_traj))
        dot_3 = max(0, dot(move_traj, goal_3_traj))
        dot_4 = max(0, dot(move_traj, goal_4_traj))
        total = dot_1 + dot_2 + dot_3 + dot_4
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        prob_3 = float(dot_3) / float(total)
        prob_4 = float(dot_4) / float(total)
            
        probs[j-1] = (prob_1, prob_2, prob_3, prob_4)
    
    # print(guess_list)
    guesses = [0.] * 4
    for p1, p2, p3, p4 in probs:
        guesses[0] += p1
        guesses[1] += p2
        guesses[2] += p3
        guesses[3] += p4
    total = sum(guesses)
    
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses


def norm_prob(lis):
    global goal1
    global goal2
    global goal3
    global goal4
    
    pos = lis[len(lis) -1]
    probs = [(0.,0.)] * 5
    weights = [0.44, 0.25, 0.18, 0.09, 0.05]
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        goal_3_traj = vec_minus(goal3, start)
        goal_4_traj = vec_minus(goal4, start)
        
        dot_1 = max(0, dot(move_traj, goal_1_traj, True))
        dot_2 = max(0, dot(move_traj, goal_2_traj, True))
        dot_3 = max(0, dot(move_traj, goal_3_traj, True))
        dot_4 = max(0, dot(move_traj, goal_4_traj, True))
        total = dot_1 + dot_2 #+ dot_3 #+ dot_4
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        prob_3 = float(dot_3) / float(total)
        prob_4 = float(dot_4) / float(total)
        
        probs[j-1] = (prob_1, prob_2)#, prob_3)#, prob_4)
    
    # print(guess_list)
    guesses = [0.] * 4
    for i in range(len(weights)):
        guesses[0] += probs[i][0] * weights[i]
        guesses[1] += probs[i][1] * weights[i]
        #guesses[2] += probs[i][2] * weights[i]
        #guesses[3] += probs[i][3] * weights[i]
    total = sum(guesses)
    
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses

def quad_prob(lis):
    global goal1
    global goal2
    global goal3
    global goal4
    
    pos = lis[len(lis) -1]
    probs = [(0.,0.)] * 5
    weights = [0.44, 0.25, 0.18, 0.09, 0.05]
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        goal_3_traj = vec_minus(goal3, start)
        goal_4_traj = vec_minus(goal4, start)
        
        dot_1 = max(0, dot(move_traj, goal_1_traj, True)) ** 2
        dot_2 = max(0, dot(move_traj, goal_2_traj, True)) ** 2
        dot_3 = max(0, dot(move_traj, goal_3_traj, True)) ** 2
        dot_4 = max(0, dot(move_traj, goal_4_traj, True)) ** 2
        total = dot_1 + dot_2 #+ dot_3 + dot_4
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        prob_3 = float(dot_3) / float(total)
        prob_4 = float(dot_4) / float(total)
        
        probs[j-1] = (prob_1, prob_2)#, prob_3)#, prob_4)
    
    # print(guess_list)
    guesses = [0.] * 4
    for i in range(len(weights)):
        guesses[0] += probs[i][0] * weights[i]
        guesses[1] += probs[i][1] * weights[i]
        #guesses[2] += probs[i][2] * weights[i]
        #guesses[3] += probs[i][3] * weights[i]
    total = sum(guesses)
    
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses

def exp_prob(lis):
    global goal1
    global goal2
    global goal3
    global goal4
    
    pos = lis[len(lis) -1]
    probs = [(0.,0.)] * 5
    weights = [0.44, 0.25, 0.18, 0.09, 0.05]
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        goal_3_traj = vec_minus(goal3, start)
        goal_4_traj = vec_minus(goal4, start)
        
        dot_1 = exp(max(0, dot(move_traj, goal_1_traj, True)))
        dot_2 = exp(max(0, dot(move_traj, goal_2_traj, True)))
        dot_3 = exp(max(0, dot(move_traj, goal_3_traj, True)))
        dot_4 = exp(max(0, dot(move_traj, goal_4_traj, True)))
        total = dot_1 + dot_2 + dot_3 + dot_4
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        prob_3 = float(dot_3) / float(total)
        prob_4 = float(dot_4) / float(total)
        
        probs[j-1] = (prob_1, prob_2, prob_3, prob_4)
    
    # print(guess_list)
    guesses = [0.] * 4
    for i in range(len(weights)):
        guesses[0] += probs[i][0] * weights[i]
        guesses[1] += probs[i][1] * weights[i]
        guesses[2] += probs[i][2] * weights[i]
        guesses[3] += probs[i][3] * weights[i]
    total = sum(guesses)
    
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses


def nexp(lis):
    global goal1
    global goal2
    
    pos = lis[len(lis) -1]
    guess_list = [(0,0.)] * 5
    weights = [0.44, 0.25, 0.18, 0.09, 0.05]
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        
        dot_1 = exp(max(0, dot(move_traj, goal_1_traj, True)))
        dot_2 = exp(max(0, dot(move_traj, goal_2_traj, True)))
        total = dot_1 + dot_2
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        
        guess = 0
        
        if prob_1 < prob_2:
            guess = 1
            
        guess_list[j - 1] = (guess, max(prob_1, prob_2) - min(prob_1, prob_2))
    
    # print(guess_list)
    guesses = [0.] * 2
    for i in range(len(weights)):
        g = guess_list[i][0]
        conf = guess_list[i][1]
        weight = weights[i]
        guesses[g] += conf * weight
    total = sum(guesses)
    
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses


def cube_prob(lis):
    global goal1
    global goal2
    global goal3
    global goal4
    
    pos = lis[len(lis) -1]
    probs = [(0.,0.)] * 5
    weights = [0.44, 0.25, 0.18, 0.09, 0.05]
    
    for j in range(1,6):
        start_index = max(0, len(lis) - (j + 1))
        start = lis[start_index]
        move_traj = vec_minus(pos, start)
        goal_1_traj = vec_minus(goal1, start)
        goal_2_traj = vec_minus(goal2, start)
        goal_3_traj = vec_minus(goal3, start)
        goal_4_traj = vec_minus(goal4, start)
        
        dot_1 = max(0, dot(move_traj, goal_1_traj, True) ** 3)
        dot_2 = max(0, dot(move_traj, goal_2_traj, True) ** 3)
        dot_3 = max(0, dot(move_traj, goal_3_traj, True) ** 3)
        dot_4 = max(0, dot(move_traj, goal_4_traj, True) ** 3)
        total = dot_1 + dot_2 + dot_3 + dot_4
        
        prob_1 = float(dot_1) / float(total)
        prob_2 = float(dot_2) / float(total)
        prob_3 = float(dot_3) / float(total)
        prob_4 = float(dot_4) / float(total)
        
        probs[j-1] = (prob_1, prob_2, prob_3, prob_4)
    
    # print(guess_list)
    guesses = [0.] * 4
    for i in range(len(weights)):
        guesses[0] += probs[i][0] * weights[i]
        guesses[1] += probs[i][1] * weights[i]
        guesses[2] += probs[i][2] * weights[i]
        guesses[3] += probs[i][3] * weights[i]
    total = sum(guesses)
    
    for i in range(len(guesses)):
        guesses[i] /= total
        
    return guesses
            
    

for i, position in enumerate(positions_list):
    if i == 0:
        continue
    probabilities = d_s(positions_list[:i+1])
    # print(probabilities)
    probabilities_naive = naive(positions_list[:i+1])
    probabilities_norm = norm(positions_list[:i+1])
    probabilities_quad = quad(positions_list[:i+1])
    probabilities_naive_prob = naive_prob(positions_list[:i+1])
    probabilities_norm_prob = norm_prob(positions_list[:i+1])
    probabilities_quad_prob = quad_prob(positions_list[:i+1])
    probabilities_exp = nexp(positions_list[:i+1])
    probabilities_exp_prob = exp_prob(positions_list[:i+1])
    probabilities_cube_prob = cube_prob(positions_list[:i+1])
    print(probabilities_norm_prob)
    
    
