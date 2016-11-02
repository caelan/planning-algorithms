from time import time
from cvxopt import matrix, solvers, mul
solvers.options['show_progress'] = False
from itertools import product
from DrawingWindowStandalone import *

states = ['left', 'right']
actions = ['open left', 'open right', 'listen']
observations = ['tiger left', 'tiger right']

transition_probs = {}
for action in ['open left', 'open right']:
    for initial in states:
        for terminal in states:
            transition_probs[(initial, action, terminal)] = .5
for initial in states:
    for terminal in states:
        if initial == terminal:
            transition_probs[(initial, 'listen', terminal)] = 1
        else:
            transition_probs[(initial, 'listen', terminal)] = 0

observation_probs = {}
for action in ['open left', 'open right']:
    for obs in observations:
        for state in states:
            observation_probs[(action, obs, state)] = .5
for obs in observations:
    for state in states:
        if state in obs:
            observation_probs[('listen', obs, state)] = .85
        else:
            observation_probs[('listen', obs, state)] = .15

rewards = {}
for action in ['open left', 'open right']:
    for state in states:
        if state in action:
            rewards[(state, action)] = -100
        else:
            rewards[(state, action)] = 10
for state in states:
    rewards[(state, 'listen')] = -1

discount_factor = 1

def intermediate_reward(initial, action):
    reward = 0
    for terminal in states:
        for obs in observations:
            reward += transition_probs[(initial, action, terminal)]*observation_probs[(action, obs, terminal)]*rewards[(initial, action)]
    return reward

def inital_vectors():
    vectors = []
    for action in actions:
        vector = []
        for state in states:
            vector.append(intermediate_reward(state, action))
        vector.append(action)
        vectors.append(vector)
    print len(vectors), 'generated vectors'
    return vectors

def generate_vectors(current_vectors):
    new_vectors = []
    for combination in product(current_vectors, repeat=len(observations)):
        for action in actions:
            vector = []
            for state in states:
                value = intermediate_reward(state, action)
                for i in range(len(observations)):
                    obs = observations[i]
                    alpha = combination[i]
                    for terminal in states:
                        value += discount_factor*transition_probs[(state, action, terminal)]*observation_probs[(action, obs, terminal)]*alpha[states.index(terminal)]
                vector.append(value)
            vector.append(action)
            new_vectors.append(vector)
    print len(new_vectors), 'generated vectors'
    return new_vectors

def dominated(vector, others):
    n = len(states)
    c = matrix([0.0]*n + [-1.0])
    constraints = []
    b = []
    for other in others:
        constraint = []
        for i in range(n):
            constraint.append(other[i] - vector[i])
        constraint.append(1.0)
        constraints.append(constraint)
        b.append(0.0)
    constraints.append([1.0]*n + [0.0])
    b.append(1.0)
    constraints.append([-1.0]*n + [0.0])
    b.append(-1.0)
    for i in range(n+1):
        constraint = [0.0]*(n+1)
        constraint[i] = -1.0
        constraints.append(constraint)
        b.append(0.0)

    G = matrix(constraints).trans()
    h = matrix(b)
    sol=solvers.lp(c,G,h)['x']
    
    if sol is None or sol[-1] < .000001:
        return True
    else:
        return False

def prune_vectors(current_vectors):
    initial_num = len(current_vectors)
    i = 0
    while i < len(current_vectors):
        if dominated(current_vectors[i], current_vectors[:i] + current_vectors[i+2:]):
            current_vectors.pop(i)
        else:
            i+=1
    print len(current_vectors), 'pruned vectors.', 1 - float(len(current_vectors))/initial_num, 'percent pruned'
    return current_vectors

def get_nth_alpha_vectors(n, prune = False):
    print 'Iteration', 0
    vectors = prune_vectors(inital_vectors()) if prune else inital_vectors()
    for i in range(1, n):
        print 'Iteration', i
        vectors = prune_vectors(generate_vectors(vectors)) if prune else generate_vectors(vectors)
    return vectors

window = DrawingWindow(600, 600, 0, 1, -50, 51, 'POMDPS')
for x in range(-50, 51, 10):
    window.drawLineSeg(0, x, 1, x, color = 'black')
window.drawLineSeg(0, 0, 1, 0, color = 'purple')

color_action = {'open left': 'red', 'listen': 'green', 'open right': 'blue'}

def draw_vectors(vectors):
    for vector in vectors:
        window.drawLineSeg(0, vector[1], 1, vector[0], color = color_action[vector[-1]])

t1 = time()
get_nth_alpha_vectors(4)
#draw_vectors(get_nth_alpha_vectors(4))
print time() - t1, 'seconds'

raw_input()

