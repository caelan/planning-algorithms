"""A simple state-space planner for states represented as sets of
ground assertions."""

from random import shuffle
from time import time
from heapq import heappush, heappop

class State:
    def __init__(self, assertions):
        self.assertions = frozenset(assertions)
    def check(self, assertions):
        return all([a in self.assertions for a in assertions])
    def addDelete(self, add, delete, noDel = False):
        return State(add + [a for a in self.assertions if noDel or (not a in delete)])
    def __str__(self):
        return str(self.assertions)
    def __eq__(self, other):
        return self.assertions == other.assertions
    def __hash__(self):
        return self.assertions.__hash__()
    __repr__ = __str__

class Action:
    def __init__(self, args):
        self.args = args
    def __str__(self):
        return str(self.actionName) + str(tuple(self.args))
    __repr__ = __str__
    
## Used to create all the arguments for all the action instances.

def combinations(listOfLists):
    def merge(fn, values):
        ans = []
        for x in values: ans.extend(fn(x))
        return ans

    if not listOfLists:
        return [[]]
    else:
        return merge(lambda old: [[elt] + old for elt in listOfLists[0]],
                     combinations(listOfLists[1:]))

## This is generic interface to search

class PlanProblem:
    def __init__(self, initial, goal, acts):
        self.initialState = initial
        self.goalAssertions = goal.assertions
        self.actionInstances = acts
        self.ffTable = {}
        self.helpfulTable = {}
    def goalTest(self, state, goalConditions = []):
        # returns True when all the the assertions in self.goal are in state
        # if goalConditions is specified, use that instead of self.goal
        if goalConditions:
            return state.check(goalConditions)
        else:
            return state.check(self.goalAssertions)
    def stateActions(self, state):
        # returns list of action instances relevant to state, that is,
        # which when applied to state return a valid state.  Since our
        # convention is that actions always return a valid state,
        # possibly the same as the current state, we can simply use
        # all the actions.
        return self.actionInstances
        #return self.helpfulActions(state)
    def helpfulActions(self, state):
        if start in self.ffTable:
            return self.ffTable[start]

        level = 0
        fluents = {}
        for fluent in start.assertions:
            fluents[fluent] = level
        state = start
        while True:
            level += 1
            if self.goalTest(state):
              h_max = max([fluents[goal] for goal in self.goalAssertions]) #Uniform costs => FF = max
              self.helpfulTable[start] = h_max
              return h_max
            added = []
            for action in self.stateActions(state):
              pair = self.planSuccessor(state, action, noDel = True)
              if pair is None:
                continue
              (next, cost) = pair
              for fluent in next.assertions:
                if fluent not in fluents:
                  fluents[fluent] = level
                  added.append(fluent)
            if len(added) == 0:
              self.ffTable[start] = None
              return None
            state = State(added + [assertion for assertion in state.assertions])


        return map(lambda pair: pair[0], filter(lambda pair: pair[1] is not None and
            any([assertion not in state.assertions for assertion in pair[1][0].assertions]),
            [(action, self.planSuccessor(state, action, noDel=True)) for action in self.actionInstances]))    
    def planSuccessor(self, state, act, noDel = False):
        # returns (newState, cost) that is result of applying act to state
        # if the act is not applicable to the state (returns None),
        # return (state, cost) where cost is arbitrary (> 0).
        # if noDel is True, dont do the deletes
        return act.resultStateAndCost(state, noDel=noDel)
    def noHeuristic(self, state):
        return 0
    def naiveHeuristic(self, state):
        return sum([1 if a not in state.assertions else 0 for a in self.goalAssertions])
    def ffHeuristic(self, start):
        #if start in self.ffTable:
        #    return self.ffTable[start]

        level = 0
        fluents = {}
        for fluent in start.assertions:
            fluents[fluent] = level
        state = start
        actions = self.stateActions(state)
        while True:
            level += 1
            if self.goalTest(state):
              h_max = max([fluents[goal] for goal in self.goalAssertions]) #Uniform costs => FF = max
              self.ffTable[start] = h_max
              return h_max
            added = []
            for action in actions:
              pair = self.planSuccessor(state, action, noDel = True)
              if pair is None:
                continue
              del action
              (next, cost) = pair
              for fluent in next.assertions:
                if fluent not in fluents:
                  fluents[fluent] = level
                  added.append(fluent)
            if len(added) == 0:
              self.ffTable[start] = None
              return None
            state = State(added + [assertion for assertion in state.assertions])

    def ffHeuristic_meh(self, state):
        #if state in self.ffTable:
        #    print 'yay'
        #    return self.ffTable[state]



        paths = [(0, state)]
        fluents = {}
        for assertion in state.assertions:
            fluents[assertion] = 0
        states = {}
        states[state] = 0
        actions = set()
        while len(paths) != 0:
            (dist, last) = heappop(paths)
            #print len(paths), len(visited)
            #print dist, last, '\n'
            if states[last] != dist: #Becuase I'm not using a fibonacci heap, bad paths may be in the queue
              continue
            if self.goalTest(last):
              print 'done'
              self.ffTable[state] = dist
              return max([fluents[goal] for goal in self.goalAssertions]) #Uniform costs => FF = max
            for action in self.stateActions(last):
              if action in actions:
                continue
              pair = self.planSuccessor(last, action, noDel = True)
              if pair is None:
                continue
              (next, cost) = pair
              new_dist = dist + cost
              #actions.add(action)
              for fluent in next.assertions:
                if fluent not in fluents or new_dist < fluents[fluent]:
                  fluents[fluent] = new_dist
              heappush(paths, (new_dist, next)) #Substitute for decrease_key
              states[next] = new_dist

        self.ffTable[state] = None
        print 'fuck'
        #quit()
        return None
    def ffHeuristic_bad(self, state):
        #if state in self.ffTable:
        #    print 'yay'
        #    return self.ffTable[state]

        print 'ff'

        start = state
        paths = [(0, start)]
        visited_states = {}
        visited_states[start] = 0
        visited_actions = set()
        while len(paths) != 0:
            (dist, last) = heappop(paths)
            #print len(paths), len(visited)
            #print dist, last, '\n'
            if visited_states[last] != dist: #Becuase I'm not using a fibonacci heap, bad paths may be in the queue
              continue
            if self.goalTest(last):
              self.ffTable[state] = dist

              print 'done'
              #quit()
              return dist
            for action in self.stateActions(last):
              if action in visited_actions:
                continue
              pair = self.planSuccessor(last, action, noDel = True)
              if pair is None:
                continue
              (next, cost) = pair
              new_dist = dist + cost
              visited_actions.add(action)
              if next not in visited_states or new_dist < visited_states[next]:
                heappush(paths, (new_dist, next)) #Substitute for decrease_key
                visited_states[next] = new_dist

        self.ffTable[state] = None
        print 'fuck'
        #quit()
        return None
    def planHeuristic(self, state):
        # returns estimate of cost to the goal
        #return self.noHeuristic(state)
        #return self.naiveHeuristic(state)
        return self.ffHeuristic(state)
    def findPlan(self, maxNodes = 10000):
        # Call a search function
        t1 = time()
        result = search(self.initialState,
                        self.goalTest,
                        self.stateActions,
                        self.planSuccessor,
                        heuristic = self.planHeuristic,
                        maxNodes = maxNodes)
        print 'Took', time() - t1, 'seconds'
        if result:
            (plan, cost) = result
            print 'Found plan with cost', cost
            for (action, state) in plan:
                if action: print action
            return plan
        else:
            print 'Failed to find a plan'

# Best-first heuristic search

def retrace_path(path_map, state):
    success = [(None, state)]
    retrace = state
    while retrace is not None:
      success.append((path_map[retrace][2], path_map[retrace][1]))
      retrace = path_map[retrace][1]
    success.reverse()
    return success

def search(start, goalTest, stateActions, planSuccessor, heuristic, maxNodes):
    reached = {}
    reached[start] = (0, None, None)
    paths = [(heuristic(start) + 0, 0, start)]
    iterations = 0
    while len(paths) != 0 and iterations < maxNodes:
        (hDist, dist, state) = heappop(paths)
        if reached[state][0] != dist: #Becuase I'm not using a fibonacci heap, bad paths may be in the queue
          continue
        if goalTest(state):
          print iterations, 'iterations'
          return (retrace_path(reached, state), dist)
        for action in stateActions(state):
          result = planSuccessor(state, action)
          if result is None:
            continue
          (nextState, cost) = result
          newHDist = heuristic(nextState)
          if newHDist is None:
            #reached[nextState] = (None, state, action) #Store if no possible path under relaxed plan?
            continue
          newDist = dist + cost
          if (nextState not in reached or newDist < reached[nextState][0]):
            heappush(paths, (newHDist + newDist, newDist, nextState)) #Substitute for decrease_key
            reached[nextState] = (newDist, state, action)
        iterations+=1
    print iterations, 'iterations'
    return None

def search_dfs(start, goalTest, stateActions, planSuccessor, heuristic, maxNodes):
    reached = {}
    reached[start] = maxNodes

    print heuristic(start)
    #quit()

    def recur(state, maxDepth):
        if goalTest(state):
            return ([(None, state)], 0)
        elif maxDepth == 0:
            #print 'backtrack'
            return None
        possibleActions = filter(lambda h_tup: h_tup[0] is not None, 
            [(heuristic(tup[1][0]), tup[0], tup[1])  for tup in filter(lambda pair: pair[1] is not None,
            [(action, planSuccessor(state, action)) for action in stateActions(state)])])
        shuffle(possibleActions)
        possibleActions.sort()
        for stuff in possibleActions:
            print stuff, '\n'
        raw_input()
        for (h, action, (newState, cost)) in possibleActions:
            if newState not in reached or reached[newState] < maxDepth:
                reached[newState] = maxDepth
                success = recur(newState, maxDepth - 1)
                if success:
                    return ([(action, state)] + success[0], cost + success[1])
        return None

    return recur(start, maxNodes)

# Given an initial state and a sequence of action instances, it
# simulates the effects of the actions starting in the initial state.
# This is useful for debugging action definitions.

def simulate(initial, actions):
    state = initial
    print 'State', state
    for act in actions:
        print act
        result = act.resultStateAndCost(state)
        if result:
            state = result[0]
            print 'State', state
        else:
            print 'Action does not apply'
            break

## Blocks World example

class PickUp(Action):
    actionName = 'PickUp'
    def resultStateAndCost(self, state, noDel = False):
        (obj,) = self.args
        if state.check([('clear', obj), ('onTable',  obj), ('armEmpty')]):
            return (state.addDelete([('holding', obj)],
                                    # Deletes
                                    [('clear', obj),
                                     ('onTable', obj),
                                     ('armEmpty')],
                                    noDel),
                    1)

class PutDown(Action):
    actionName = 'PutDown'
    def resultStateAndCost(self, state, noDel = False):
        (obj,) = self.args
        if state.check([('holding', obj)]):
            return (state.addDelete([('clear', obj),
                                     ('onTable', obj),
                                     ('armEmpty')],
                                    # Deletes
                                    [('holding', obj)],
                                    noDel),
                    1)

class Stack(Action):
    actionName = 'Stack'
    def resultStateAndCost(self, state, noDel = False):
        (obj, underObj, ) = self.args
        if state.check([('holding', obj), ('clear', underObj)]):
            return (state.addDelete([('clear', obj),
                                     ('armEmpty'),
                                     ('on', obj, underObj)],
                                    # Deletes
                                    [('holding', obj),
                                     ('clear', underObj)],
                                    noDel),
                    1)

class Unstack(Action):
    actionName = 'Unstack'
    def resultStateAndCost(self, state, noDel = False):
        (obj, underObj, ) = self.args
        if state.check([('clear', obj), ('armEmpty'), ('on', obj, underObj)]):
            return (state.addDelete([('holding', obj),
                                     ('clear', underObj)],
                                    # Deletes
                                    [('clear', obj),
                                     ('armEmpty'),
                                     ('on', obj, underObj)],
                                    noDel),
                    1)

## Painting
#Do I have to remove past painted colors?
class SprayPaint(Action):
    actionName = 'SprayPaint'
    def resultStateAndCost(self, state, noDel = False):
        (obj, sprayer, color) = self.args
        if state.check([('onTable', obj), ('clear', obj), ('paintsColor', sprayer, color), ('holding', sprayer)]):
            return (state.addDelete([('colored', obj, color)],
                                    # Deletes
                                    [],
                                    noDel),
                    1)

class BrushPaint(Action):
    actionName = 'BrushPaint'
    def resultStateAndCost(self, state, noDel = False):
        (obj, brush, color) = self.args
        if state.check([('onTable', obj), ('clear', obj), ('paintsColor', brush, color), ('holding', brush)]): #and state.checkNegative([('clean', brush)]):
            return (state.addDelete([('colored', obj, color)],
                                    # Deletes
                                    [],
                                    noDel),
                    1)

class LoadBrush(Action):
    actionName = 'LoadBrush'
    def resultStateAndCost(self, state, noDel = False):
        (brush, paintCan, color) = self.args
        if state.check([('clean', brush), ('clear', paintCan), ('paintsColor', paintCan, color), ('holding', brush)]): # does the paintCan need to be on the table?
            return (state.addDelete([('paintsColor', brush, color)],
                                    # Deletes
                                    [('clean', brush)],
                                    noDel),
                    1)

class WashBrush(Action):
    actionName = 'WashBrush'
    def resultStateAndCost(self, state, noDel = False):
        (brush, waterBucket, color) = self.args
        if state.check([('clear', waterBucket), ('holding', brush), ('paintsColor', brush, color)]): # does the bucket need to be on the table?
            return (state.addDelete([('clean', brush)],
                                    # Deletes
                                    [('paintsColor', brush, color)],
                                    noDel),
                    1)

def blocks_stack():
    print 'Blocks Stack'
    BLOCKS = ['blockA', 'blockB', 'blockC']
    OBJECTS = BLOCKS

    BLOCKACTIONS = [PickUp(args) for args in combinations([OBJECTS])] +\
            [PutDown(args) for args in combinations([OBJECTS])] +\
            [Stack(args) for args in combinations([OBJECTS, OBJECTS])] +\
            [Unstack(args) for args in combinations([OBJECTS, OBJECTS])]
    ACTIONS = BLOCKACTIONS

    INITIAL = State([('on', 'blockA', 'blockB'), 
                     ('on', 'blockB', 'blockC'), 
                     ('onTable', 'blockC'),
                     ('clear', 'blockA'), 
                     ('armEmpty')])
    GOAL = State([('on', 'blockC', 'blockB'),
                     ('on', 'blockB', 'blockA'),
                     ('onTable', 'blockA'),
                     ('clear', 'blockC'), 
                     ('armEmpty')])


    blocksProblem = PlanProblem(INITIAL, GOAL, ACTIONS)
    blocksProblem.findPlan(maxNodes = 100000)


def blocks_paint_1():
    print 'Paint Blocks 1'
    BLOCKS = ['blockA']
    SPRAYERS = []
    BRUSHES = ['brush']
    PAINTCANS = ['redPaintCan']
    WATERBUCKETS = []
    COLORS = ['red']
    OBJECTS = BLOCKS + SPRAYERS + BRUSHES + PAINTCANS + WATERBUCKETS

    BLOCKACTIONS = [PickUp(args) for args in combinations([OBJECTS])] +\
            [PutDown(args) for args in combinations([OBJECTS])] +\
            [Stack(args) for args in combinations([OBJECTS, OBJECTS])] +\
            [Unstack(args) for args in combinations([OBJECTS, OBJECTS])]
    PAINTACTIONS = [SprayPaint(args) for args in combinations([OBJECTS, SPRAYERS, COLORS])] +\
            [BrushPaint(args) for args in combinations([OBJECTS, BRUSHES, COLORS])] +\
            [LoadBrush(args) for args in combinations([BRUSHES, PAINTCANS, COLORS])] +\
            [WashBrush(args) for args in combinations([BRUSHES, WATERBUCKETS, COLORS])]
    ACTIONS = BLOCKACTIONS + PAINTACTIONS

    INITIAL = State([('onTable', 'blockA'),
                     ('onTable', 'brush'),
                     ('onTable', 'redPaintCan'),
                     ('clear', 'blockA'), 
                     ('clear', 'brush'), 
                     ('clear', 'redPaintCan'), 
                     ('clean', 'brush'),
                     ('paintsColor', 'redPaintCan', 'red'),
                     ('armEmpty')])
    GOAL = State([('colored', 'blockA', 'red'),
                     ('armEmpty')])

    blocksProblem = PlanProblem(INITIAL, GOAL, ACTIONS)
    blocksProblem.findPlan(maxNodes = 100000)

def blocks_paint_2():
    print 'Paint Blocks 2'
    BLOCKS = ['blockA', 'blockB']
    SPRAYERS = ['blueSprayer']
    BRUSHES = ['brush']
    PAINTCANS = ['redPaintCan']
    WATERBUCKETS = ['bucket']
    COLORS = ['red', 'blue']
    OBJECTS = BLOCKS + SPRAYERS + BRUSHES + PAINTCANS + WATERBUCKETS

    BLOCKACTIONS = [PickUp(args) for args in combinations([OBJECTS])] +\
            [PutDown(args) for args in combinations([OBJECTS])] +\
            [Stack(args) for args in combinations([OBJECTS, OBJECTS])] +\
            [Unstack(args) for args in combinations([OBJECTS, OBJECTS])]
    PAINTACTIONS = [SprayPaint(args) for args in combinations([OBJECTS, SPRAYERS, COLORS])] +\
            [BrushPaint(args) for args in combinations([OBJECTS, BRUSHES, COLORS])] +\
            [LoadBrush(args) for args in combinations([BRUSHES, PAINTCANS, COLORS])] +\
            [WashBrush(args) for args in combinations([BRUSHES, WATERBUCKETS, COLORS])]
    ACTIONS = BLOCKACTIONS + PAINTACTIONS

    INITIAL = State([('on', 'blockB', 'blockA'), 
                     ('onTable', 'blockA'),
                     ('onTable', 'blueSprayer'),
                     ('onTable', 'brush'),
                     ('onTable', 'redPaintCan'),
                     ('onTable', 'bucket'),
                     ('clear', 'blockB'), 
                     ('clear', 'blueSprayer'), 
                     ('clear', 'brush'), 
                     ('clear', 'redPaintCan'), 
                     ('clear', 'bucket'), 
                     ('paintsColor', 'blueSprayer', 'blue'),
                     ('clean', 'brush'),
                     ('paintsColor', 'redPaintCan', 'red'),
                     ('armEmpty')])
    GOAL = State([('colored', 'blockA', 'red'),
                     ('colored', 'blockB', 'blue')])

    blocksProblem = PlanProblem(INITIAL, GOAL, ACTIONS)
    blocksProblem.findPlan(maxNodes = 100000)

#blocks_stack()
#blocks_paint_1()
blocks_paint_2()

'''

## Factory Schedule example

TEMPS = ('cold', 'hot')
SHAPES = ('cylindrical', 'rectangular')
MACHINES = ('polisher', 'roller', 'lathe', 'grinder', 'punch',
            'drillPress', 'sprayPainter')
SURFACES = ('polished', 'rough', 'smooth')
ORIENTS = ('vertical', 'horizontal')
COLORS = ('white', 'black')

class Lathe(Action):
    actionName = 'Lathe'
    def resultStateAndCost(self, state, noDel = False):
        (part,) = self.args
        if state.check([('available', 'lathe'), ('free', part)]):
            return (state.addDelete([('objscheduled',),
                                     ('shape', part, 'cylindrical'),
                                     ('surface', part, 'rough')],
                                    # Deletes
                                    [('available', 'lathe'), ('free', part)] +\
                                    [('surface', part, x) for x in SURFACES] +\
                                    [('paint', part, x) for x in COLORS] +\
                                    [('shape', part, x) for x in SHAPES],
                                    noDel),
                    1)

PARTS = ['partA', 'partB', 'partC']

INITIAL = State([('free', part) for part in PARTS] +\
                [('available', machine) for machine in MACHINES] +\
                [('surface', part, 'rough') for part in PARTS] +\
                [('temperature', part, 'cold') for part in PARTS] +\
                [('shape', part, 'rectangular') for part in PARTS])

GOAL0 = [('surface', part, 'smooth') for part in PARTS] +\
        [('shape', part, 'cylindrical') for part in PARTS]

# Fewer machines
ACTS0 = [Lathe(args) for args in combinations([PARTS])] +\
        [Grind(args) for args in combinations([PARTS])] +\
        [Drill(args) for args in combinations([PARTS, ORIENTS])] +\
        [TimeStep([])]

factory0 = PlanProblem(INITIAL, GOAL0, ACTS0)

'''