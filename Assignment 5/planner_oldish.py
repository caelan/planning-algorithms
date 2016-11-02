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
    def resultStateAndCost(self, state, noDel = False):
        if state.check(self.preconditions):
            return (state.addDelete(self.positiveEffects, self.negativeEffects, noDel), 1)
        else:
            return None
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
        self.ffTime = 0
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
        #return self.actionInstances
        return self.helpfulActions(state)
    def helpfulActions(self, start):
        if start in self.helpfulTable:
            return self.helpfulTable[start]

        level = 0
        fluents = {}
        for fluent in start.assertions:
            fluents[fluent] = (level, None)
        state = start
        unseenActions = self.actionInstances
        while True:
            level += 1
            if self.goalTest(state):
                level -= 1
                actions = set()
                for goal in self.goalAssertions:
                    if fluents[goal][0] == level and fluents[goal][1] is not None:
                        actions.add(fluents[goal][1])

                goals = set()
                while level > 1:
                    level -= 1
                    goals = set()
                    for action in actions:
                        for precondition in action.preconditions:
                            if fluents[precondition][0] == level:
                                goals.add(precondition)
                    for goal in self.goalAssertions:
                        if fluents[goal][0] == level:
                            goals.add(goal)

                    actions = set()
                    for goal in goals:
                        if fluents[goal][1] is not None:
                            actions.add(fluents[goal][1])

                #print start
                #print goals
                #quit()

                helpful= filter(lambda action: start.check(action.preconditions) and any([pe in goals for pe in action.positiveEffects]), self.actionInstances) #Cannot just take actions on first layer
                self.helpfulTable[start] = helpful
                return helpful
            added = []
            for action in unseenActions:
                pair = self.planSuccessor(state, action, noDel = True)
                if pair is None:
                    continue
                (next, cost) = pair
                for fluent in next.assertions:
                    if fluent not in fluents:
                        fluents[fluent] = (level, action)
                        added.append(fluent)
                del action
            if len(added) == 0:
                self.helpfulTable[start] = []
                return []
            state = State(added + [assertion for assertion in state.assertions])

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
    def relaxedPlanGraph(self, start):
        literals = [list(start.assertions)]
        actions = []
        state = start
        seenLiterals = set(start.assertions)
        unseenActions = self.actionInstances
        literalMap = {}
        for literal in start.assertions:
            literalMap[literal] = 0
        while True:
            levelLiterals = set()
            levelActions = []
            for action in unseenActions:
                pair = self.planSuccessor(state, action, noDel = True)
                if pair is None:
                    continue
                (next, cost) = pair
                for literal in next.assertions:
                    if literal not in seenLiterals:
                        levelLiterals.add(literal)
                        levelActions.append(action)
                        literalMap[literal] = len(literals)
                del action
            if len(levelActions) == 0:
                return None

            seenLiterals.update(levelLiterals)
            levelLiterals = list(levelLiterals)
            literals.append(levelLiterals)
            actions.append(levelActions)
            state = State(levelLiterals + [assertion for assertion in state.assertions])
            if self.goalTest(state):
                return (literals, actions, literalMap)

    def printPlanGraph(literals, actions):
        for level in range(len(literals)):
            if level != 0:
                print level-1, actions[level-1]
            print level, literals[level]

    def extractRelaxedPlan(self, start):
        planGraph = self.relaxedPlanGraph(start)
        if planGraph is None:
            return None
        (literals, actions, literalMap) = planGraph
        goals = [set() for i in range(len(literals))]
        relaxedPlan = [set() for i in range(len(actions))]
        marked = set()

        for goal in self.goalAssertions:
            if goal in literalMap and literalMap[goal] != 0:
                goals[literalMap[goal]].add(goal)

        for i in range(len(literals)-1, 0, -1):
            for goal in goals[i]:
                if (goal, i) in marked:
                    continue
                easiestAction, easietValue = None, None
                for action in actions[i-1]:
                    if goal in action.positiveEffects:
                        value = sum([literalMap[precondition] for precondition in action.preconditions])
                        if easietValue is None or value < easietValue:
                            easiestAction, easietValue = action, value
                relaxedPlan[i-1].add(easiestAction)
                for precondition in easiestAction.preconditions:
                    goals[literalMap[precondition]].add(precondition)
                for effect in easiestAction.positiveEffects:
                    marked.add((effect, i))
                    marked.add((effect, i-1))
        return relaxedPlan, goals[1]

    def ffHeuristic(self, start):
        print self.extractRelaxedPlan(start)
        quit()
        t0 = time()
        if start in self.ffTable:
            return self.ffTable[start]

        level = 0
        fluents = {}
        for fluent in start.assertions:
            fluents[fluent] = (level, None)
        state = start
        unseenActions = self.actionInstances
        while True:
            level += 1
            if self.goalTest(state):
                h_max = max([fluents[goal][0] for goal in self.goalAssertions])
                h_add = sum([fluents[goal][0] for goal in self.goalAssertions])  #Not quite the same as h_add because don't double count for intermediate fluents, but that makes it better

                #Computes FF by retracing through planning graph

                totalActions = []

                level -= 1
                actions = set()
                for goal in self.goalAssertions:
                    if fluents[goal][0] == level and fluents[goal][1] is not None:
                        actions.add(fluents[goal][1])

                planLength = len(actions)
                totalActions += actions
                while level >= 1:
                    level -= 1
                    goals = set()
                    for action in actions:
                        for precondition in action.preconditions:
                            if fluents[precondition][0] == level:
                                goals.add(precondition)
                    for goal in self.goalAssertions:
                        if fluents[goal][0] == level:
                            goals.add(goal)

                    actions = set()
                    for goal in goals:
                        if fluents[goal][1] is not None:
                            actions.add(fluents[goal][1])

                    planLength += len(actions)
                    totalActions += actions

                #print start
                #print totalActions
                #print planLength, h_add, h_max
                #assert planLength <= h_add

                h = planLength
                self.ffTable[start] = h
                self.ffTime += (time() - t0)
                return h
            added = []
            for action in unseenActions:
                pair = self.planSuccessor(state, action, noDel = True)
                if pair is None:
                    continue
                (next, cost) = pair
                for fluent in next.assertions:
                    if fluent not in fluents:
                        fluents[fluent] = (level, action)
                        added.append(fluent)
                del action
            if len(added) == 0:
                self.ffTable[start] = None
                self.ffTime += (time() - t0)
                return None
            state = State(added + [assertion for assertion in state.assertions])

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
        print 'Took', time() - t1, 'seconds for plan'
        print 'Took', self.ffTime, 'seconds for just FF'
        if result:
            (plan, cost) = result
            print 'Found plan with cost', cost
            for (action, state) in plan:
                if action: print action
            return plan
        else:
            print 'Failed to find a plan'

## Best-first heuristic search
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





## Blocks World actions
class PickUp(Action):
    actionName = 'PickUp'
    def __init__(self, args):
        Action.__init__(self, args)
        (obj,) = self.args
        self.preconditions = [('clear', obj), ('onTable',  obj), ('armEmpty')]
        self.positiveEffects = [('holding', obj)]
        self.negativeEffects = [('clear', obj), ('onTable', obj), ('armEmpty')]

class PutDown(Action):
    actionName = 'PutDown'
    def __init__(self, args):
        Action.__init__(self, args)
        (obj,) = self.args
        self.preconditions = [('holding', obj)]
        self.positiveEffects = [('clear', obj), ('onTable', obj), ('armEmpty')]
        self.negativeEffects = [('holding', obj)]

class Stack(Action):
    actionName = 'Stack'
    def __init__(self, args):
        Action.__init__(self, args)
        (obj, underObj, ) = self.args
        self.preconditions = [('holding', obj), ('clear', underObj)]
        self.positiveEffects = [('clear', obj), ('armEmpty'), ('on', obj, underObj)]
        self.negativeEffects = [('holding', obj), ('clear', underObj)]

class Unstack(Action):
    actionName = 'Unstack'
    def __init__(self, args):
        Action.__init__(self, args)
        (obj, underObj, ) = self.args
        self.preconditions = [('clear', obj), ('armEmpty'), ('on', obj, underObj)]
        self.positiveEffects = [('holding', obj), ('clear', underObj)]
        self.negativeEffects =  [('clear', obj), ('armEmpty'), ('on', obj, underObj)]

class SprayPaint(Action):
    actionName = 'SprayPaint'
    def __init__(self, args):
        Action.__init__(self, args)
        (obj, sprayer, color) = self.args
        self.preconditions = [('onTable', obj), ('clear', obj), ('paintsColor', sprayer, color), ('holding', sprayer)]
        self.positiveEffects = [('colored', obj, color)]
        self.negativeEffects =  []

class BrushPaint(Action):
    actionName = 'BrushPaint'
    def __init__(self, args):
        Action.__init__(self, args)
        (obj, brush, color) = self.args
        self.preconditions = [('onTable', obj), ('clear', obj), ('paintsColor', brush, color), ('holding', brush)]
        self.positiveEffects = [('colored', obj, color)]
        self.negativeEffects =  []

class LoadBrush(Action):
    actionName = 'LoadBrush'
    def __init__(self, args):
        Action.__init__(self, args)
        (brush, paintCan, color) = self.args
        self.preconditions = [('clean', brush), ('clear', paintCan), ('paintsColor', paintCan, color), ('holding', brush)]
        self.positiveEffects = [('paintsColor', brush, color)]
        self.negativeEffects = [('clean', brush)]

class WashBrush(Action):
    actionName = 'WashBrush'
    def __init__(self, args):
        Action.__init__(self, args)
        (brush, waterBucket, color) = self.args
        self.preconditions = [('clear', waterBucket), ('holding', brush), ('paintsColor', brush, color)]
        self.positiveEffects = [('clean', brush)]
        self.negativeEffects = [('paintsColor', brush, color)]

def blocks_stack():
    print 'Restack Blocks'
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

def blocks_paint_3():
    print 'Paint Blocks 3'
    BLOCKS = ['blockA', 'blockB', 'blockC']
    SPRAYERS = ['redSprayer', 'blueSprayer', 'greenSprayer']
    BRUSHES = ['brush']
    PAINTCANS = ['redPaintCan', 'bluePaintCan', 'greenPaintCan']
    WATERBUCKETS = ['bucket']
    COLORS = ['red', 'blue', 'green']
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
                     ('onTable', 'blockB'),
                     ('onTable', 'blockC'),
                     ('onTable', 'redSprayer'),
                     ('onTable', 'blueSprayer'),
                     ('onTable', 'greenSprayer'),
                     ('onTable', 'brush'),
                     ('onTable', 'redPaintCan'),
                     ('onTable', 'bluePaintCan'),
                     ('onTable', 'greenPaintCan'),
                     ('onTable', 'bucket'),
                     ('clear', 'blockA'),
                     ('clear', 'blockB'),
                     ('clear', 'blockC'),
                     ('clear', 'redSprayer'),
                     ('clear', 'blueSprayer'),
                     ('clear', 'greenSprayer'),
                     ('clear', 'brush'),
                     ('clear', 'redPaintCan'),
                     ('clear', 'bluePaintCan'),
                     ('clear', 'greenPaintCan'),
                     ('clear', 'bucket'),
                     ('paintsColor', 'redSprayer', 'red'),
                     ('paintsColor', 'blueSprayer', 'blue'),
                     ('paintsColor', 'greenSprayer', 'green'),
                     ('clean', 'brush'),
                     ('paintsColor', 'redPaintCan', 'red'),
                     ('paintsColor', 'bluePaintCan', 'blue'),
                     ('paintsColor', 'greenPaintCan', 'green'),                     
                     ('armEmpty')])
    GOAL = State([('colored', 'blockA', 'red'),
                  ('colored', 'blockB', 'blue'),
                  ('colored', 'blockC', 'green'),
                  ('clean', 'brush'),
                  ('armEmpty')])

    blocksProblem = PlanProblem(INITIAL, GOAL, ACTIONS)
    blocksProblem.findPlan(maxNodes = 100000)

blocks_stack()
#blocks_paint_1()
#blocks_paint_2()
#blocks_paint_3()
