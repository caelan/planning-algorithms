from geometry import *
from DrawingWindowStandalone import *
from random import shuffle
from time import time
from heapq import heappush, heappop

#Admissible heuristics
def distance(one, two):
  return (two - one).length()
  
def manhattan(one, two):
  return abs(two.x - one.x) + abs(two.y - one.y)
  
def max_coordinate(one, two):
  return max(abs(two.x - one.x), abs(two.y - one.y))
  
#Not admissible heuristics
def bad_heuristic(one, two):
  return 10
  
def fast_heuristic(one, two):
  return 5*manhattan(one, two)
  
#Sort functions (for best first search etc...)
def in_order(list):
  return list

def sort_heuristic(list, heuristic):
  return sorted(list, key=heuristic)
  
def shuffle_return(list):
  shuffle(list)
  return list

class World:
  def __init__(self, x, y, res, robot, obstacles, goal):
    self.x = x
    self.y = y
    self.res = res
    self.robot = robot
    self.obstacles = obstacles
    self.goal = goal
    self.region = AABB(Point(0, 0), Point(x, y))
    self.robot_table = {}

    assert self.valid_location(self.goal)

  def valid_location(self, point):
    return within(point.x-self.res*int(point.x/self.res)) and within(point.y-self.res*int(point.y/self.res)) and point.x <= self.x and point.x >= 0 and point.y <= self.y and point.y >= 0

  def get_neighbors(self, point):
    assert self.valid_location(point)
    movements = [Point(0, self.res), Point(0, -self.res), Point(self.res, 0), Point(-self.res, 0)]
    #movements += [Point(self.res, self.res), Point(self.res, -self.res), Point(-self.res, -self.res), Point(-self.res, self.res)] #Diagonals too
    return [move + point for move in movements if self.valid_location(move + point)]

  def get_swept_robot(self, start, end):
    #return self.robot.sweep_move(start, end)
    if (start, end) not in self.robot_table:
      self.robot_table[(start, end)] = self.robot.sweep_move(start, end)
    return self.robot_table[(start, end)]
    
  def collide(self, start, end):
    moved_robot = self.get_swept_robot(start, end)
    return any([moved_robot.collides(obs) for obs in self.obstacles]) or not all([self.region.contains(poly, edges = True) for poly in moved_robot.polys])
        
  def retrace_path(self, path_map, node):
    success = []
    retrace = node
    while retrace != None:
      success.append(retrace)
      retrace = path_map[retrace]
    success.reverse()
    return success
    
  def dfs(self, sort_fn = in_order):
    start = self.robot.reference
    paths = [start]
    visited = {}
    visited[start] = None
    
    def recur(last):
      if last == self.goal:
        return True
      for next in sort_fn(self.get_neighbors(last)):
        if next not in visited and not self.collide(last, next):
          visited[next] = last
          if recur(next):
            return True
      return False
   
    if not recur(start):
      return None
    else:
      return self.retrace_path(visited, self.goal)
    
  def dfs_iterative(self, sort_fn = in_order): #Changing a queue to a stack is just too amusing...
    start = self.robot.reference
    paths = [start]
    visited = {}
    visited[start] = None
    while len(paths) != 0:
        last = paths.pop()
        if last == self.goal:
          return self.retrace_path(visited, last)
        for next in sort_fn(self.get_neighbors(last)):
          if next not in visited and not self.collide(last, next):
            paths.append(next)
            visited[next] = last
    return None
      
  def bfs(self, sort_fn = in_order):
    start = self.robot.reference
    paths = [start]
    visited = {}
    visited[start] = None
    while len(paths) != 0:
        last = paths.pop(0) #Remove 0 to make DFS
        if last == self.goal:
          return self.retrace_path(visited, last)
        for next in sort_fn(self.get_neighbors(last)):
          if next not in visited and not self.collide(last, next):
            paths.append(next)
            visited[next] = last
    return None
  
  def retrace_weighted_path(self, path_map, node):
    success = []
    retrace = node
    while retrace != None:
      success.append(retrace)
      retrace = path_map[retrace][1]
    success.reverse()
    return success
  
  def ucs(self):
    start = self.robot.reference
    paths = [(0, start)]
    visited = {}
    visited[start] = (0, None)
    while len(paths) != 0:
        (dist, last) = heappop(paths)
        if visited[last][0] != dist: #Becuase I'm not using a fibonacci heap, bad paths may be in the queue
          continue
        if last == self.goal:
          return self.retrace_weighted_path(visited, last)
        for next in self.get_neighbors(last):
          new_dist = dist + distance(last, next)
          if (next not in visited or new_dist < visited[next][0]) and not self.collide(last, next):
            heappush(paths, (new_dist, next)) #Substitute for decrease_key
            visited[next] = (new_dist, last)
    return None
    
  def astar(self, heuristic): #Can make UCS with heuristic of 0
    start = self.robot.reference
    paths = [(heuristic(start, self.goal), 0, start)]
    visited = {}
    visited[start] = (0, None)
    while len(paths) != 0:
        (h_dist, dist, last) = heappop(paths)
        if visited[last][0] != dist: #Becuase I'm not using a fibonacci heap, bad paths may be in the queue
          continue
        if last == self.goal:
          return self.retrace_weighted_path(visited, last)
        for next in self.get_neighbors(last):
          new_dist = dist + distance(last, next)
          if (next not in visited or new_dist < visited[next][0]) and not self.collide(last, next):
            heappush(paths, (new_dist + heuristic(next, self.goal), new_dist, next)) #Substitute for decrease_key
            visited[next] = (new_dist, last)
    return None
 
  def drawPath(self, path, color = None):
    for i in range(len(path) - 1):
      Line(path[i], path[i+1]).draw(self.window, color = color, width = 3)
 
  def display(self):     
    #Assuming the grid structure, BFS and UCS should both return an optimal path
    t1 = time()
    #dfs_path = self.dfs()
    #dfs_path = self.dfs(sort_fn = lambda list: sort_heuristic(list, lambda item: manhattan(item, self.goal)))
    #dfs_path = self.dfs(sort_fn = lambda list: sort_heuristic(list, lambda item: distance(item, self.goal)))
    dfs_path = self.dfs(sort_fn = shuffle_return)
    print 'DFS took ', time() - t1, ' seconds'
    
    t1 = time()
    bfs_path = self.bfs()
    print 'BFS took ', time() - t1, ' seconds'

    t1 = time()    
    ucs_path = self.ucs()
    print 'UCS took ', time() - t1, ' seconds'

    t1 = time()
    astar_1_path = self.astar(fast_heuristic)
    print 'A* Not Admissible took ', time() - t1, ' seconds'
    
    t1 = time()
    astar_2_path = self.astar(manhattan)
    print 'A* Manhattan took ', time() - t1, ' seconds'
    
    self.window = DrawingWindow(600, 600, 0, self.x, 0, self.y, 'Discrete Planning')
    for r in range(int(self.x/self.res)):
      Line(Point(0, r*self.res), Point(self.x, r*self.res)).draw(self.window, color='black', width=1)
    for c in range(int(self.y/self.res)):
      Line(Point(c*self.res, 0), Point(c*self.res, self.y)).draw(self.window, color='black', width=1)

    self.robot.draw(self.window, 'blue')
    for obs in self.obstacles:
      obs.draw(self.window, 'red')
     
    if dfs_path == None:
      print 'No path to goal'
    else:
      print 'DFS path has length ', (len(dfs_path) - 1)*self.res
      self.drawPath(dfs_path, color = 'purple')
        
      print 'A* Not Admissible path has length ', (len(astar_1_path) - 1)*self.res 
      self.drawPath(astar_1_path, color='brown')
        
      print 'Optimal path has length ', (len(astar_2_path) - 1)*self.res #path could be returned by bfs, ucs, or astar_2 with equal weight edges
      self.drawPath(astar_2_path, color='green')

    self.goal.draw(self.window, color='gold', radius=3)

def world_easy():
  print 'Easy world'
  start = Point(1, 1)
  robot = Object(start, [Polygon([Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)])])
  obstacles = []
  goal = Point(19, 19)

  world = World(20, 20, 1, robot, obstacles, goal)
  world.display()
  raw_input()
    
def world_medium():
  print 'Medium world'
  start = Point(1, 1)
  robot = Object(start, [Polygon([Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)])])
  obstacles = [Object(Point(0, 7), [Polygon([Point(0, 0), Point(15, 0), Point(15, 5), Point(0, 5)])])]
  goal = Point(1, 19)

  world = World(20, 20, 1, robot, obstacles, goal)
  world.display()
  raw_input()
    
def world_hard():
  print 'Hard world'
  start = Point(4, 0)
  robot = Object(start, [Polygon([Point(-3, 1), Point(0, 0), Point(1, 3)])])
  obstacles = [ \
    Object(Point(4, 4), [Polygon([Point(0, 0), Point(2, 0), Point(2, 4), Point(0, 4)]), Polygon([Point(1, 1), Point(1, 3), Point(-1, 2)])]), \
    Object(Point(7, 6), [Polygon([Point(0, 0), Point(1, 2), Point(3, 3), Point(5, 4), Point(4, 2)])]), \
    Object(Point(6, 17), [Polygon([Point(0, 0), Point(-4, -4), Point(-4, -5), Point(-2, -3)])]), \
    Object(Point(4, 11), [Polygon([Point(0, 0), Point(3, .5), Point(0, 1)])]), \
    Object(Point(13, 13), [Polygon([Point(0, 0), Point(2, 2), Point(-3, 2)]), Polygon([Point(0, 0), Point(-2, -2), Point(3, -2)])])
  ]
  goal = Point(9, 9)
  
  world = World(20, 20, 1, robot, obstacles, goal)
  world.display()
  raw_input()

#world_easy()
#world_medium()
world_hard()