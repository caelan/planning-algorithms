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
    self.start = robot.reference
    self.goal = goal
    self.region = AABB(Point(0, 0), Point(x, y))
    assert self.valid_location(self.goal)
    t1 = time()
    self.create_visibility_graph()
    print 'Visibility Graph took ', time() - t1, ' seconds'

  def valid_location(self, point):
    return point.x <= self.x and point.x >= 0 and point.y <= self.y and point.y >= 0
    
  def create_cspace_obstacles(self):
    self.cspace_obstacles = [obst.cspace_object(self.robot) for obst in self.obstacles]
    return self.cspace_obstacles

  def create_visibility_graph(self):
    self.visibility_graph = {}   
    cspace_obstacles = []
    for obst in self.create_cspace_obstacles():
      cspace_obstacles.extend(obst.polys)
          
    for poly in cspace_obstacles:
      for i in range(len(poly.points)):
        if not self.collide(poly.points[i-1], poly.points[i], cspace_obstacles): #Don't test edge against the polygon itself
          self.add_edge(poly.points[i-1], poly.points[i])
          
    for i in range(len(cspace_obstacles)):
      for j in range(i+1, len(cspace_obstacles)):
        for vert1 in cspace_obstacles[i].points:
          for vert2 in cspace_obstacles[j].points:
            if not self.collide(vert1, vert2, cspace_obstacles):
              self.add_edge(vert1, vert2)
  
    for poly in cspace_obstacles:
      for vert in poly.points:
        if not self.collide(self.start, vert, cspace_obstacles):
          self.add_edge(self.start, vert)
  
    for poly in cspace_obstacles:
      for vert in poly.points:
        if not self.collide(vert, self.goal, cspace_obstacles):
          self.add_edge(vert, self.goal)
          
    if not self.collide(self.start, self.goal, cspace_obstacles):
      self.add_edge(self.start, self.goal)
          
  def draw_visibility_graph(self):
    for start in self.visibility_graph:
      for end in self.visibility_graph[start]:
        Line(start, end).draw(self.window, color = 'blue', width = 2)
    
  def add_edge(self, one, two):
    if one in self.visibility_graph:
      self.visibility_graph[one].append(two)
    else:
      self.visibility_graph[one] = [two]
      
    if two in self.visibility_graph:
      self.visibility_graph[two].append(one)
    else:
      self.visibility_graph[two] = [one]
    
  def collide(self, start, end, cspace_obstacles):
    movement = Line(start, end)
    return any([poly.collides(movement) for poly in cspace_obstacles]) or not self.region.contains(movement, edges = True) #need cspace for region
        
  def get_neighbors(self, point):
    assert point in self.visibility_graph
    return self.visibility_graph[point]
        
  def retrace_path(self, path_map, node):
    success = []
    dist = 0
    retrace = node
    while retrace != None:
      success.append(retrace)
      if path_map[retrace] != None:
        dist += distance(retrace, path_map[retrace])
      retrace = path_map[retrace]
    success.reverse()
    return success, dist
    
  def dfs(self, sort_fn = in_order):
    start = self.robot.reference
    paths = [start]
    visited = {}
    visited[start] = None
    
    def recur(last):
      if last == self.goal:
        return True
      for next in sort_fn(self.get_neighbors(last)):
        if next not in visited:
          visited[next] = last
          if recur(next):
            return True
      return False
   
    if not recur(start):
      return None, None
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
          if next not in visited:
            paths.append(next)
            visited[next] = last
    return None, None
      
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
          if next not in visited:
            paths.append(next)
            visited[next] = last
    return None, None
  
  def retrace_weighted_path(self, path_map, node):
    success = []
    retrace = node
    while retrace != None:
      success.append(retrace)
      retrace = path_map[retrace][1]
    success.reverse()
    return success, path_map[node][0]
  
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
          if (next not in visited or new_dist < visited[next][0]):
            heappush(paths, (new_dist, next)) #Substitute for decrease_key
            visited[next] = (new_dist, last)
    return None, None
    
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
          if (next not in visited or new_dist < visited[next][0]):
            heappush(paths, (new_dist + heuristic(next, self.goal), new_dist, next)) #Substitute for decrease_key
            visited[next] = (new_dist, last)
    return None, None
 
  def drawPath(self, path, color = None):
    for i in range(len(path) - 1):
      Line(path[i], path[i+1]).draw(self.window, color = color, width = 3)
 
  def display(self):     
    t1 = time()
    dfs_path, dfs_distance = self.dfs()
    #dfs_path, dfs_distance = self.dfs(sort_fn = lambda list: sort_heuristic(list, lambda item: manhattan(item, self.goal)))
    #dfs_path, dfs_distance = self.dfs(sort_fn = lambda list: sort_heuristic(list, lambda item: distance(item, self.goal)))
    #dfs_path, dfs_distance = self.dfs(sort_fn = shuffle_return)
    print 'DFS took ', time() - t1, ' seconds'
    
    t1 = time()
    bfs_path, bfs_distance = self.bfs()
    print 'BFS took ', time() - t1, ' seconds'

    t1 = time()    
    ucs_path, ucs_distance = self.ucs()
    print 'UCS took ', time() - t1, ' seconds'

    t1 = time()
    astar_1_path, astar_1_distance = self.astar(fast_heuristic)
    print 'A* Not Admissible took ', time() - t1, ' seconds'
    
    t1 = time()
    astar_2_path, astar_2_distance = self.astar(manhattan)
    print 'A* Manhattan took ', time() - t1, ' seconds'
    
    self.window = DrawingWindow(600, 600, 0, self.x, 0, self.y, 'Visibility Graph Planning')
    for obs in self.cspace_obstacles:
      obs.draw(self.window, 'grey')
    
    self.robot.draw(self.window, 'blue')
    for obs in self.obstacles:
      obs.draw(self.window, 'red')
    self.draw_visibility_graph()
     
    if dfs_path == None:
      print 'No path to goal'
    else:
      print 'DFS path has length ', dfs_distance
      self.drawPath(dfs_path, color = 'purple')
        
      print 'BFS path has length ', bfs_distance
      self.drawPath(bfs_path, color = 'orange')
        
      print 'A* Not Admissible path has length ', astar_1_distance
      self.drawPath(astar_1_path, color='brown')
        
      print 'Optimal path has length ', astar_2_distance
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
  obstacles = [Object(Point(.5, 7), [Polygon([Point(0, 0), Point(15, 0), Point(15, 5), Point(0, 5)])])]
  goal = Point(1, 19)

  world = World(20, 20, 1, robot, obstacles, goal)
  world.display()
  raw_input()
    
def world_hard():
  print 'Hard world'
  start = Point(4, 0)
  robot = Object(start, [Polygon([Point(-3, 1), Point(0, 0), Point(1, 3)])])
  obstacles = [ \
    Object(Point(4, 4.5), [Polygon([Point(0, 0), Point(2, 0), Point(2, 4), Point(0, 4)]), Polygon([Point(1.5, 1), Point(1.5, 3), Point(-1, 2)])]), \
    Object(Point(7, 6), [Polygon([Point(0, 0), Point(4, 2), Point(5, 4), Point(3, 3), Point(1, 2)])]), \
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