from geometry import *
from DrawingWindowStandalone import *
from random import shuffle
from time import time
from heapq import heappush, heappop

#Sort functions (for best first search etc...)
def in_order(list):
  return list

def sort_heuristic(list, heuristic):
  return sorted(list, key=heuristic)
  
def shuffle_return(list):
  shuffle(list)
  return list
        
class World:
  def __init__(self, x, y, res, robot, obstacles, start, goal, angle_slices):
    self.x = x
    self.y = y
    self.res = res
    self.robot = robot
    self.obstacles = obstacles
    self.start = start
    self.goal = goal
    self.region = AABB(Point(0, 0), Point(x, y))
    self.angle_slices = angle_slices
    self.cspace_obstacles = {}

    assert self.valid_pose(self.start)
    assert self.valid_pose(self.goal)
    t1 = time()
    self.create_visibility_graph()
    print 'Visibility Graph took ', time() - t1, ' seconds'

  def valid_pose(self, pose):
    return pose.position.x <= self.x and pose.position.x >= 0 and pose.position.y <= self.y and pose.position.y >= 0 and pose.orientation in self.angle_slices and not self.collide(pose.position, self.get_cspace_obstacles(pose.orientation))
    
  def get_cspace_obstacles(self, orientation):
    if orientation not in self.cspace_obstacles:
      self.cspace_obstacles[orientation] = [] 
      for obst in [obst.cspace_object(self.robot.rotate(orientation)) for obst in self.obstacles]:
        self.cspace_obstacles[orientation].extend(obst.polys)
    return self.cspace_obstacles[orientation]

  def create_visibility_graph(self):
    self.visibility_graph = {}       
    for orientation in self.angle_slices:
      cspace_obstacles = self.get_cspace_obstacles(orientation)          
      for poly in cspace_obstacles:
        for i in range(len(poly.points)):
          if not self.translation_collide(poly.points[i-1], poly.points[i], cspace_obstacles):
            self.add_edge(Pose(poly.points[i-1], orientation), Pose(poly.points[i], orientation))
          
      for i in range(len(cspace_obstacles)):
        for j in range(i+1, len(cspace_obstacles)):
          for vert1 in cspace_obstacles[i].points:
            for vert2 in cspace_obstacles[j].points:
              if not self.translation_collide(vert1, vert2, cspace_obstacles):
                self.add_edge(Pose(vert1, orientation), Pose(vert2, orientation))
  
      for poly in cspace_obstacles:
        for vert in poly.points:
          if not self.translation_collide(self.start.position, vert, cspace_obstacles):
            self.add_edge(Pose(self.start.position, orientation), Pose(vert, orientation))
  
      for poly in cspace_obstacles:
        for vert in poly.points:
          if not self.translation_collide(vert, self.goal.position, cspace_obstacles):
            self.add_edge(Pose(vert, orientation), Pose(self.goal.position, orientation))
          
      if not self.translation_collide(self.start.position, self.goal.position, cspace_obstacles):
        self.add_edge(Pose(self.start.position, orientation), Pose(self.goal.position, orientation))
       
    if len(self.angle_slices) == 1:
      return
              
    for i in range(len(self.angle_slices)):
      orientation_1 = self.angle_slices[i-1]
      orientation_2 = self.angle_slices[i]            
      cspace_obstacles_1 = self.get_cspace_obstacles(orientation_1) 
      cspace_obstacles_2 = self.get_cspace_obstacles(orientation_2) 
      
      for poly1 in cspace_obstacles_1:
        for vert1 in poly1.points:
          if not self.rotation_collide(vert1, cspace_obstacles_1, cspace_obstacles_2):
            self.add_edge(Pose(vert1, orientation_1), Pose(vert1, orientation_2))      
            
            if Pose(vert1, orientation_2) in self.visibility_graph:
              continue
            
            for poly2 in cspace_obstacles_2:
              for vert2 in poly2.points:
                if not self.translation_collide(vert1, vert2, cspace_obstacles_2):
                  self.add_edge(Pose(vert1, orientation_2), Pose(vert2, orientation_2))
                  
            if not self.translation_collide(self.start.position, vert1, cspace_obstacles_2):
              self.add_edge(Pose(self.start.position, orientation_2), Pose(vert1, orientation_2))
              
            if not self.translation_collide(vert1, self.goal.position, cspace_obstacles_2):
              self.add_edge(Pose(vert1, orientation_2), Pose(self.goal.position, orientation_2))          
                  
      for poly2 in cspace_obstacles_2:
        for vert2 in poly2.points:
          if not self.rotation_collide(vert2, cspace_obstacles_1, cspace_obstacles_2):
            self.add_edge(Pose(vert2, orientation_1), Pose(vert2, orientation_2))

            if Pose(vert2, orientation_1) in self.visibility_graph:
              continue
            
            for poly1 in cspace_obstacles_1:
              for vert1 in poly1.points:
                if not self.translation_collide(vert1, vert2, cspace_obstacles_1):
                  self.add_edge(Pose(vert1, orientation_1), Pose(vert2, orientation_1))
                  
            if not self.translation_collide(self.start.position, vert2, cspace_obstacles_1):
              self.add_edge(Pose(self.start.position, orientation_1), Pose(vert2, orientation_1))
              
            if not self.translation_collide(vert2, self.goal.position, cspace_obstacles_1):
              self.add_edge(Pose(vert2, orientation_1), Pose(self.goal.position, orientation_1))
              
      if not self.rotation_collide(self.start.position, cspace_obstacles_1, cspace_obstacles_2):
        self.add_edge(Pose(self.start.position, orientation_1), Pose(self.start.position, orientation_2))

      if not self.rotation_collide(self.goal.position, cspace_obstacles_1, cspace_obstacles_2):
        self.add_edge(Pose(self.goal.position, orientation_1), Pose(self.goal.position, orientation_2))          
                  
  def add_edge(self, one, two):
    if one in self.visibility_graph:
      self.visibility_graph[one].append(two)
    else:
      self.visibility_graph[one] = [two]
      
    if two in self.visibility_graph:
      self.visibility_graph[two].append(one)
    else:
      self.visibility_graph[two] = [one]
    
  #Could later add support to ensure that actions happen in a region
  def collide(self, point, cspace_obstacles):
    return any([poly.contains(point) for poly in cspace_obstacles])
    
  def translation_collide(self, start, end, cspace_obstacles): #cspace_obstacles are already oriented for the correct slice
    movement = Line(start, end)
    return any([poly.collides(movement) for poly in cspace_obstacles])
    
  def rotation_collide(self, point, cspace_obstacles_1, cspace_obstacles_2): #cspace_obstacles are already oriented for the correct slices
    return self.collide(point, cspace_obstacles_1) or self.collide(point, cspace_obstacles_2)
    
  def draw_visibility_graph(self):
    for start in self.visibility_graph:
      for end in self.visibility_graph[start]:
        if start.position != end.position:
          Line(start.position, end.position).draw(self.window, color = 'blue', width = 2)
        
  def get_neighbors(self, pose):
    assert pose in self.visibility_graph
    return self.visibility_graph[pose]
        
  #Admissible heuristics
  def distance(self, one, two):
    return (two.position - one.position).length() + self.robot.get_radius()*abs(two.orientation - one.orientation)
  
  def manhattan(self, one, two):
    return abs(two.position.x - one.position.x) + abs(two.position.y - one.position.y) + self.robot.get_radius()*abs(two.orientation - one.orientation)
  
  def max_coordinate(self, one, two):
    return max(abs(two.position.x - one.position.x), abs(two.position.y - one.position.y), self.robot.get_radius()*abs(two.orientation - one.orientation))
  
  #Not admissible heuristics
  def bad_heuristic(self, one, two):
    return 10
  
  def fast_heuristic(self, one, two):
    return 10*self.distance(one, two)
  
  def retrace_path(self, path_map, node):
    success = []
    dist = 0
    retrace = node
    while retrace != None:
      success.append(retrace)
      if path_map[retrace] != None:
        dist += self.distance(retrace, path_map[retrace])
      retrace = path_map[retrace]
    success.reverse()    
    return success, dist
    
  def dfs(self, sort_fn = in_order):
    start = self.start
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
    start = self.start
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
    return None
      
  def bfs(self, sort_fn = in_order):
    start = self.start
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
    start = self.start
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
          new_dist = dist + self.distance(last, next)
          if (next not in visited or new_dist < visited[next][0]):
            heappush(paths, (new_dist, next)) #Substitute for decrease_key
            visited[next] = (new_dist, last)
    return None, None
    
  def astar(self, heuristic): #Can make UCS with heuristic of 0
    start = self.start
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
          new_dist = dist + self.distance(last, next)
          if (next not in visited or new_dist < visited[next][0]):
            heappush(paths, (new_dist + heuristic(next, self.goal), new_dist, next)) #Substitute for decrease_key
            visited[next] = (new_dist, last)
    return None, None
 
  def drawRobotPath(self, path, color = None):
    for i in range(len(path) - 1):
      Line(path[i].position, path[i+1].position).draw(self.window, color = color, width = 3)
      self.robot.at_pose(path[i+1]).draw(self.window, color)
      
  def drawPath(self, path, color = None):
    for i in range(len(path) - 1):
      Line(path[i].position, path[i+1].position).draw(self.window, color = color, width = 3)
 
  def display(self):     
    t1 = time()
    dfs_path, dfs_distance = self.dfs()
    #dfs_path, dfs_distance = self.dfs(sort_fn = lambda list: sort_heuristic(list, lambda item: self.manhattan(item, self.goal)))
    #dfs_path, dfs_distance = self.dfs(sort_fn = lambda list: sort_heuristic(list, lambda item: self.distance(item, self.goal)))
    #dfs_path, dfs_distance = self.dfs(sort_fn = shuffle_return)
    print 'DFS took ', time() - t1, ' seconds'
    
    t1 = time()
    bfs_path, bfs_distance = self.bfs()
    print 'BFS took ', time() - t1, ' seconds'

    t1 = time()    
    ucs_path, ucs_distance = self.ucs()
    print 'UCS took ', time() - t1, ' seconds'

    t1 = time()
    astar_1_path, astar_1_distance = self.astar(self.fast_heuristic)
    print 'A* Not Admissible took ', time() - t1, ' seconds'
    
    t1 = time()
    astar_2_path, astar_2_distance = self.astar(self.distance)
    print 'A* Admissible took ', time() - t1, ' seconds'
    
    self.window = DrawingWindow(600, 600, 0, self.x, 0, self.y, 'Visibility Graph Planning')
    #for orientation in self.cspace_obstacles:
    #  for obs in self.cspace_obstacles[orientation]:
    #    obs.draw(self.window, 'grey')
    
    self.robot.at_pose(self.start).draw(self.window, 'blue')
    for obs in self.obstacles:
      obs.draw(self.window, 'red')
    #self.draw_visibility_graph()
     
    if dfs_path == None:
      print 'No path to goal'
    else:
      print 'DFS path has length ', dfs_distance
      self.drawPath(dfs_path, color = 'purple')
        
      print 'BFS path has length ', bfs_distance
      self.drawPath(bfs_path, color = 'orange')
        
      print 'A* Not Admissible path has length ', astar_1_distance
      self.drawPath(astar_1_path, color='brown')
        
      print 'A* Admissible path has length ', astar_2_distance
      self.drawRobotPath(astar_2_path, color='green')

    self.goal.draw(self.window, color='gold', radius=3)

def world_easy():
  print 'Easy world'
  start = Pose(Point(2, 2), 0)
  robot = Object(Point(0, 0), [Polygon([Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)])])
  obstacles = []
  goal = Pose(Point(18, 18), -pi)

  angle_slices = uniform_angle_slices(8)
  for orient in [start.orientation, goal.orientation]:
    if orient not in angle_slices:
      angle_slices.append(orient)
  angle_slices.sort()
  
  world = World(20, 20, 1, robot, obstacles, start, goal, angle_slices)
  world.display()
  raw_input()
    
def world_medium():
  print 'Medium world'
  start = Pose(Point(5, 2), 0)
  robot = Object(Point(0, 0), [Polygon([Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)])])
  obstacles = [Object(Point(2.5, 7.5), [Polygon([Point(0, 0), Point(15, 0), Point(15, 5), Point(0, 5)])])]
  goal = Pose(Point(14, 18), pi/2)

  angle_slices = uniform_angle_slices(16)
  for orient in [start.orientation, goal.orientation]:
    if orient not in angle_slices:
      angle_slices.append(orient)
  angle_slices.sort()
  
  world = World(20, 20, 1, robot, obstacles, start, goal, angle_slices)
  world.display()
  raw_input()
    
def world_hard():
  print 'Hard world'
  start = Pose(Point(6, 0), 0)
  robot = Object(Point(0, 0), [Polygon([Point(-3, 1), Point(0, 0), Point(1, 3)])])
  obstacles = [ \
    Object(Point(6, 4.5), [Polygon([Point(0, 0), Point(2, 0), Point(2, 4), Point(0, 4)]), Polygon([Point(1.5, 1), Point(1.5, 3), Point(-1, 2)])]), \
    Object(Point(9, 6), [Polygon([Point(0, 0), Point(4, 2), Point(5, 4), Point(3, 3), Point(1, 2)])]), \
    Object(Point(8, 17), [Polygon([Point(0, 0), Point(-4, -4), Point(-4, -5), Point(-2, -3)])]), \
    Object(Point(6, 11), [Polygon([Point(0, 0), Point(3, .5), Point(0, 1)])]), \
    Object(Point(15, 13), [Polygon([Point(0, 0), Point(2, 2), Point(-3, 2)]), Polygon([Point(0, 0), Point(-2, -2), Point(3, -2)])])
  ]
  goal = Pose(Point(11, 9), -pi/4)

  angle_slices = uniform_angle_slices(16)
  for orient in [start.orientation, goal.orientation]:
    if orient not in angle_slices:
      angle_slices.append(orient)
  angle_slices.sort()
    
  world = World(20, 20, 1, robot, obstacles, start, goal, angle_slices)
  world.display()

  raw_input()

#world_easy()
world_medium()
#world_hard()