from geometry import *
from DrawingWindowStandalone import *
from random import shuffle, random, randint, uniform
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

class Range:
  def __init__(self, low, high, wrap_around = False):
    self.low = low
    self.high = high
    self.wrap_around = wrap_around
  def difference(self, one, two):
    if self.wrap_around:
      if one < two:
        if abs(two - one) < abs((self.low - one) + (two - self.high)):
          return two - one
        else:
          return (self.low - one) + (two - self.high)
      else:
        if abs(two - one) < abs((self.high - one) + (two - self.low)):
          return two - one
        else:
          return (self.high - one) + (two - self.low)
    else:
      return two - one
  def in_range(self, value):
    if self.wrap_around:
      altered = value
      while not (altered >= self.low and altered <= self.high):
        if altered > self.high:
          altered -= (self.high - self.low)
        else:
          altered += (self.high - self.low)
      return altered
    else:
      if self.contains(value):
        return value
      else:
        return None
  def sample(self):
    return (self.high - self.low)*random() + self.low
  def contains(self, x):
    return self.wrap_around or (x >= self.low and x <= self.high)

class TreeNode:
  def __init__(self, value, parent = None):
    self.value = value
    self.parent = parent
    self.children = []

class RRT:
  def __init__(self, root, cspace):
    self.root = root
    self.cspace = cspace
    self.size = 1
  def add_configuration(self, parent_node, child_value):
    child_node = TreeNode(child_value, parent_node)
    parent_node.children.append(child_node)
    self.size +=1 
    return child_node
  def nearest(self, configuration): #scipy.spatial for the k-d tree isn't working on my machine
    assert self.cspace.valid_configuration(configuration)
    def recur(node):
      closest, distance = node, self.cspace.distance(node.value, configuration)
      for child in node.children:
        (child_closest, child_distance) = recur(child)
        if child_distance < distance:
          closest = child_closest
          distance = child_distance
      return closest, distance
    return recur(self.root)[0]
  def draw(self, window, color = 'black'):
    def recur(node):
      node_point = Point(node.value[0], node.value[1])
      for child in node.children:
        child_point = Point(child.value[0], child.value[1])
        Line(node_point, child_point).draw(window, color=color, width=1)
        recur(child)
    recur(self.root)

class ConfigurationSpace:
  def __init__(self, cspace_ranges, start, robot_distance):
    self.cspace_ranges = cspace_ranges
    self.robot_distance = robot_distance
  def distance(self, one, two):
    return self.robot_distance(tuple([self.cspace_ranges[i].difference(one[i], two[i]) for i in range(len(self.cspace_ranges))]))
  def path(self, one, two, samples):
    assert self.valid_configuration(one) and self.valid_configuration(two)
    linear_interpolation = [self.cspace_ranges[i].difference(one[i], two[i])/(samples - 1.0) for i in range(len(self.cspace_ranges))]
    path = [one]
    for s in range(1, samples-1):
      sample = tuple([self.cspace_ranges[i].in_range(one[i] + s*linear_interpolation[i]) for i in range(len(self.cspace_ranges))])
      #if not self.valid_configuration(sample): #Rectangular space => not needed
      #  return path
      path.append(sample)
    return path + [two]
  def sample(self):
    return tuple([r.sample() for r in self.cspace_ranges])
  def valid_configuration(self, config):
    return len(config) == len(self.cspace_ranges) and all([self.cspace_ranges[i].contains(config[i]) for i in range(len(self.cspace_ranges))])

class World:
  def __init__(self, x, y, robot, obstacles, start, goal, cspace, display_tree = False):
    self.x = x
    self.y = y
    self.robot = robot
    self.obstacles = obstacles
    self.start = start
    self.goal = goal
    self.region = AABB(Point(0, 0), Point(x, y))
    self.cspace = cspace
    self.display_tree = display_tree

    assert self.valid_configuration(self.start)
    assert self.valid_configuration(self.goal)

  def valid_configuration(self, configuration):
    return self.cspace.valid_configuration(configuration) and not self.collide(configuration)
            
  def collide(self, configuration):
    config_robot = self.robot.configuration(configuration)
    return any([config_robot.collides(obstacle) for obstacle in self.obstacles]) or not self.region.contains(config_robot)
    
  def generate_random_regular_poly(self, num_verts, radius, angle = uniform(-pi, pi)):
    (min_verts, max_verts) = num_verts
    (min_radius, max_radius) = radius
    assert not (min_verts < 3 or min_verts > max_verts or min_radius <= 0 or min_radius > max_radius)

    reference = Point(random()*self.x, random()*self.y)
    distance = uniform(min_radius, max_radius)
    sides = randint(min_verts, max_verts)
    obj = Object(reference, [Polygon([Point(distance*cos(angle + 2*n*pi/sides), distance*sin(angle + 2*n*pi/sides)) for n in range(sides)])])
    if any([obj.collides(current) for current in self.obstacles]) or obj.collides(self.robot.configuration(self.start)) or obj.collides(self.robot.configuration(self.goal)) or not self.region.contains(obj):
      self.generate_random_regular_poly(num_verts, radius, angle=angle)
    else:
      self.obstacles.append(obj)

  def generate_random_poly(self, num_verts, radius):
    (min_verts, max_verts) = num_verts
    (min_radius, max_radius) = radius
    assert not (min_verts < 3 or min_verts > max_verts or min_radius <= 0 or min_radius > max_radius)

    reference = Point(random()*self.x, random()*self.y)
    verts = randint(min_verts, max_verts)
    points = [Point(0, 0)]
    for i in range(verts):
      angle = 2*pi*random()
      points.append(((max_radius - min_radius)*random() + min_radius)*Point(cos(angle), sin(angle)))
    obj = Object(reference, [Polygon(convex_hull(points))])
    if any([obj.collides(current) for current in self.obstacles]) or obj.collides(self.robot.configuration(self.start)) or obj.collides(self.robot.configuration(self.goal)) or not self.region.contains(obj):
      self.generate_random_poly(num_verts, radius)
    else:
      self.obstacles.append(obj)

  def test_path(self, start, end, samples = 10): 
    path = self.cspace.path(start, end, samples)
    for configuration in path:
      if self.collide(configuration):
        return False
    return True 

  def safe_path(self, start, end, samples = 10): 
    path = self.cspace.path(start, end, samples)
    safe_path = []
    for configuration in path:
      if self.collide(configuration):
        return safe_path
      safe_path.append(configuration)
    return safe_path 

  def smooth_path(self, path, attempts = 100):
    smoothed_path = path
    for attempt in range(attempts):
      if len(smoothed_path) <= 2:
        return smoothed_path
      i = randint(0, len(smoothed_path) - 1)
      j = randint(0, len(smoothed_path) - 1)
      if i == j or abs(i-j) == 1:
        continue

      one, two = i, j
      if j < i:
        one, two = j, i
      if self.test_path(smoothed_path[one], smoothed_path[two], samples = 10):
        smoothed_path = smoothed_path[:one+1] + smoothed_path[two:]
    return smoothed_path

  def path_distance(self, path):
    distance = 0
    for i in range(len(path)-1):
      distance += self.cspace.distance(path[i], path[i+1])
    return distance

  def rrt_planning(self, max_iterations = 1000, goal_sample = .05):
    rrt = RRT(TreeNode(self.start), self.cspace)
    for iteration in range(max_iterations + 1):
      sample = self.cspace.sample()
      if random() < goal_sample or iteration == max_iterations:
        sample = self.goal
      closest_node = rrt.nearest(sample)
      safe_path = self.safe_path(closest_node.value, sample)
      last_node = closest_node
      for i in range(1, len(safe_path)):
        last_node = rrt.add_configuration(last_node, safe_path[i])

      if last_node.value == self.goal:
        success_path = [last_node.value]
        while last_node.parent is not None:
          last_node = last_node.parent
          success_path.append(last_node.value)
        success_path.reverse()
        self.rrts = [rrt]
        return success_path
    self.rrts = [rrt]
    return None

  def bidirectional_rrt_planning(self, max_iterations = 1000, goal_sample = .05):
    rrt_start = RRT(TreeNode(self.start), self.cspace)
    rrt_goal = RRT(TreeNode(self.goal), self.cspace)

    one = rrt_start
    two = rrt_goal
    for iteration in range(max_iterations + 1):
      if rrt_start.size < rrt_goal.size:
        one = rrt_start
        two = rrt_goal
      else:
        two = rrt_start
        one = rrt_goal

      sample = self.cspace.sample()
      one_closest_node = one.nearest(sample)
      one_safe_path = self.safe_path(one_closest_node.value, sample)
      one_last_node = one_closest_node
      for i in range(1, len(one_safe_path)):
        one_last_node = one.add_configuration(one_last_node, one_safe_path[i])

      two_closest_node = two.nearest(one_last_node.value)
      two_safe_path = self.safe_path(two_closest_node.value, sample)
      two_last_node = two_closest_node
      for i in range(1, len(two_safe_path)):
        two_last_node = two.add_configuration(two_last_node, two_safe_path[i])

      if one_last_node.value == two_last_node.value:
        success_path = [one_last_node.value]
        while one_last_node.parent is not None:
          one_last_node = one_last_node.parent
          success_path.append(one_last_node.value)
        success_path.reverse()

        while two_last_node.parent is not None:
          two_last_node = two_last_node.parent
          success_path.append(two_last_node.value)
        self.rrts = [one, two]
        return success_path
    self.rrts = [one, two]
    return None

  def draw_robot_path(self, path, color = None):
    for i in range(1, len(path) - 1):
      self.robot.configuration(path[i]).draw(self.window, color)
 
  def display(self):         
    self.window = DrawingWindow(600, 600, 0, self.x, 0, self.y, 'RRT Planning')  
    for obs in self.obstacles:
      obs.draw(self.window, 'red')
    self.robot.configuration(self.start).draw(self.window, 'orange')

    t1 = time()
    #path = self.rrt_planning()
    path = self.bidirectional_rrt_planning()
    print 'RRT took', time() - t1, 'seconds'
    if self.display_tree:
      for rrt in self.rrts:
        rrt.draw(self.window)
    if path == None:
      print 'No path found'
    else:
      print 'Path found with ' + str(len(path)-1) + ' movements of distance ', self.path_distance(path)
      smooth_path = self.smooth_path(path)
      print 'Smoothed path found with ' + str(len(smooth_path)-1) + ' movements of distance ', self.path_distance(smooth_path)
      self.draw_robot_path(path, color = 'yellow')
      self.draw_robot_path(smooth_path, color = 'gold')
    
    self.robot.configuration(self.goal).draw(self.window, 'green')

def random_robot_world():
  print 'Random Robot World'
  robot = Robot([ \
    Polygon([Point(-2, -1), Point(2, -1), Point(2, 1), Point(-2, 1)]),
    Polygon([Point(-1, 1), Point(1, 1), Point(0, 2)])
  ])

  start = (5, 5, 0)
  goal = (15, 15, pi/2)

  cspace = ConfigurationSpace([ \
    Range(0, 20), \
    Range(0, 20), \
    Range(-pi, pi, wrap_around = True)
  ], start, robot.distance)

  obstacles = []   
  world = World(20, 20, robot, obstacles, start, goal, cspace, display_tree = True)
  for i in range(3):
    world.generate_random_poly((3, 15), (1, 4))
  world.display()

  raw_input()

def robot_world():
  print 'Robot World'
  robot = Robot([ \
    Polygon([Point(-2, -1), Point(2, -1), Point(2, 1), Point(-2, 1)]),
    Polygon([Point(-1, 1), Point(1, 1), Point(0, 2)])
  ])

  start = (5.0, 5.0, 0)
  goal = (15.0, 15.0, pi/2)

  cspace = ConfigurationSpace([ \
    Range(0, 20), \
    Range(0, 20), \
    Range(-pi, pi, wrap_around = True)
  ], start, robot.distance)

  obstacles = [Object(Point(10, 10), [Polygon([Point(0, -2), Point(2, 0), Point(0, 2), Point(-2, 0)])])]   
  world = World(20, 20, robot, obstacles, start, goal, cspace, display_tree = True)
  world.display()

  raw_input()
    
def create_link(length, width):
  return Polygon([Point(0, -width/2.0), Point(length, -width/2.0), Point(length, width/2.0), Point(0, width/2.0)])

def robot_arm_world():
  print 'Robot Arm World'
  robot = RobotArm(Point(10, 10), [ \
    (Point(-.5, 0), create_link(4, 1)), \
    (Point(3.5, 0), create_link(3, 1)), \
    (Point(2.5, 0), create_link(2, 1))
  ])

  start = (pi/2, pi/2, -pi/4)
  goal = (pi/4, -pi/6, -pi/3)

  cspace = ConfigurationSpace([ \
    Range(-pi/2, pi/2), \
    Range(-pi/2, pi/2), \
    Range(-pi/2, pi/2)
  ], start, robot.distance)
  
  """
  cspace = ConfigurationSpace([ \
    Range(-pi, pi, wrap_around = True), \
    Range(-pi, pi, wrap_around = True), \
    Range(-pi, pi, wrap_around = True)
  ], start, robot.distance)
  """

  obstacles = [Object(Point(12, 17.5), [Polygon([Point(0, -2), Point(2, 0), Point(0, 2), Point(-2, 0)])])]   
  world = World(20, 20, robot, obstacles, start, goal, cspace)
  world.display()

  raw_input()

def random_robot_arm_world():
  print 'Random Robot Arm World'
  robot = RobotArm(Point(10, 10), [ \
    (Point(-.5, 0), create_link(4, 1)), \
    (Point(3.5, 0), create_link(2, 1)), \
    (Point(1.5, 0), create_link(4, 1))
  ])

  start = (pi/2, pi/2, -pi/4)
  goal = (-pi/4, -pi/6, -pi/3)

  
  cspace = ConfigurationSpace([ \
    Range(-pi/2, pi/2), \
    Range(-pi/2, pi/2), \
    Range(-pi/2, pi/2)
  ], start, robot.distance)
  
  """
  cspace = ConfigurationSpace([ \
    Range(-pi, pi, wrap_around = True), \
    Range(-pi, pi, wrap_around = True), \
    Range(-pi, pi, wrap_around = True)
  ], start, robot.distance)
  """

  obstacles = []    
  world = World(20, 20, robot, obstacles, start, goal, cspace)
  for i in range(3):
    world.generate_random_poly((3, 15), (1, 4))
  world.display()

  raw_input()

#robot_world()
#random_robot_world()
#robot_arm_world()
random_robot_arm_world()