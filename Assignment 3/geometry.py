from math import sqrt, cos, sin, atan2, pi
#from scipy.spatial import ConvexHull

def within(exp, threshold = .001):
  return abs(exp) < threshold
  
def standard_angle(angle): #Converts angle to [-pi, pi]
  while angle < -pi:
    angle += 2*pi
  while angle > pi:
    angle -= 2*pi
  return angle
  
def uniform_angle_slices(num_slices):
  return [-pi + i*2*pi/num_slices for i in range(num_slices)]

def is_convex(verts): #Must be increasing except for one pair on the circular list
  angles = [(verts[i] - verts[i-1]).perpendicular().angle() for i in range(len(verts))]
  out_of_order = 0
  for i in range(len(angles)):
    if angles[i-1] > angles[i]:
      out_of_order += 1
      if out_of_order > 1:
        return False
  return True

def order_increasing(verts):
  angles = [(verts[i] - verts[i-1]).perpendicular().angle() for i in range(len(verts))]
  for i in range(len(angles)-1):
    if angles[i] > angles[i+1]:
      return verts[i+1:] + verts[:i+1]
  return verts

def merge(a, b):
  result = []
  p_a = 0
  p_b = 0
  for i in range(len(a) + len(b)):
    if p_b >= len(b) or (p_a < len(a) and a[p_a] <= b[p_b]):
      result.append(a[p_a])
      p_a += 1
    else:
      result.append(b[p_b])
      p_b += 1
  return result

class Point:
  def __init__(self, x, y):
    self.x = float(x)
    self.y = float(y)
  def __add__(self, other):
    return Point(self.x + other.x, self.y + other.y)
  def __sub__(self, other):
    return Point(self.x - other.x, self.y - other.y)
  def length(self):
    return sqrt(self.x*self.x + self.y*self.y)
  def angle(self):
    return atan2(self.y, self.x)
  def normalize(self):
    return (1.0/self.length())*self
  def distance(self, other):
    return (other-self).length()
  def dot(self, other):
    return self.x*other.x + self.y*other.y
  def cross(self, other): #Returns z component of cross (everything else is zero)
    return (a.x*b.y - a.y*b.x)
  def perpendicular(self):
    return Point(self.y, -self.x)
  def rotate(self, angle):
    current_angle = self.angle()
    return self.length()*Point(cos(current_angle + angle), sin(current_angle + angle))
  def rotate_about(self, point, angle):
    return (self - point).rotate(angle) + point
  def __rmul__(self, scale):
    return Point(scale*self.x, scale*self.y)
  def __neg__(self):
    return Point(-self.x, -self.y)
  def __eq__(self, other):
    return isinstance(other, Point) and self.x == other.x and self.y == other.y
  def __ne__(self, other):
    return not self == other
  def draw(self, window, color = 'black', radius = 3):
    window.drawPoint(self.x, self.y, color, radius = radius)
  def __repr__(self):
    return 'Point: (' + str(self.x) + ', ' + str(self.y) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__
  
class Pose:
  def __init__(self, position, orientation):
    self.position = position
    self.orientation = float(orientation)
  def __eq__(self, other):
    return isinstance(other, Pose) and self.position == other.position and self.orientation == other.orientation
  def __ne__(self, other):
    return not self == other
  def draw(self, window, color = 'black', radius = 3):
    self.position.draw(window, color=color, radius=radius)
  def __repr__(self):
    return 'Pose: (' + str(self.position) + ', ' + str(self.orientation) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__
  
class Line:
  def __init__(self, start, end):
    self.start = start
    self.end = end
    self.aabb = None
    self.leq = None
  def get_aabb(self):
    if self.aabb is None:
      min_x = min(self.start.x, self.end.x)
      min_y = min(self.start.y, self.end.y)
      max_x = max(self.start.x, self.end.x)
      max_y = max(self.start.y, self.end.y)
      self.aabb = AABB(Point(min_x, min_y), Point(max_x, max_y))
    return self.aabb
  def get_leq(self):
    if self.leq is None:
      n = self.unit_vector().perpendicular()
      d = -n.dot(self.start)
      self.leq = (n, d)
    return self.leq
  def length(self):
    return self.start.distance(self.end)
  def unit_vector(self):
      return (1.0/self.length())*(self.end-self.start)
  def left(self):
    if self.start.x <= self.end.x:
      return self.start
    else:
      return self.end
  def right(self):
    if self.start.x > self.end.x:
      return self.start
    else:
      return self.end
  def is_left_turn(self, point): #returns true if the point is left of the line
    (n, d) = self.get_leq()
    return n.dot(point) + d < 0
  def distance(self, pt):
    if self.start == self.end:
        return self.start.distance(pt)
    else:
        (n, d) = self.get_leq()
        return n.dot(pt) + d    
  def intersect_vertical(self, x): #intersection with the vertical line
    if x < self.left().x or x > self.right().x:
      return None
    (n, d) = self.get_leq()
    return Point(x, (-d - n.x*x)/n.y)
  def intersect(self, other, edges = False): #edges = True counts a point being on an edge as collision 
    if not self.get_aabb().collides(other.get_aabb()):
      return False
    d_1 = (self.end - self.start)
    perp_1 = -d_1.perpendicular()
    d_2 = other.end - other.start
    diff = (self.start - other.start)
    num = diff.dot(perp_1)
    denom = d_2.dot(perp_1)
    if denom == 0:
      return False
    b = (1.0*num)/denom
    intersect = other.start + b*d_2
    if d_1.x == 0:
      a = (intersect - self.start).y/d_1.y
    else:
      a = (intersect - self.start).x/d_1.x
    if edges:
      return a >= 0 and a <= 1 and b >= 0 and b <= 1
    else:   
      return a > 0 and a < 1 and b > 0 and b < 1
  def cut(self, other):
    if not self.get_aabb().collides(other.get_aabb()):
      return None
    d_1 = (self.end - self.start)
    perp_1 = -d_1.perpendicular()
    d_2 = other.end - other.start
    diff = (self.start - other.start)
    num = diff.dot(perp_1)
    denom = d_2.dot(perp_1)
    if denom == 0:
      return None
    b = (1.0*num)/denom
    intersect = other.start + b*d_2
    if d_1.x == 0:
      a = (intersect - self.start).y/d_1.y
    else:
      a = (intersect - self.start).x/d_1.x
    if a > 0 and a < 1 and b > 0 and b < 1 and not within((intersect - self.start).length()) and not within((intersect - self.end).length()): #Sometime the cut is just the line + point
      return [Line(self.start, intersect), Line(intersect, self.end)]
    else:
      return None
  def equal_leq(self, other):
    return self.get_leq() == other.get_leq()
  def midpoint(self):
    return (self.start + self.end)/2
  def draw(self, window, color="black", width = 3):
    window.drawLineSeg(self.start.x, self.start.y, self.end.x, self.end.y, color=color, width=width)
  def __eq__(self, other):
    return isinstance(other, Line) and self.start == other.start and self.end == other.end #directed line or not?
  def __ne__(self, other):
    return not self == other
  def __repr__(self):
    return 'Line: (' + str(self.start) + ', ' + str(self.end) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__
  
class AABB:
  def __init__(self, min, max):
    self.min = min
    self.max = max
    self.verts = None
  def get_verts(self):
    if self.verts is None:
      self.verts = [self.min, Point(self.max.x, self.min.y), self.max, Point(self.min.x, self.max.y)]
    return self.verts
  def get_lines(self):
    points = self.get_verts()
    return [Line(points[i-1], points[i]) for i in range(len(points))]
  def translate(self, reference):
    return AABB(self.min + point, self.max + point)
  def union(self, other):
    return AABB(Point(min(self.min.x, other.min.x), min(self.min.y, other.min.y)), Point(max(self.max.x, other.max.y), max(self.max.y, other.max.y)))
  def intersection(self, other):
    if self.collides(other):
        (xlo1, ylo1, zlo1), (xhi1, yhi1, zhi1) = bb1
        (xlo2, ylo2, zlo2), (xhi2, yhi2, zhi2) = bb2
        return AABB(Point(max(self.min.x, other.min.x), max(self.min.y, other.min.y)),
                    Point(min(self.max.x, other.max.x), min(self.max.y, other.max.y)))
    else:
        return None
  def volume(self):
    return (self.max.x - self.min.x)*(self.max.y - self.min.y)
  def collides(self, other, edges=False):
    return not (self.min.x > other.max.x or self.max.x < other.min.x or \
                self.min.y > other.max.y or self.max.y < other.min.y)    
  def contains(self, other, edges = False): #edges = True counts a point being on an edge as containment
    if isinstance(other, Point):
      if edges:
        return self.min.x <= other.x and self.min.y <= other.y and self.max.x >= other.x and self.max.y >= other.y
      else:
        return self.min.x < other.x and self.min.y < other.y and self.max.x > other.x and self.max.y > other.y
    elif isinstance(other, Line): #Definition of convex object
      return self.contains(other.start, edges=edges) and self.contains(other.end, edges=edges) 
    elif isinstance(other, AABB):
      if edges:
        return self.min.x <= other.min.x and self.min.y <= other.min.y and self.max.x >= other.max.x and self.max.y >= other.max.y
      else:
        return self.min.x < other.min.x and self.min.y < other.min.y and self.max.x > other.max.x and self.max.y > other.max.y  
    elif isinstance(other, Polygon):
      return all([self.contains(vert, edges) for vert in other.points])   
    elif isinstance(other, Object):
      return all([self.contains(poly, edges) for poly in other.polys]) 
    else:
      raise Exception
  def draw(self, window, color="black", width = 3):
    verts = self.get_verts()
    for i in range(len(verts)):
      Line(verts[i-1], verts[i]).draw(window, color=color, width=width)
  def __repr__(self):
    return 'AABB: (' + str(self.min) + ', ' + str(self.max) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__
    
class Polygon:
  def __init__(self, points):
    assert is_convex(points)
    self.points = order_increasing(points)
    self.leqs = None
    self.aabb = None
    self.radius = None
  def get_aabb(self):
    if self.aabb is None:
      min_x = min([point.x for point in self.points])
      min_y = min([point.y for point in self.points])
      max_x = max([point.x for point in self.points])
      max_y = max([point.y for point in self.points])
      self.aabb = AABB(Point(min_x, min_y), Point(max_x, max_y))
    return self.aabb
  def get_leqs(self): #In normal form
    if self.leqs is None:
      self.leqs = []
      for i in range(len(self.points)):
        diff = self.points[i] - self.points[i-1]
        n = diff.perpendicular()
        d = -n.dot(self.points[i])
        self.leqs.append((n, d))
    return self.leqs
  def get_lines(self):
    return [Line(self.points[i-1], self.points[i]) for i in range(len(self.points))]
  def get_radius(self):
    if self.radius is None:
      self.radius = max([point.length() for point in self.points])
    return self.radius
  def cspace_poly(self, other):
    poly = other.invert()
    my_angles = [vector[0].angle() for vector in self.get_leqs()]
    my_edges = [(my_angles[i], self.points[i-1], self.points[i]) for i in range(len(self.points))]
    
    poly_angles = [vector[0].angle() for vector in poly.get_leqs()]
    poly_edges = [(poly_angles[i], poly.points[i-1], poly.points[i]) for i in range(len(poly.points))]
    
    start_point = my_edges[0][1] + poly_edges[0][1]
    new_edges = merge(my_edges, poly_edges)
    
    verts = [start_point]
    for i in range(len(new_edges)-1):
      (angle, p1, p2) = new_edges[i]
      verts.append(verts[-1] + (p2 - p1))
                
    return Polygon(verts)       
  def invert(self):
    return Polygon([-1*point for point in self.points])
  def translate(self, reference):
    return Polygon([reference + point for point in self.points])
  def rotate(self, angle): #counter-clockwise
    return Polygon([point.rotate(angle) for point in self.points])
  def at_pose(self, pose):
    return self.rotate(pose.orientation).translate(pose.position)
  def collides(self, other, edges = False):  #edges = True counts a point being on an edge as collision
    if not self.get_aabb().collides(other.get_aabb()):
      return False
    if isinstance(other, Polygon):
      if self.contains(other.points[0]) or other.contains(self.points[0]):
        return True
      for i in range(len(self.points)):
        for j in range(len(other.points)):
          if Line(self.points[i-1], self.points[i]).intersect(Line(other.points[j-1], other.points[j]), edges=edges):
            return True
      return False
    elif isinstance(other, Line):
      if self.contains(other.midpoint()) or self.contains(other.start) or self.contains(other.end):
        return True
      for i in range(len(self.points)):
        if Line(self.points[i-1], self.points[i]).intersect(other, edges=edges):
          return True
      return False
    else:
      raise Exception
  def contains(self, other, edges = False): #edges = True counts a point being on an edge as containment
    if isinstance(other, Point):
      if edges:
        return self.get_aabb().contains(other) and all([line[0].dot(other) + line[1] <= 0 for line in self.get_leqs()])
      else:
        return self.get_aabb().contains(other) and all([line[0].dot(other) + line[1] < 0 for line in self.get_leqs()])
    elif isinstance(other, Polygon):
      return self.get_aabb().contains(other.get_aabb()) and all([self.contains(vert, edges) for vert in other.points])
    elif isinstance(other, Line): #Definition of convex object
      return self.contains(line.start, edges=edges) and self.contains(line.end, edges=edges)  
    else:
      raise Exception
  def sweep_move(self, direction):
    return [Polygon([self.points[i-1], self.points[i], self.points[i] + direction, self.points[i-1] + direction]) for i in range(len(self.points))]
  def draw(self, window, color = 'blue'):
    window.drawPolygon(self.points, color = color)
  def __eq__(self, other):
    return isinstance(other, Polygon) and len(self.points) == len(other.points) and all(self.points[i] == other.points[i] for i in range(len(self.points)))
  def __ne__(self, other):
    return not self == other
  def __repr__(self):
    return 'Polygon: (' + str(self.points) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__
  
class Object:
  def __init__(self, reference, polys):
    self.reference = reference
    self.local_polys = polys
    self.polys = [poly.translate(reference) for poly in polys]
    self.aabb = None
    self.radius = None
  def get_aabb(self):
    if self.aabb is None:
      min_x = min([poly.get_aabb().min.x for poly in self.polys])
      min_y = min([poly.get_aabb().min.y for poly in self.polys])
      max_x = max([poly.get_aabb().max.x for poly in self.polys])
      max_y = max([poly.get_aabb().max.y for poly in self.polys])
      self.aabb = AABB(Point(min_x, min_y), Point(max_x, max_y))
    return self.aabb
  def get_lines(self):
    lines = []
    for poly in self.polys:
      lines += poly.get_lines()
    return lines
  def get_radius(self):
    if self.radius is None:
      self.radius = max([poly.get_radius() for poly in self.local_polys])
    return self.radius
  def collides(self, other, edges = False): #edges = True counts a point being on an edge as collision
    return self.get_aabb().collides(other.get_aabb(), edges) and any([my_poly.collides(other_poly, edges) for my_poly in self.polys for other_poly in other.polys])
  def contains(self, point, edges = False): #edges = True counts a point being on an edge as collision
    return self.get_aabb().contains(point, edges) and any([poly.contains(point, edges) for poly in self.polys])
  def rotate(self, angle): #About the reference
    return Object(self.reference, [poly.rotate(angle) for poly in self.local_polys])
  def translate(self, reference):
    return Object(reference, self.local_polys)
  def at_pose(self, pose):
    return self.translate(pose.position).rotate(pose.orientation)
  def sweep_move(self, start, finish):
    return Object(start, [swept for poly in self.local_polys for swept in poly.sweep_move(finish - start)])
  def cspace_object(self, other):
    new_polys = []
    for my_poly in self.local_polys:
      for other_poly in other.local_polys:
        new_polys.append(my_poly.cspace_poly(other_poly))        
    return Object(self.reference, new_polys)
  def draw(self, window, color = 'blue'):
    for poly in self.polys:
      poly.draw(window, color)
    #self.get_aabb().draw(window, color=color, width = 3)
    self.reference.draw(window, color='black', radius=3)
  def __repr__(self):
    return 'Object: (' + str(self.reference) + ", " + str(self.polys) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__

class Robot:
  def __init__(self, polys):
    self.polys = polys
    self.radius = None
  def configuration(self, configuration): #[x, y, theta]
    assert len(configuration) == 3
    return Object(Point(configuration[0], configuration[1]), [poly.rotate(configuration[2]) for poly in self.polys])
  def get_radius(self):
    if self.radius is None:
      self.radius = max([poly.get_radius() for poly in self.polys])
    return self.radius
  def distance(self, configuration):
    return sqrt(configuration[0]*configuration[0] + configuration[1]*configuration[1]) + self.get_radius()*abs(configuration[2])
  def __repr__(self):
    return 'Robot: (' + str(self.polys) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__

class RobotArm:
  def __init__(self, reference, joints):
    self.reference = reference
    self.joints = joints
  def configuration(self, configuration): #[theta_1, theta_2, ..., theta_n]
    assert len(configuration) == len(self.joints)
    polys = []
    origin = None
    angle = None
    for i in range(len(configuration)):
      (joint, link) = self.joints[i]
      if origin == None:
        angle = configuration[i]
        origin = joint.rotate(angle)
      else:
        origin += joint.rotate(angle)
        angle += configuration[i]
      polys.append(link.at_pose(Pose(origin, angle)))
    return Object(self.reference, polys) 
  def distance(self, configuration):
    assert len(configuration) == len(self.joints)
    return max([abs(configuration[i])*self.joints[i][1].get_radius() for i in range(len(configuration))])
  def distance_2(self, configuration):
    assert len(configuration) == len(self.joints)
    return sum([abs(configuration[i])*self.joints[i][1].get_radius() for i in range(len(configuration))])
  def distance_3(self, configuration):
    assert len(configuration) == len(self.joints)
    max_distance = None
    origin = None
    angle = None
    for i in range(len(configuration)):
      (joint, link) = self.joints[i]
      if origin == None:
        angle = configuration[i]
        origin = joint.rotate(angle)
      else:
        origin += joint.rotate(angle)
        angle += configuration[i]
      max_distance = max(max_distance, origin.length() + link.get_radius()*abs(angle)) if max_distance != None else origin.length() + link.get_radius()*abs(angle)
    return max_distance
  def __repr__(self):
    return 'RobotArm: (' + self.reference + ', ' + str(self.joints) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__

def convex_hull(verts):
  points = [(v.x, v.y) for v in verts]
  return [Point(v[0], v[1]) for v in convexHull(points)[::-1]]

######################################################################
# Convex hull code from http://python.net/~gherman/convexhull.html
######################################################################

def _myDet(p, q, r):
  # We use Sarrus' Rule to calculate the determinant.
  # (could also use the Numeric package...)
  sum1 = q[0]*r[1] + p[0]*q[1] + r[0]*p[1]
  sum2 = q[0]*p[1] + r[0]*q[1] + p[0]*r[1]

  return sum1 - sum2


def _isRightTurn((p, q, r)):
  assert p != q and q != r and p != r
          
  if _myDet(p, q, r) < 0:
    return 1
  else:
    return 0

def convexHull(P):
  # Get a local list copy of the points and sort them lexically.
  points = map(None, P)
  points.sort()

  # Build upper half of the hull.
  upper = [points[0], points[1]]
  for p in points[2:]:
    upper.append(p)
    while len(upper) > 2 and not _isRightTurn(upper[-3:]):
      del upper[-2]

  # Build lower half of the hull.
  points.reverse()
  lower = [points[0], points[1]]
  for p in points[2:]:
    lower.append(p)
    while len(lower) > 2 and not _isRightTurn(lower[-3:]):
      del lower[-2]

  # Remove duplicates.
  del lower[0]
  del lower[-1]

  # Concatenate both halfs and return.
  return tuple(upper + lower)