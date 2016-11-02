from math import sqrt, cos, sin, atan2, pi

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
  def dot(self, other):
    return self.x*other.x + self.y*other.y
  def perpendicular(self):
    return Point(self.y, -self.x)
  def rotate(self, angle):
    current_angle = self.angle()
    return self.length()*Point(cos(current_angle + angle), sin(current_angle + angle))
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
  def get_aabb(self):
    if self.aabb is None:
      min_x = min(self.start.x, self.end.x)
      min_y = min(self.start.y, self.end.y)
      max_x = max(self.start.x, self.end.x)
      max_y = max(self.start.y, self.end.y)
      self.aabb = AABB(Point(min_x, min_y), Point(max_x, max_y))
    return self.aabb
    
  def ccw(self, A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

 # Return true if line segments AB and CD intersect
  def intersect2(self, A,B,C,D):
    return self.ccw(A,C,D) != self.ccw(B,C,D) and self.ccw(A,B,C) != self.ccw(A,B,D)
    
    
  def intersect(self, other, edges = False): #edges = True counts a point being on an edge as collision
    #return self.intersect2(self.start, self.end, other.start, other.end)
  
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
  def midpoint(self):
    return Point((self.start.x + self.end.x)/2, (self.start.y + self.end.y)/2)
  def draw(self, window, color="black", width = 3):
    window.drawLineSeg(self.start.x, self.start.y, self.end.x, self.end.y, color=color, width=width)
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
  def translate(self, reference):
    return AABB(self.min + point, self.max + point)
  def union(self, other):
    return AABB(Point(min(self.min.x, other.min.x), min(self.min.y, other.min.y)), Point(max(self.max.x, other.max.y), max(self.max.y, other.max.y)))
  def collides(self, other, edges=False):
    return not (self.min.x > other.max.x or self.max.x < other.min.x or \
                self.min.y > other.max.y or self.max.y < other.min.y)    
  def contains(self, other, edges = False): #edges = True counts a point being on an edge as containment
    if isinstance(other, Point):
      if edges:
        return self.min.x <= other.x and self.min.y <= other.y and self.max.x >= other.x and self.max.y >= other.y
      else:
        return self.min.x < other.x and self.min.y < other.y and self.max.x > other.x and self.max.y > other.y
    elif isinstance(other, Polygon):
      return all([self.contains(vert, edges) for vert in other.points])
    elif isinstance(other, AABB):
      if edges:
        return self.min.x <= other.min.x and self.min.y <= other.min.y and self.max.x >= other.max.x and self.max.y >= other.max.y
      else:
        return self.min.x < other.min.x and self.min.y < other.min.y and self.max.x > other.max.x and self.max.y > other.max.y  
    elif isinstance(other, Line): #Definition of convex object
      return self.contains(other.start, edges=edges) and self.contains(other.end, edges=edges)    
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
    self.points = points
    self.lines = None
    self.aabb = None
  def get_aabb(self):
    if self.aabb is None:
      min_x = min([point.x for point in self.points])
      min_y = min([point.y for point in self.points])
      max_x = max([point.x for point in self.points])
      max_y = max([point.y for point in self.points])
      self.aabb = AABB(Point(min_x, min_y), Point(max_x, max_y))
    return self.aabb
  def get_lines(self): #In normal form
    if self.lines is None:
      self.lines = []
      for i in range(len(self.points)):
        diff = self.points[i] - self.points[i-1]
        n = diff.perpendicular()
        d = -n.dot(self.points[i])
        self.lines.append((n, d))
    return self.lines
  def cspace_poly(self, other):
    poly = other.invert()
    my_angles = [vector[0].angle() for vector in self.get_lines()]
    my_edges = [(my_angles[i], self.points[i-1], self.points[i]) for i in range(len(self.points))]
    my_edges.sort() #Should just wrap back of list to the front
    
    poly_angles = [vector[0].angle() for vector in poly.get_lines()]
    poly_edges = [(poly_angles[i], poly.points[i-1], poly.points[i]) for i in range(len(poly.points))]
    poly_edges.sort() #Should just wrap back of list to the front
    
    start_point = my_edges[0][1] + poly_edges[0][1]
    
    new_edges = my_edges + poly_edges
    new_edges.sort() #Could just merge
    
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
        return self.get_aabb().contains(other) and all([line[0].dot(other) + line[1] <= 0 for line in self.get_lines()])
      else:
        return self.get_aabb().contains(other) and all([line[0].dot(other) + line[1] < 0 for line in self.get_lines()])
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
  def get_radius(self):
    if self.radius is None:
      distances = []
      for poly in self.local_polys:
        distances.extend([point.length() for point in poly.points])
      self.radius = max(distances)
    return self.radius
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
    return 'Object: (' + str(self.reference) + str(self.polys) + ')'
  def __hash__(self):
    return str(self).__hash__()
  __str__ = __repr__