import random, math

class Lidar():
  def __init__(self):
    pass

  def read(self):
    points = [] # tuples of (radius, degree)
    theta = 0.0
    while theta < 360.0:
      theta += random.random() + 2.0
      if theta > 360.0:
        break
      radius = random.random() * 100.0 + 100.0
      points.append((radius, theta * math.pi / 180.0))
    return points
