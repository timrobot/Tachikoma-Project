import pygame, math, sys, lidarsim, numpy

lidar = lidarsim.Lidar()

pygame.init()
screen = pygame.display.set_mode((640, 640))
clock = pygame.time.Clock()

def cos_rule_angle(A, B, C):
  if abs((A + B) - C) < 0.000001:
    return math.pi
  assert (A + B) >= C
  return math.acos((A * A + B * B - C * C) / (2.0 * A * B))

def concave(l, c, r, lrad, crad, rrad):
  dlcx = c[0] - l[0]
  dlcy = c[1] - l[1]
  drcx = c[0] - r[0]
  drcy = c[1] - r[1]
  lc = math.sqrt(dlcx * dlcx + dlcy * dlcy)
  rc = math.sqrt(drcx * drcx + drcy * drcy)
  theta = cos_rule_angle(lc, crad, lrad)
  phi = cos_rule_angle(rc, crad, rrad)
  return (theta + phi) - math.pi > -0.01

def within(l, c, r, lrad, crad, rrad):
  if crad == 0.0:
    return True
  return concave(l, c, r, lrad, crad, rrad)

def get_filled_boundary(pts):
  # get min and max of x and y
  X = map(lambda pt: pt[0] * math.cos(pt[1]), pts)
  Y = map(lambda pt: pt[0] * math.sin(pt[1]), pts)
  min_x = 0
  min_y = 0
  max_x = 0
  max_y = 0
  for i in range(len(pts)):
    tx = int(math.floor(X[i]))
    if tx < min_x:
      min_x = tx
    tx = int(math.ceil(X[i]))
    if tx > max_x:
      max_x = tx
    ty = int(math.floor(Y[i]))
    if ty < min_y:
      min_y = ty
    ty = int(math.ceil(Y[i]))
    if ty > max_y:
      max_y = ty
  max_x += 1
  max_y += 1
  # create vmap for bfs visited
  vmap = map(lambda s: map(lambda k: False, range(min_x, max_x)), range(min_y, max_y))
  colored = []
  infection_queue = [(0, 0)]
  vmap[-min_y][-min_x] = True
  while len(infection_queue) > 0:
    x = infection_queue[0][0]
    y = infection_queue[0][1]

    theta = math.atan2(y, x)
    if theta < 0.0:
      theta += 2.0 * math.pi
    radius = math.sqrt(x * x + y * y)
    # binary search for angle boundary
    left = 0
    right = len(pts) - 1
    if pts[right][1] <= theta or theta < pts[left][1]:
      left = right
      right = 0
    else:
      mid = (left + right) / 2
      while (left + 1) != right:
        if theta >= pts[mid][1]:
          left = mid
        else:
          right = mid
        mid = (left + right) / 2
    # bfs with hashmap
    if within((X[left], Y[left]), (float(x), float(y)), (X[right], Y[right]), \
        pts[left][0], radius, pts[right][0]):
      colored.append([x, y])
      if y+1 < max_y and vmap[y - min_y + 1][x - min_x] == False:
        infection_queue.append((x, y+1))
        vmap[y - min_y + 1][x - min_x] = True
      if y-1 >= min_y and vmap[y - min_y - 1][x - min_x] == False:
        infection_queue.append((x, y-1))
        vmap[y - min_y - 1][x - min_x] = True
      if x+1 < max_x and vmap[y - min_y][x - min_x + 1] == False:
        infection_queue.append((x+1, y))
        vmap[y - min_y][x - min_x + 1] = True
      if x-1 >= min_x and vmap[y - min_y][x - min_x - 1] == False:
        infection_queue.append((x-1, y))
        vmap[y - min_y][x - min_x - 1] = True

    del infection_queue[0]
  return colored

def main():
  center = (300, 300)
  
  while True:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()

    data = lidar.read()
    print "getting raycast area..."
    raycast_area = get_filled_boundary(data)

    # draw stuff
    screen.fill((0, 0, 0))
    for pt in raycast_area:
      screen.set_at((pt[0] + center[0], -pt[1] + center[1]), (128, 0, 0))
    for i in range(len(data)):
      pt = data[i]
      x = pt[0] * math.cos(pt[1]) + center[0]
      y = -pt[0] * math.sin(pt[1]) + center[1]
      newpt = (int(round(x)), int(round(y)))
      screen.set_at(newpt, (0, 255, 0))
    screen.set_at((299, 300), (0, 0, 255))
    screen.set_at((300, 300), (0, 0, 255))
    screen.set_at((300, 301), (0, 0, 255))
    screen.set_at((301, 300), (0, 0, 255))
    screen.set_at((300, 299), (0, 0, 255))
      
    pygame.display.flip()
    clock.tick(40)

main()
