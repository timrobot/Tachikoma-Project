import pygame, math, sys, lidarsim, numpy

lidar = lidarsim.Lidar()

pygame.init()
screen = pygame.display.set_mode((640, 640))
clock = pygame.time.Clock()

def cos_rule_angle(A, B, C):
  assert (A + B) >= C
  return math.acos((A * A + B * B - C * C) / (2.0 * A * B))

def cart(r, t):
  return [r * math.cos(t), r * math.sin(t)]

def concave(l, c, r):
  pl = cart(l[0], l[1])
  pr = cart(r[0], r[1])
  pc = cart(c[0], c[1])
  lc = math.sqrt((pl[0] - pc[0]) ** 2.0 + (pl[1] - pc[1]) ** 2.0)
  rc = math.sqrt((pr[0] - pc[0]) ** 2.0 + (pr[1] - pc[1]) ** 2.0)
  theta = cos_rule_angle(lc, c[0], l[0])
  phi = cos_rule_angle(rc, c[0], r[0])
  return (theta + phi) > math.pi

def get_unit(vec):
  SUM = sum(map(lambda s: s * s, vec))
  return map(lambda s: s / SUM, vec)

def within(l, c, r):
  if c[0] == 0.0 and c[1] == 0.0: # branch
    return True
  return concave(l, c, r)

def within2(l, c, r):
  if c[0] == 0.0 and c[1] == 0.0: # branch
    return True
  l = cart(l[0], l[1]) # 2 trig, 2 mult
  c = cart(c[0], c[1]) # 2 trig, 2 mult
  r = cart(r[0], r[1]) # 2 trig, 2 mult
  assert l[0] != r[0] or l[1] != r[1]
  if (l[0] == c[0] and l[1] == c[1]) or (r[0] == c[0] and r[1] == c[1]): # branch
    return True
  lr = [r[0] - l[0], r[1] - l[1]]
  lc = [c[0] - l[0], c[1] - l[1]]
  rc = [c[0] - r[0], c[1] - r[1]]
  # on the line
  ulr = get_unit(lr) # 1 sqrt, 2 mult, 2 div
  ulc = get_unit(lc) # 1 sqrt, 2 mult, 2 div
  urc = get_unit(rc) # 1 sqrt, 2 mult, 2 div
  if ulr[0] == ulc[0] and ulr[1] == ulr[1] and -ulr[0] == urc[0] and -ulr[1] == urc[1]: # branch
    return True
  # O = c + lc(t1) + rc(t2)
  # Ox = lx + lcxt1 + rcxt2
  # Oy = ly + lcyt1 + rcyt2
  T = numpy.mat([[lc[0], rc[0]], [lc[1], rc[1]]]).I * numpy.mat([[-l[0]], [-l[1]]]) # inv, 2 mult
  return T[0] > 0.0 and T[1] > 0.0 # make this more efficient (under or overline)

def within3(l, c, r):
  if c[0] == 0.0 and c[1] == 0.0: # branch
    return True
  l = cart(l[0], l[1]) # 2 trig, 2 mult
  c = cart(c[0], c[1]) # 2 trig, 2 mult
  r = cart(r[0], r[1]) # 2 trig, 2 mult
  assert l[0] != r[0] or l[1] != r[1]
  if (l[0] == c[0] and l[1] == c[1]) or (r[0] == c[0] and r[1] == c[1]): # branch
    return True
  lr = [r[0] - l[0], r[1] - l[1]]
  lc = [c[0] - l[0], c[1] - l[1]]
  rc = [c[0] - r[0], c[1] - r[1]]
  # angle detect
  theta_lr = math.atan2(lr[1], lr[0]) # trig
  theta_lc = math.atan2(lc[1], lc[0]) # trig
  dtheta = theta_lr - theta_lc
#  if delta > math.pi: # branch
#    delta -= math.pi * 2.0
#  elif delta < -math.pi: # branch
#    delta += math.pi * 2.0
  return dtheta < 0.0

pi2 = 2.0 * math.pi
def within4(l, c, r):
  if c[0] == 0.0 and c[1] == 0.0: # branch
    return True
  global pi2
  dlrx = r[0] - l[0]
  dlry = r[1] - l[1]
  tlr = math.atan2(dlry, dlrx)
  dlcx = c[0] - l[0]
  dlcy = c[1] - l[1]
  tlc = math.atan2(dlcy, dlcx)
  dt = tlr - tlc
  #if abs(dt) - math.pi > 0.000001:
  if dt >= math.pi - 0.000001:
    dt -= pi2
  elif dt < -math.pi - 0.000001:
    dt += pi2
  return dt > 0.0000001

def positive(num):
  return num >= 0.0

def get_convex_perimeter(points):
  pts = [] + points
  # determine the concavity
  left = pts[len(pts)-1]
  curr = pts[0]
  right = pts[1]
  delta = 0.0
  index = 0
  concavity = False
  while positive(curr[1] - delta) or concavity:
    if not positive(curr[1] - delta):
      concavity = False
      delta = 0.0
      index = 0
    while concave(left, curr, right):
      # concave bitch!
      concavity = True
      del pts[index]
      index -= 1
      if index < 0:
        index += 1
        left = pts[len(pts)-1]
        curr = pts[0]
        right = pts[1]
      else:
        curr = pts[index % len(pts)]
        left = pts[(index - 1) % len(pts)]
    index += 1
    delta = curr[1]
    curr = pts[index % len(pts)]
    left = pts[(index - 1) % len(pts)]
    right = pts[(index + 1) % len(pts)]
  return pts

# will return the index of the list such that L[index] <

def cvt_cartesian(pts):
  carts = []
  for pt in pts:
    carts.append([pt[0] * math.cos(pt[1]), pt[0] * math.sin(pt[1])])
  return carts

def get_filled_boundary(pts):
  # get min and max of x and y
  X = map(lambda pt: pt[0] * math.cos(pt[1]), pts)
  Y = map(lambda pt: pt[0] * math.sin(pt[1]), pts)
  min_x = int(round(min(X)))
  min_y = int(round(min(Y)))
  max_x = int(round(max(X))) + 1
  max_y = int(round(max(Y))) + 1
  # for all x, y {e} [min_x,max_x], [min_y,max_y] # replace with infection algorithm
  #   if within (search_left(x, y)), (search_right(x, y)) // change to binary search
  colored = []
  for x in range(min_x, max_x):
    for y in range(min_y, max_y):
      theta = math.atan2(y, x)
      if theta < 0.0:
        theta += 2.0 * math.pi
      radius = (x ** 2.0 + y ** 2.0) ** 0.5
      left = -1
      right = -1
      for i in range(len(pts)):
        left = (i - 1) % len(pts)
        right = i
        if (i == 0 and (theta >= pts[left][1] or theta < pts[right][1])) or \
          (i != 0 and pts[left][1] <= theta and theta < pts[right][1]):
          break
      if within(pts[left], [radius, theta], pts[right]):
        colored.append([x, y])
  return colored

def get_filled_boundary2(pts):
  # get min and max of x and y
  X = map(lambda pt: pt[0] * math.cos(pt[1]), pts)
  Y = map(lambda pt: pt[0] * math.sin(pt[1]), pts)
  min_x = int(round(min(X)))
  min_y = int(round(min(Y)))
  max_x = int(round(max(X))) + 1
  max_y = int(round(max(Y))) + 1
  vmap = map(lambda s: map(lambda k: False, range(min_x, max_x)), range(min_y, max_y))
  colored = []
  infection_queue = [(0, 0)]
  vmap[0][0] = True
  #print within2(pts[len(pts)-1],(0.0, 0.0),pts[0])
  while len(infection_queue) > 0:
  #for x in range(min_x, max_x):
  #  for y in range(min_y, max_y):
      x = infection_queue[0][0]
      y = infection_queue[0][1]

      theta = math.atan2(y, x)
      if theta < 0.0:
        theta += 2.0 * math.pi
      radius = (x ** 2.0 + y ** 2.0) ** 0.5
      # binary search
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
      if within(pts[left], [radius, theta], pts[right]):
      #if min_x <= x and x < max_x + 1 and min_y <= y and y < max_y + 1:
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
#    raycast = get_convex_perimeter(data)
    print "getting raycast area..."
    raycast_area = get_filled_boundary2(data)
    #print colored

    # draw stuff
    screen.fill((0, 0, 0))
    for pt in raycast_area:
      screen.set_at((pt[0] + center[0], pt[1] + center[1]), (128, 0, 0))
#    box = pygame.Rect(colored[0] + center[0], colored[2] + center[1], colored[1] - colored[0], colored[3] - colored[2])
#    print colored
#    pygame.draw.rect(screen, (255, 255, 255), box, 1)
    for i in range(len(data)):
      pt = data[i]
      x = pt[0] * math.cos(pt[1]) + center[0]
      y = pt[0] * math.sin(pt[1]) + center[1]
      newpt = (int(round(x)), int(round(y)))
      screen.set_at(newpt, (0, 255, 0))
      #pygame.draw.line(screen, (64, 64, 64), center, newpt)
#      pt2 = data[(i + 1) % len(data)]
#      x2 = pt2[0] * math.cos(pt2[1]) + center[0]
#      y2 = pt2[0] * math.sin(pt2[1]) + center[1]
#      newpt2 = (int(round(x2)), int(round(y2)))
#      pygame.draw.line(screen, (255, 64, 0), newpt, newpt2)
    screen.set_at((299, 300), (0, 0, 255))
    screen.set_at((300, 300), (0, 0, 255))
    screen.set_at((300, 301), (0, 0, 255))
    screen.set_at((301, 300), (0, 0, 255))
    screen.set_at((300, 299), (0, 0, 255))
#    for i in range(len(raycast)):
#      x = raycast[i][0] * math.cos(raycast[i][1]) + center[0]
#      y = raycast[i][0] * math.sin(raycast[i][1]) + center[1]
#      newpt1 = (int(round(x)), int(round(y)))
#      j = (i + 1) % len(raycast)
#      x = raycast[j][0] * math.cos(raycast[j][1]) + center[0]
#      y = raycast[j][0] * math.sin(raycast[j][1]) + center[1]
#      newpt2 = (int(round(x)), int(round(y)))
#      pygame.draw.line(screen, (255, 0, 0), newpt1, newpt2)
#      screen.set_at(newpt1, (255, 255, 255))
#      screen.set_at((newpt1[0]+1,newpt1[1]), (255, 255, 255))
#      screen.set_at((newpt1[0]-1,newpt1[1]), (255, 255, 255))
#      screen.set_at((newpt1[0],newpt1[1]+1), (255, 255, 255))
#      screen.set_at((newpt1[0],newpt1[1]-1), (255, 255, 255))
      
    pygame.display.flip()
    clock.tick(1)

main()
