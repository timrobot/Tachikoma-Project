'''
File testing the capabilities of the PRQuadTree.

Author: Pawel Szczurko
'''

from prquadtree import *

# creates two squares
b = Box(Point(5,5), 50)
b2 = Box(Point(50,50), 50)

print "Do 'b' and 'b2' intersect: %s " % b.intersect(b2)

# create PRQuadTree based on b2
qt = PRQuadTree(b2)

qt.insert(Point(1,1))
qt.insert(Point(4,14))
qt.insert(Point(14,14))
qt.insert(Point(16,4))
qt.insert(Point(1,2))
qt.insert(Point(2,3))
qt.insert(Point(5,3))

# insert more random data
for x in range(100):
    qt.insert(Point(uniform(0.0,100.0), uniform(0.0,100.0)))

print "======Start print of all points in PRQuadTree======"
print qt.print_all_points(qt)
print "======End print of all points in PRQuadTree======"

# defines seach point
pt = Point(2,2)
# get 20 nearby points to search point
nearby = qt.query_k_nearest(pt, 20)

print "======View nearby points for %s =======" % pt
c = 1
for point in nearby:
    print "%s: %s" % (c, point)
    c+=1

print "Num points: %s" % PRQuadTree.size(qt)
