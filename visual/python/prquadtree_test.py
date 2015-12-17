import unittest
from prquadtree import *

class TestPoint(unittest.TestCase):
    def test_point_insert(self):
        x = 5
        y = 6
        p = Point(x,y)
        self.assertEqual(p.x, x)
        self.assertEqual(p.y, y)

class TestParticle(unittest.TestCase):
    def test_particle_insert(self):
        x = 5
        y = 6
        part = Particle(x,y)
        self.assertEqual(part.x, x)
        self.assertEqual(part.y, y)
        self.assertEqual(part.score, 0)

class TestBox(unittest.TestCase):
    def test_box_insert(self):
        x = 5
        y = 6
        p = Point(x,y)
        box = Box(p, 50)
        self.assertEqual(box.half_size, 50)
    def test_box_contains(self):
        x = 5
        y = 6
        p = Point(x,y)
        box = Box(p, 50)
        p2 = Point(x + 1, y + 1)
        self.assertTrue(box.contains_point(p2))

class TestPrQuadTree(unittest.TestCase):
    def test_insert(self):
        b2 = Box(Point(50,50), 50)
        qt = PRQuadTree(b2)
        qt.insert(Point(1,1))
        self.assertEqual(qt.size(qt), 1)
    def test_nearby(self):
        b2 = Box(Point(50,50), 50)
        qt = PRQuadTree(b2)

        for x in range(100):
            qt.insert(Point(uniform(0.0,100.0), uniform(0.0,100.0)))
        pt = Point(2,2) 
        nearby = qt.query_k_nearest(pt, 20)
        self.assertEqual(len(nearby), 20)

if __name__ == "__main__":
    unittest.main()
