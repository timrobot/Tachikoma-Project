from prquadtree import *

class ParticleFilter:
    ##
    # Constructor
    # @param box Box the box representing the web cam view
    #
    def __init__(self, box):
      self.pr_tree = PRQuadTree(box)
      self.image_box = box
      self.iterations = 0
      '''
      particle filter will clear itself every interval of this fixed number
      of iterations
      '''
      self.iterations_before_clearing = 100

      #create uniform sampling
      subdivisions = 25 #number of subdivisions of grid (one side)
      cell_size = box.half_size * 2.0 / subdivisions
      for x in range(subdivisions):
          for y in range(subdivisions):
              self.pr_tree.insert(Particle(x * cell_size, y * cell_size))
    ##
    # For each blob, it updates the points in the tree increasing the score of those
    # which are within the bounding square of the blob
    #
    # @param blobs array An array of blob objects which were matched
    #
    def iterate(self, blobs):
        self.iterations += 1
        if self.iterations > self.iterations_before_clearing:
            self.clear_scores()

        for blob in blobs:
            #query with the bounding box
            half_size = max(blob.minRectWidth(), blob.minRectHeight())/2
            box = Box(Point(blob.minRectX(), blob.minRectY()), half_size)
            points = self.pr_tree.query_range(box)
            for p in points:
                p.score += 1
    ##
    # Returns the sum of the scores of the points found within this blob by querying
    # the quadtree
    # 
    # @param blob Blob A single blob
    # @return int The sum of the scores of the points contained in the passed blob
    #
    def score(self, blob):
        half_size = max(blob.minRectWidth(), blob.minRectHeight())/2
        box = Box(Point(blob.minRectX(), blob.minRectY()), half_size)
        points = self.pr_tree.query_range(box)
        point_sum = 0
        for p in points:
            point_sum += p.score
        return point_sum
    ##
    # Resets all scores of blobs
    # This should be used when changing the webcam view
    #
    def clear_scores(self):
        points = self.pr_tree.query_range(self.image_box)
        for p in points:
            p.score = 0
        self.iterations = 0
