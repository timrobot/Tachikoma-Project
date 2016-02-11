from cv2 import *

def main():
  v = VideoWriter("stream.avi", -1, 20, (640, 480))
  if not v.isOpened():
    print "Cannot open"

main()
