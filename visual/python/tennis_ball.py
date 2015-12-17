'''
Simple detection of ball using SimpleCV (much easier than OpenCV). The run method
identifies a tennis ball in the camera stream image. 'is_ball_middle' function
can be used to determine whether a ball is horizontally centered based on a specified
threshold.

-Pawel Szczurko
'''

from SimpleCV import *
from time import sleep
from particlefilter import ParticleFilter
from image_support import *
import sys

# global particle filter
# error point elimination
particle_filter = None

## Internal wrapper to particle filter initializer.
#
# @param img SimpleCV.Image
def _init_particle_filter(img):
    global particle_filter
    if not particle_filter:
        particle_filter = external_init_particle_filter(img)

## Internal wrapper image hue filter. 
#
# @param img SimpleCV.Image 
# @return img SimpleCV.Image converted to HSV
def _ball_image_hue_filter(img):
    color = (185, 206, 111)
    return image_hue_filter(img, True)

## Entry point for module which determines whether tennis ball is in the middle of the image.
#
# @param img SimpleCV.Image 
# @return boolean. True if tennis ball is in middle, false otherwise.
def is_ball_middle(img):
    global particle_filter

    _init_particle_filter(img)
    img = _ball_image_hue_filter(img)
    blobs = get_hue_blobs(img)
    if blobs:
        particle_filter.iterate(blobs)
        best = get_best_blob(blobs, particle_filter)
        return is_blob_in_middle_helper(img, best)
    return False

## Continuously captures image from computer camera and feeds it to the is_ball_middle method to detect whether tennis ball is in the middle of the screen.
def run():
    global particle_filter

    cam = Camera()
    disp = Display()

    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()
        org_img = img

        # close window with left click
        if disp.mouseLeft:
            break

        _init_particle_filter(img)
        img = _ball_image_hue_filter(img)
        blobs = get_hue_blobs(img)
        if blobs:
            particle_filter.iterate(blobs)
            best = get_best_blob(blobs, particle_filter)
            if best:
                rad = best.radius()
                centroid = best.centroid()
                print "Location: (%s, %s)" % (centroid[0], centroid[1])
                # error buffer for drawing circle on img
                #rad += 10 
                # draw circle on picture
                if is_blob_in_middle_helper(img, best):
                    org_img.drawCircle(centroid, rad, (0,255,0), 2)
                    print "BALL IN MIDDLE!"
        org_img.save(disp)

