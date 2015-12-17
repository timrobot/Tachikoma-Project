#!/usr/bin/python
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
import signal

# global particle filter
# error point elimination
particle_filter = None
mode = "basket" # basket|ball
quiet = False

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
    global mode
    color = (185, 206, 111)
    return image_hue_filter(img, mode)

## Entry point for module which determines whether tennis ball is in the middle of the image.
#
# @param img SimpleCV.Image 
# @return boolean. True if tennis ball is in middle, false otherwise.
def is_ball_middle(img):
    global particle_filter, mode

    _init_particle_filter(img)
    img = _ball_image_hue_filter(img, mode)
    blobs = get_hue_blobs(img)
    if blobs:
        particle_filter.iterate(blobs)
        best = get_best_blob(blobs, particle_filter, mode)
        return is_blob_in_middle_helper(img, best)
    return False

##
#Internal wrapper image hue filter.
#param img SimpleCV.Image the image to apply the hue filter to
#
def _basket_image_hue_filter(img):
    color = 280
    return image_hue_filter(img, "basket")
##
#Saves an image to the current directory
#@param img SimpleCV.Image the image to save
#
def _save_image(img):
    global save_count, base_filename
    f = "cam_capture_%s_%s.jpg" % (base_filename, save_count)
    img.save(f)
    print "saved image %s" % f
    save_count += 1

## Single entry function returning True/False if basket is in the middle of the screen
#
# @param img SimpleCV.Image The image to test
#
def is_basket_middle(img):
    global particle_filter, mode

    _init_particle_filter(img)
    img = _basket_image_hue_filter(img, mode)
    blobs = get_hue_blobs(img)
    if blobs:
        particle_filter.iterate(blobs)
        best = get_best_blob(blobs, particle_filter, mode)
        return is_blob_in_middle_helper(img, best)
    return False

##
# Runs continuously and prints if the best detected blob is in the middle
#
def run_middle():
    def middle_callback(img, best):
        if is_blob_in_middle_helper(img, best):
            print "Basket in middle"
    run(middle_callback)



## Continuously captures image from computer camera and feeds it to the is_ball_middle method to detect whether tennis ball is in the middle of the screen.
def run():
    global particle_filter, mode, quiet
    cam_width = 320
    cam_height = 240
    cam = Camera(0, {"width" : cam_width, "height" : cam_height})
    disp = Display()
    best_found = False
    
    print "VISUAL-PROC-STARTED"
    sys.stdout.flush()
    while True:
        best_found = False
        #sleep(.05)
        img = cam.getImage()
        # close window with left click
        #if disp.mouseLeft:
            #break
        if mode == "ball":        
            img = _ball_image_hue_filter(img)
            #img = img.dilate(2)
            img = img.smooth()
            blobs = get_hue_blobs(img)
            if blobs:
                #blobs.draw()
                best = get_best_blob(blobs, particle_filter, mode)
                if best:
                    best_found = True
                    rad = best.radius()
                    centroid = best.centroid()
                    img.drawCircle(centroid, rad, (0,255,0), 2)
                    dist = (38 * 1200) / best.area()
                    if not quiet:
                        print "coordinate:[%s %s %s %s]" % (mode, centroid[0] - cam_width/2, centroid[1] - cam_height/2, dist)
                        sys.stdout.flush()
        elif mode == "basket":
            img = cam.getImage()
            img = _basket_image_hue_filter(img)
            blobs = get_hue_blobs(img)
            if blobs:
                best = get_best_blob(blobs, particle_filter, mode)                
                if best:
                    best_found = True
                    centroid = best.centroid()
                    rect = best.minRect()
                    height = max([abs(p1[1] - p2[1]) for p1 in rect for p2 in rect])
                    dist = (38 * 140) / height
                    best.drawRect(color=Color.BLUE, width=2)
                    if not quiet:
                        print "coordinate:[%s %s %s %s]" % (mode, centroid[0] - cam_width/2, centroid[1] - cam_height/2, dist)
                        sys.stdout.flush()
                else:
                    if not quiet:
                        print "notfound:[%s]" % mode
                        sys.stdout.flush()
        if not best_found and not quiet:
            print "notfound:[%s]" % mode
            sys.stdout.flush()
        if mode == "basket":
            mode = "ball"
        else:
            mode = "basket"
        img.save(disp)
        sys.stdout.flush()

def switch_handler(signum, frame):
    global mode
    if signum == 40:
        mode = "ball"
    elif signum == 41:
        mode = "basket"

def end_handler(signum, frame):
    sys.exit(1)
    
signal.signal(40, switch_handler)
signal.signal(41, switch_handler)
signal.signal(signal.SIGINT, end_handler)

run()
