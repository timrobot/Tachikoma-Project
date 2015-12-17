from SimpleCV import *
from time import sleep
import sys
from datetime import datetime
import numpy as np
import experiment
from particlefilter import ParticleFilter
from image_support import *

particle_filter = None
image_half_size = -1
save_count = 1
base_filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

##
#Internal wrapper image hue filter.
#param img SimpleCV.Image the image to apply the hue filter to
#
def _basket_image_hue_filter(img):
    color = 280
    return image_hue_filter(img, False)
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

##
#Internal wrapper to particle filter initializer.
#@param img SimpleCV.Image Any image captured from the Camera, used
#to initialize the size
#
def _init_particle_filter(img):
    global particle_filter
    if not particle_filter:
        particle_filter = external_init_particle_filter(img)

## Single entry function returning True/False if basket is in the middle of the screen
#
# @param img SimpleCV.Image The image to test
#
def is_basket_middle(img):
    global particle_filter

    _init_particle_filter(img)
    img = _basket_image_hue_filter(img)
    blobs = get_hue_blobs(img)
    if blobs:
        particle_filter.iterate(blobs)
        best = get_best_blob(blobs, particle_filter)
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


##
# Runs continuously outlines best matched blob if it is in the middle
# @param bestBlobCallback function Callback called passing the best blob found
#
def run(bestBlobCallback=False):
    global particle_filter

    cam = Camera(1)
    disp = Display()
    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()
        img = _basket_image_hue_filter(img)
        _init_particle_filter(img)
        blobs = get_hue_blobs(img)
        if blobs:
            particle_filter.iterate(blobs)
            #blobs.show()
            best = get_best_blob(blobs, particle_filter)
            if is_blob_in_middle_helper(img, best):
                print "About %s inches away" % (880000.0 / best.area())
                best.drawRect(color=Color.BLUE, width=10)
            if bestBlobCallback:
                bestBlobCallback(img, best)

        img.save(disp)
        if disp.mouseLeft:
            break
        if disp.mouseRight:
            _save_image(img)
