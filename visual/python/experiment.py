'''
A utility file for testing out computer vision techniques on preset images.
The purpose of this is to avoid using the webcam, and test on consistent test
cases.
'''

from SimpleCV import *
import glob
import time

'''
Iterates through a directory of images, applying a custom function to each
image and then applying a custom function to find blobs and outputs the result.

@param function image_function Takes an image as parameter and returns
an image. Used for modifying the image (e.g. applying masks, etc)
@param function blob_function Takes an image and returns blobs.
@param string directory The directory to find images

Example usage:
def imgFn(img):
    return img.blur(10)
def blobFn(img):
    return img.blobs()
experiment(imgFn, blobFn)
'''
def experiment(image_function=None, blob_function=None, directory="./"):
    filenames = glob.glob("%s*.jpg" % directory)
    end = False
    blobs = None
    for filename in filenames:
        print "Displaying image: %s" % filename
        disp = Display()
        img = Image(filename)
        if image_function is not None:
            img = image_function(img)
        if blob_function is not None:
            blobs = blob_function(img)
        if blobs:
            blobs.show()
        img.save(disp)
        while disp.isNotDone():
            #click mouse left to proceed to next image
            if disp.mouseLeft:
                break
            if disp.mouseRight:
                end = True
                break
        if end:
            break
    print "End experiment"


'''
Some example image and blob functions
'''

'''
Image function
warning: very slow, use masks instead
'''
def hard_threshold(img):
    def t(p):
        if p[0] > 30 or p[1] > 80:
            return (0,0,0)
        else:
            return p
    return img.applyPixelFunction(t)

'''
Image function
makes all colors NOT in the specified range black. You can also call invert
'''
def binary_mask(img):
    mask = img.createBinaryMask(color1=(0,0,0),color2=(200,0,0))
    return img.applyBinaryMask(mask)

'''
Image function
effectively smoothes function
'''
def dilation_and_blur(img):
    img = img.dilate(10)
    img = img.blur(10)


'''
Blob function
'''
def blobs_by_mask(img):
    mask = img.createBinaryMask(color1=(0,0,35),color2=(255,255,255))
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)
    return blobs
