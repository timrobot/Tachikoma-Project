#!/bin/bash

DEV=/dev/video0
URL=0.0.0.0 # change this to the URL of the destination
PORT=9000

gst-launch-0.10 v4l2src device=${DEV} ! 'video/x-raw-yuv,width=640,height=480,framerate=30/1' ! \
    queue ! ffmpegcolorspace ! jpegenc ! \
    udpsink host=${URL} port=${PORT}
