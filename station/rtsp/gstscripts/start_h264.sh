#!/bin/bash

HOST=192.168.0.101
PORT=9001

gst-launch-1.0 -vvv v4l2src device=/dev/video0 ! 'video/x-raw,width=640,height=480' !  x264enc pass=qual quantizer=20 tune=zerolatency ! rtph264pay ! udpsink host=${HOST} port=${PORT}
