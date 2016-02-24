#!/bin/bash

HOST=192.168.43.57
PORT=9001

gst-launch-1.0 -vvv tcpclientsrc port=${PORT} host=${HOST} ! \
  tsdemux ! \
  h264parse ! \
  avdec_h264 ! \
  jpegenc ! \
  multifilesink post-messages=true location="frame%05d.jpg" max-files=100
