#!/bin/bash
gst-launch-1.0 \
  videomixer name=mix ! \
  jpegenc ! \
  multifilesink post-messages=true location="frame%zu.jpg" max-files=100 \
  v4l2src device=/dev/video1 ! \
  videorate ! \
  video/x-raw, width=640, height=480, framerate=60/1 ! \
  videoflip method=clockwise ! \
  videobox border-alpha=0 left=-0 ! \
  mix. \
  v4l2src device=/dev/video0 ! \
  videorate ! \
  video/x-raw, width=640, height=480, framerate=60/1 ! \
  videoflip method=clockwise \
  videobox border-alpha=0 left=-480 ! \
  mix.
