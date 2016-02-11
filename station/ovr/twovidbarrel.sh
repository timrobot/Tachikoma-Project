#!/bin/bash
gst-launch-1.0 \
  videomixer name=mix ! \
  videoconvert ! \
  xvimagesink \
  v4l2src device=/dev/video1 ! \
  videorate ! \
  video/x-raw, format=YUY2, width=640, height=480, framerate=15/1 ! \
  videocrop top=0 left=128 right=128 bottom=0 ! \
  barreldistort ! \
  videobox border-alpha=0 left=-0 ! \
  mix. \
  v4l2src device=/dev/video0 ! \
  videorate ! \
  video/x-raw, format=YUY2, width=640, height=480, framerate=15/1 ! \
  videocrop top=0 left=128 right=128 bottom=0 ! \
  barreldistort ! \
  videobox border-alpha=0 left=-384 ! \
  mix.
