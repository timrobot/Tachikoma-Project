#!/bin/bash
gst-launch-0.10 \
  videomixer name=mix ! \
  ffmpegcolorspace ! \
  xvimagesink \
  v4l2src device=/dev/video0 ! \
  "video/x-raw-yuv,width=640,height=480,framerate=60/1" ! \
  videocrop top=0 left=128 right=128 bottom=0 ! \
  videobox border-alpha=0 left=0 ! \
  mix. \
  v4l2src device=/dev/video2 ! \
  "video/x-raw-yuv,width=640,height=480,framerate=60/1" ! \
  videocrop top=0 left=128 right=128 bottom=0 ! \
  videobox border-alpha=0 left=-640 ! \
  mix.
