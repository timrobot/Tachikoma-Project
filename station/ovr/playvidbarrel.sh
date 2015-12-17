#!/bin/bash
#gst-launch-0.10 v4l2src device=/dev/video0 ! videorate ! "video/x-raw-yuv,width=640,height=480,framerate=60/1" ! videocrop top=0 left=128 right=128 bottom=0 ! xvimagesink
gst-launch-1.0 -e v4l2src device=/dev/video0 ! \
    video/x-raw,format=\(string\)YUY2,width=\(int\)640,height=\(int\)480 ! \
    videocrop top=0 left=128 right=128 bottom=0 ! \
    barreldistort ! \
    xvimagesink
