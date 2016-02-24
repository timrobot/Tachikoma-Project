#!/bin/bash

HOST=192.168.0.101
PORT=9001

gst-launch-1.0 -vvv udpsrc uri=udp://${HOST}:${PORT} ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! xvimagesink sync=false
