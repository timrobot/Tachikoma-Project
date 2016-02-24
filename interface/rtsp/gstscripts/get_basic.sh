#!/bin/bash

gst-launch-1.0 tcpclientsrc port=8554 host=localhost  ! tsdemux ! h264parse ! avdec_h264 ! xvimagesink 
