#!/bin/bash

gst-launch-1.0 videotestsrc horizontal-speed=5 ! x264enc tune="zerolatency" threads=1 ! mpegtsmux ! tcpserversink port=8554
