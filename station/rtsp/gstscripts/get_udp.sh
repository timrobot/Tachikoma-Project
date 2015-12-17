#!/bin/bash
PORT=9000
gst-launch-0.10 -v udpsrc port=${PORT} ! \
    jpegdec ! \
    autovideosink
