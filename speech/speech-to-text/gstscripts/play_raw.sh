#!/bin/bash
gst-launch-0.10 -v filesrc location=$1 ! 'audio/x-raw-int, endianness=1234, signed=true, width=16, height=16, rate=16000, channels=1' ! alsasink
