#!/bin/bash

for i in 0 1 2 3 4 5
do

  v4l2-ctl --set-ctrl=gain_automatic=0 -d /dev/video$i
  v4l2-ctl --set-ctrl=gain=10 -d /dev/video$i

  v4l2-ctl --set-ctrl=auto_exposure=1 -d /dev/video$i
  v4l2-ctl --set-ctrl=exposure=50 -d /dev/video$i

done
