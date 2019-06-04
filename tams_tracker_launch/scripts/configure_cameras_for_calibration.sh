#!/bin/bash

v4l2-ctl --set-ctrl=gain_automatic=0 -d /dev/video0
v4l2-ctl --set-ctrl=gain=10 -d /dev/video0

v4l2-ctl --set-ctrl=gain_automatic=0 -d /dev/video1
v4l2-ctl --set-ctrl=gain=10 -d /dev/video1

v4l2-ctl --set-ctrl=gain_automatic=0 -d /dev/video2
v4l2-ctl --set-ctrl=gain=10 -d /dev/video2



v4l2-ctl --set-ctrl=auto_exposure=1 -d /dev/video0
v4l2-ctl --set-ctrl=exposure=100 -d /dev/video0

v4l2-ctl --set-ctrl=auto_exposure=1 -d /dev/video1
v4l2-ctl --set-ctrl=exposure=100 -d /dev/video1

v4l2-ctl --set-ctrl=auto_exposure=1 -d /dev/video2
v4l2-ctl --set-ctrl=exposure=100 -d /dev/video2
