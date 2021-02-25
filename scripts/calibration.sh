#!/usr/bin/env sh

rosrun camera_calibration cameracalibrator.py --size 7x4 --square 0.03 image:=/rosbot/main_camera/image_raw camera:=/rosbot/main_camera
