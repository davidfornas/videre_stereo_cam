videre_stereo_cam
=================

This repository contains a catkinized version of **videre_stereo_cam** from University of Arizona. At the moment only the driver program is working as disparity_view uses very old OpenCV code that which API has changed and is not necessary because disparity_view from image_view is available.

Further changes include a hack to introduce camera parameters manually in a function that usually loads them from the camera. Camera hardware, then, it is not used. For this reason using this fork driver in STOC mode is not recommended.

Future modifications will be: a more elegant solution instead of that hack, removing unused STOC code and develop set_camera_info from ROS to be standard image_pipeline compliant.

