# depth_color_alignment_ros
The project is a ROS wrapper of https://github.com/xiaoqingyue/depth_color_alignment.git

* Subscribe `/camera/rgb/image_raw` and `/camera/depth/image_raw`
* Align depth image to color image for rgbd camera
* Publish registered depth image `/camera/depth_registered/image_raw`

Set the camera parameter in registration.cpp before use. 
