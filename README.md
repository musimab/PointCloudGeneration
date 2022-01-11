
## Installation

pip install open3d

pip install pyrealsense2

## Usage
There are two options for obtaining point cloud data in ply format, first we can create point clouds with numpy or we can use ready-made functions from open3d library. But the important point is to understand how these point clouds are formed, so it will be more thought-provoking to write it from scratch with numpy.That's why I added both functions to the utils.py

python realsensePointCloud.py


## From depth map to point cloud

This repo introduces the intrinsic matrix and walks you through how you can use it to convert an RGBD (red, blue, green, depth) image to 3D space. RGBD images can be obtained in many ways. E.g. from a system like Kinect that uses infrared-based time-of flight detection.
In this project I used Intel Realsense D435 RGBD camera to get point clouds and introduced some functions in order to obtain pointclouds.

First of all, we need to adjust the settings of the camera where image width and height are selected
in DepthCamera class. Realsense camera configuration consist of many options these are (rs.format.bgr8 get frames in bgr format), frame rate...

"scaling factor" refers to the relation between depth map units and meters; 
it has nothing to do with the focal length of the camera.
Depth maps are typically stored in 16-bit unsigned integers at millimeter scale, thus to obtain Z value in meters, the depth map pixels need to be divided by 1000.


