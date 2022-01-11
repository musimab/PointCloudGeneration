
## Installation

pip install open3d

pip install pyrealsense2

## Usage
There are two options for obtaining point cloud data in ply format, first we can create point clouds with numpy or we can use ready-made functions from open3d library. But the important point is to understand how these point clouds are formed, so it will be more thought-provoking to write it from scratch with numpy.That's why I added both functions to the utils.py

"""
python realsensePointCloud.py
"""

## From depth map to point cloud

This repo introduces the intrinsic matrix and walks you through how you can use it to convert an RGBD (red, blue, green, depth) image to 3D space. RGBD images can be obtained in many ways. E.g. from a system like Kinect that uses infrared-based time-of flight detection.
In this project I used Intel Realsense D435 RGBD camera to get point clouds and introduced some functions in order to obtain pointclouds.

First of all, we need to adjust the settings of the camera where image width and height are selected
in DepthCamera class. Realsense camera configuration consist of many options these are (rs.format.bgr8 get frames in bgr format), frame rate...

"scaling factor" refers to the relation between depth map units and meters; 
it has nothing to do with the focal length of the camera.
Depth maps are typically stored in 16-bit unsigned integers at millimeter scale, thus to obtain Z value in meters, the depth map pixels need to be divided by 1000.
### Color image
![frame_color](https://user-images.githubusercontent.com/47300390/149009191-fe030fb6-086c-480f-bdb1-0ff888e8a5fe.png)

### Depth image
![vazo](https://user-images.githubusercontent.com/47300390/149009206-d9734e92-8999-4fc6-a879-ff1a044136ae.png)

### Pointclouds
![Screenshot from 2022-01-11 22-20-21](https://user-images.githubusercontent.com/47300390/149009242-dc13cd96-24f1-45cd-b0ce-28018d50fde5.png)
![Screenshot from 2022-01-11 22-20-44](https://user-images.githubusercontent.com/47300390/149009258-a1364987-6d77-4185-ae21-f821ffdc7be6.png)


