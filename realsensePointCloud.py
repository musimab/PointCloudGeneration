import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import cv2
import open3d as o3d
from realsense_depth import DepthCamera
from utils import createPointCloudO3D
from utils import depth2PointCloud
from utils import create_point_cloud_file2
from utils import write_point_cloud

resolution_width, resolution_height = (640, 480)

clip_distance_max = 5.00 #remove from the depth image all values above a given value (meters).
                          # Disable by giving negative value (default)

def main():

    Realsensed435Cam = DepthCamera(resolution_width, resolution_height)

    depth_scale = Realsensed435Cam.get_depth_scale()

    while True:

        ret , depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        if not ret:
            print("Unable to get a frame")
        
        #o3D library for construct point clouds with rgbd image and camera matrix
        pcd = createPointCloudO3D(color_raw_frame, depth_raw_frame)
        #o3d.visualization.draw_geometries([pcd]) 
        
        #numpy with point cloud generation
        points_xyz_rgb = depth2PointCloud(depth_raw_frame, color_raw_frame, depth_scale, clip_distance_max)
        #write_point_cloud("chair2.ply", points_xyz_rgb)
        create_point_cloud_file2(points_xyz_rgb,"chair.ply")
                
        #plt.show()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        print("frame shape:", color_frame.shape)
        cv2.imshow("Frame",  color_frame )
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.imwrite("frame_color.png", color_frame)
            plt.imsave("frame_depth.png", depth_frame)
            break
    
    Realsensed435Cam.release() # release rs pipeline


if __name__ == '__main__':
    main()
