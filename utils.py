import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import cv2
import open3d as o3d

def depth2PointCloud(depth, rgb, depth_scale, clip_distance_max):
    """Transform a depth image into a point cloud with one point for each
    pixel in the image, using the camera transform for a camera
    centred at cx, cy with field of view fx, fy.

    depth is a 2-D ndarray with shape (rows, cols) containing
    depths from 1 to 254 inclusive. The result is a 3-D array with
    shape (rows, cols, 3). Pixels with invalid depth in the input have
    NaN for the z-coordinate in the result.

    """
    
    intrinsics = depth.profile.as_video_stream_profile().intrinsics
    depth = np.asanyarray(depth.get_data()) * depth_scale # 1000 mm => 0.001 meters
    rgb = np.asanyarray(rgb.get_data())
    print("depth size:", depth.shape)
    print("rgb size:",  rgb.shape)
    rows,cols  = depth.shape

    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    r = r.astype(float)
    c = c.astype(float)
    print(np.max(depth))
    valid = (depth > 0) & (depth < clip_distance_max) # remove from the depth image all values above a given value (meters).
    print(valid)
    valid = np.ravel(valid)
    z = depth 
    x =  z * (c - intrinsics.ppx) / intrinsics.fx
    y =  z * (r - intrinsics.ppy) / intrinsics.fy
   
    z = np.ravel(z)[valid]
    x = np.ravel(x)[valid]
    y = np.ravel(y)[valid]
    
    r = np.ravel(rgb[:,:,0])[valid]
    g = np.ravel(rgb[:,:,1])[valid]
    b = np.ravel(rgb[:,:,2])[valid]
    
    pointsxyzrgb = np.dstack((x, y, z, b, g, r))
    pointsxyzrgb = pointsxyzrgb.reshape(-1,6)

    return pointsxyzrgb


def get_intrinsic_matrix(frame, imwidth,  imheight):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = o3d.camera.PinholeCameraIntrinsic(imwidth,  imheight, intrinsics.fx,
                                            intrinsics.fy, intrinsics.ppx,
                                            intrinsics.ppy)
    return out

# Function to create point cloud file
def create_point_cloud_file2(vertices, filename):
    ply_header = '''ply
		format ascii 1.0
		element vertex %(vert_num)d
		property float x
		property float y
		property float z
		property uchar red
		property uchar green
		property uchar blue
		end_header
		'''
    with open(filename, 'w') as f:
        f.write(ply_header %dict(vert_num=len(vertices)))
        np.savetxt(f,vertices,'%f %f %f %d %d %d')


def createPointCloudO3D(color_frame, depth_frame):
    
    color_np = np.asanyarray(color_frame.get_data())
    imwidth,  imheight, channel = color_np.shape
    color_image = o3d.geometry.Image(color_np)
    
    depth_image = o3d.geometry.Image(np.asanyarray(depth_frame.get_data()))
    #if we want to get colored point clouds covert_rgb_to_intensity should be false
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image,
        convert_rgb_to_intensity=True) # Generate colored pointclouds

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, get_intrinsic_matrix(color_frame,imwidth,  imheight))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    # Normal calculation
    pcd.estimate_normals()
    
    # Save point clouds as ply format
    o3d.io.write_point_cloud("o3d.ply", pcd)
        
    # At the specified voxel size down sampling
    #voxel_down_pcd = pcd.voxel_down_sample(voxel_size=1)
    return pcd
    

def loadPointCloud():

    pcd = o3d.io.read_point_cloud("mame.ply") # Read the point cloud
    # Visualize the point cloud within open3d
    o3d.visualization.draw_geometries([pcd]) 

    # Convert open3d format to numpy array
    # Here, you have the point cloud in numpy format. 
    point_cloud_in_numpy = np.asarray(pcd.points) 
    print(point_cloud_in_numpy)


def write_point_cloud(ply_filename, points):
    points = points.tolist()
    formatted_points = []
    for point in points:
        formatted_points.append("%f %f %f %d %d %d \n" % (point[0], point[1], point[2], point[3], point[4], point[5]))

    out_file = open(ply_filename, "w")
    out_file.write('''ply
    format ascii 1.0
    element vertex %d
    property float x
    property float y
    property float z
    property uchar blue
    property uchar green
    property uchar red
    end_header
    %s
    ''' % (len(points), "".join(formatted_points)))
    out_file.close()
