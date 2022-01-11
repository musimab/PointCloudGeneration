import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self, imwidth, imheight):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        #print(img_width, img_height)
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        depth_sensor = device.first_depth_sensor()
        # Get depth scale of the device
        self.depth_scale =  depth_sensor.get_depth_scale()
       
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth,  imwidth,  imheight, rs.format.z16, 30)
        config.enable_stream(rs.stream.color,  imwidth,  imheight, rs.format.bgr8, 30)


        # Start streaming
        self.pipeline.start(config)
       
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def get_raw_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_frame, color_frame
    
    def get_depth_scale(self):
        return self.depth_scale

    def release(self):
        self.pipeline.stop()