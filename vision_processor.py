import numpy as np
from cyberwave.sensor import CV2CameraStreamer, CameraConfig, Resolution

class VisionProcessor:
    def __init__(self, mqtt_client, twin_uuid):
        self.min_obstacle_dist = 10.0  # Meters
        
        # D455 Profile: VGA resolution is usually best for depth-processing speed
        self.config = CameraConfig(
            resolution=Resolution.VGA, 
            fps=15, 
            camera_id=0 # Usually 0 is the primary RealSense mount
        )
        
        try:
            self.streamer = CV2CameraStreamer.from_config(
                mqtt_client, 
                self.config, 
                twin_uuid=twin_uuid
            )
            # We subscribe to the depth stream specifically
            self.streamer.subscribe(self._process_depth_frame)
            print("Successfully linked to RealSense D455 Depth Stream.")
        except Exception as e:
            print(f"Vision Error: {e}")

    def _process_depth_frame(self, frame):
        """
        Processes the D455 depth map.
        In Cyberwave, depth frames are often 16-bit arrays where 
        values represent millimeters.
        """
        # 1. Define ROI: The central 100x100 pixels (the robot's 'center of mass')
        height, width = frame.shape
        roi_size = 50
        center_y, center_x = height // 2, width // 2
        
        roi = frame[center_y-roi_size : center_y+roi_size, 
                    center_x-roi_size : center_x+roi_size]

        # 2. Convert millimeters to meters and filter out 0 (invalid/too far)
        depth_meters = roi.astype(float) / 1000.0
        valid_depths = depth_meters[depth_meters > 0.1] # Filter noise < 10cm

        # 3. Update the shared distance state
        if valid_depths.size > 0:
            self.min_obstacle_dist = np.min(valid_depths)
        else:
            self.min_obstacle_dist = 10.0 # Clear path

    def get_distance(self):
        return self.min_obstacle_dist