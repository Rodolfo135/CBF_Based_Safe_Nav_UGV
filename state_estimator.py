import math

class StateEstimator:
    def __init__(self, twin):
        self.twin = twin
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def update(self):
        """Polls the twin for the latest state and parses coordinates."""
        # Force the twin to update its internal state over HTTP
        self.twin.refresh()
        data = self.twin._data
        
        self.current_x = data.position_x
        self.current_y = data.position_y
        
        # Extract rotation quaternion and convert to Yaw (in degrees)
        qw, qx, qy, qz = data.rotation_w, data.rotation_x, data.rotation_y, data.rotation_z
        
        # Math magic to convert a quaternion to an Euler Z-axis rotation (yaw)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        
        return self.current_x, self.current_y, self.current_yaw