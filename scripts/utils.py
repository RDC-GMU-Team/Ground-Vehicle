import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from nav_msgs.msg import Path , Odometry
import csv
#from grid_map_msgs.msg import GridMap

import numpy as np
#import pandas as pd
import math
import torch
import pyproj

#Class for general functions
class utilities:
    def __init__(self):
        self.queue_size = 0
    
    #map value from one range to another
    def map_value(self, value, from_min, from_max, to_min, to_max):
        # Calculate the range of the input value
        from_range = from_max - from_min

        # Calculate the range of the output value
        to_range = to_max - to_min

        # Scale the input value to the output range
        mapped_value = (value - from_min) * (to_range / from_range) + to_min

        return mapped_value
    
    # read goal points from csv file
    def csv_to_goals(self, filename):
        """
        df = pd.read_csv(filename)
        return df.values.tolist()
        """
        # Load the CSV file
        waypoints = []
        with open(filename, mode='r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                x, y = map(float, row)
                waypoints.append((x, y))
        return waypoints
        
    
    # convert lat lon to x y
    def gps_to_xy(self, latitude, longitude):
        projection = pyproj.Proj("+proj=utm +zone=17 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
        
        #x, y = pyproj.transform(projection, projection.inv, longitude, latitude)
        x, y = projection(longitude, latitude)
        return np.array([x, y], dtype=np.float32)

    # clamp angles
    def clamp_angle(self, angles):
        angles += np.pi
        angles %= (2 * np.pi)
        angles -= np.pi
        return angles
    
    # convert angle from 0-360 to -pi to pi
    def gps_to_normal_heading(self, angle_degrees):
        """Converts an angle from 0-360 degrees to the equivalent angle within -180 to 180 radians.

        Args:
            angle_degrees (float): The angle in degrees.

        Returns:
            float: The angle in radians, within the range of -180 to 180.
        """  
        # Bring angle within 0 to 360 range
        angle_degrees = angle_degrees % 360

        # Convert to radians
        angle_radians = np.radians(angle_degrees)

        # Shift to -180 to 180 range
        if angle_radians > np.pi:
            angle_radians -= 2 * np.pi

        return -angle_radians
    
    # Convert quaternion to yaw angle (in radians)
    def quaternion_to_yaw(self, quaternion):
        quaternion_norm = math.sqrt(quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2)
        if (quaternion_norm == 0):
            return 0.0
        quaternion.x /= quaternion_norm
        quaternion.y /= quaternion_norm
        quaternion.z /= quaternion_norm
        quaternion.w /= quaternion_norm

        yaw = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                         1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2))

        return yaw
    
    # Convert yaw angle (in radians) to quaternion
    def yaw_to_quaternion(self, yaw):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion
    
    # Get distance between two poses
    def get_dist(self, start_pose, goal_pose):
        if start_pose is None or goal_pose is None:
            return 0.0
        
        if start_pose.ndim == 1:  # Handle single pose case
            start_pose = np.expand_dims(start_pose, axis=0)
        
        goal_expanded = np.expand_dims(goal_pose, axis=0)
        squared_diff = np.sum((start_pose[:,:2] - goal_expanded[:,:2]) ** 2, axis=1)
        return np.sqrt(squared_diff)

    # Create a Pose message from a position and orientation
    def create_pose_stamped(self, pose):
        # Create a PoseStamped message from a Pose message
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = 'odom'  # Replace 'world' with your desired frame ID

        return pose_stamped
    
    # Ackermann model for calculating pose change
    def ackermann_model(self, velocity, steering, wheelbase, dt):
        # Initialize the pose change
        pose_change = np.zeros((velocity.shape[0], 3), dtype=np.float32)

        # Calculate the change in orientation (dtheta)
        dtheta = velocity / wheelbase * np.tan(steering) * dt

        # Calculate change in x and y coordinates
        dx = velocity * np.cos(dtheta) * dt
        dy = velocity * np.sin(dtheta) * dt
    
        pose_change[:, 0] = dx
        pose_change[:, 1] = dy
        pose_change[:, 2] = dtheta

        return pose_change
    
    # Convert SE2 poses from world to robot frame
    def to_robot_se2(self, p1_batch, p2_batch):
        # Ensure the inputs are tensors
        p1_batch = torch.tensor(p1_batch, dtype=torch.float32)
        p2_batch = torch.tensor(p2_batch, dtype=torch.float32)

        # Validate inputs
        if p1_batch.shape != p2_batch.shape or p1_batch.shape[-1] != 3:
            raise ValueError("Both batches must be of the same shape and contain 3 elements per pose")

        # Extract components
        x1, y1, theta1 = p1_batch[:, 0], p1_batch[:, 1], p1_batch[:, 2]
        x2, y2, theta2 = p2_batch[:, 0], p2_batch[:, 1], p2_batch[:, 2]

        # Construct SE2 matrices
        zeros = torch.zeros_like(x1)
        ones = torch.ones_like(x1)
        T1 = torch.stack([torch.stack([torch.cos(theta1), -torch.sin(theta1), x1]),
                            torch.stack([torch.sin(theta1),  torch.cos(theta1), y1]),
                            torch.stack([zeros, zeros, ones])], dim=-1).permute(1,2,0)

        T2 = torch.stack([torch.stack([torch.cos(theta2), -torch.sin(theta2), x2]),
                            torch.stack([torch.sin(theta2),  torch.cos(theta2), y2]),
                            torch.stack([zeros, zeros, ones])], dim=-1).permute(1,2,0)

        # Inverse of T1 and transformation
        T1_inv = torch.inverse(T1)
        tf2_mat = torch.matmul(T2, T1_inv)

        # Extract transformed positions and angles
        transform = torch.matmul(T1_inv, torch.cat((p2_batch[:,:2], ones.unsqueeze(-1)), dim=1).unsqueeze(2)).squeeze()
        transform[:, 2] = torch.atan2(tf2_mat[:, 1, 0], tf2_mat[:, 0, 0])
        
        return transform.numpy()

    # Convert SE2 poses from robot to world frame
    def to_world_se2(self, p1_batch, p2_batch):
        # # Ensure the inputs are tensors
        p1_batch = torch.tensor(p1_batch, dtype=torch.float32)
        p2_batch = torch.tensor(p2_batch, dtype=torch.float32)

        # Validate inputs
        if p1_batch.shape != p2_batch.shape or p1_batch.shape[-1] != 3:
            raise ValueError("Both batches must be of the same shape and contain 3 elements per pose")

        # Extract components
        x1, y1, theta1 = p1_batch[:, 0], p1_batch[:, 1], p1_batch[:, 2]
        x2, y2, theta2 = p2_batch[:, 0], p2_batch[:, 1], p2_batch[:, 2]

        # Construct SE2 matrices
        zeros = torch.zeros_like(x1)
        ones = torch.ones_like(x1)
        T1 = torch.stack([torch.stack([torch.cos(theta1), -torch.sin(theta1), x1]),
                            torch.stack([torch.sin(theta1),  torch.cos(theta1), y1]),
                            torch.stack([zeros, zeros, ones])], dim=-1).permute(1,2,0)

        T2 = torch.stack([torch.stack([torch.cos(theta2), -torch.sin(theta2), x2]),
                            torch.stack([torch.sin(theta2),  torch.cos(theta2), y2]),
                            torch.stack([zeros, zeros, ones])], dim=-1).permute(1,2,0)

        # Inverse of T1 and transformation
        T_tf = torch.matmul(T2, T1)

        # Extract transformed positions and angles
        transform = torch.matmul(T1, torch.cat((p2_batch[:,:2], ones.unsqueeze(-1)), dim=1).unsqueeze(2)).squeeze()
        transform[:, 2] = torch.atan2(T_tf[:, 1, 0], T_tf[:, 0, 0])
        
        return transform.numpy()