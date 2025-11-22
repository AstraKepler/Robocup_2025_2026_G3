#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # Camera intrinsic parameters
        self.fx = 692.9783821105957
        self.fy = 692.9783821105957
        self.cx = 400.0
        self.cy = 300.0
        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                       [0, self.fy, self.cy],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([-0.25, 0.12, -0.00028, -5e-05, 0.0])

        # ArUco marker parameters
        self.marker_length = 0.1  # meters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Marker world positions (x, y, z) in meters
        self.marker_world_positions = {
            20: [0.6, 1.4, 0],
            21: [2.4, 1.4, 0], 
            22: [0.6, 0.6, 0],
            23: [2.4, 0.6, 0]
        }

    def compute_transformation_matrix(self, rvec, tvec):
        """Convert rotation vector and translation vector to 4x4 transformation matrix"""
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape((3,))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def compute_world_camera_pose(self, marker_id, T_camera_marker):
        """Compute T_world_camera using marker's known world position"""
        if marker_id not in self.marker_world_positions:
            return None

        # Get marker's world position
        p = self.marker_world_positions[marker_id]
        
        # T_world_marker
        T_world_marker = np.eye(4)
        T_world_marker[0:3, 3] = p  # Set translation
        
        # Invert the ArUco pose: T_marker_camera = inv(T_camera_marker)
        T_marker_camera = np.linalg.inv(T_camera_marker)
        
        # Compute world->camera: T_world_camera = T_world_marker * T_marker_camera
        T_world_camera = T_world_marker @ T_marker_camera
        
        return T_world_camera

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Undistort the image
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame_undistorted, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            T_world_camera_list = []

            for i in range(len(ids)):
                marker_id = ids[i][0]
                
                # Draw detection and axis
                cv2.aruco.drawDetectedMarkers(frame_undistorted, corners)
                cv2.aruco.drawAxis(
                    frame_undistorted,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    self.marker_length / 2
                )

                # Compute T_camera_marker
                T_camera_marker = self.compute_transformation_matrix(rvecs[i], tvecs[i])
                
                print(f"Marker ID {marker_id} - T_camera_marker:")
                print(f"{T_camera_marker}\n")

                # Compute T_world_camera
                T_world_camera = self.compute_world_camera_pose(marker_id, T_camera_marker)
                
                if T_world_camera is not None:
                    T_world_camera_list.append((marker_id, T_world_camera))
                    
                    print(f"Marker ID {marker_id} - T_world_camera:")
                    print(f"{T_world_camera}\n")
                    print("-" * 50)

            # You can now use T_world_camera_list for further processing
            # For example, average multiple camera poses for better accuracy
            if len(T_world_camera_list) > 0:
                print(f"Computed {len(T_world_camera_list)} camera poses in world frame")

        cv2.imshow("Camera Feed", frame_undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main():
    rclpy.init()
    viewer = CameraViewer()
    rclpy.spin(viewer)


if __name__ == '__main__':
    main()