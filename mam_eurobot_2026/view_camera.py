#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image , CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        # Set this variable to True if you want to see the output of the print()'s function for debuging
        self.debug_camera = False
        self.debug_crates = True
        self.i = 0
        super().__init__('camera_viewer')

        self.subscription_info = self.create_subscription(CameraInfo, '/camera/camera_info',
                                                           self.set_camera_info,10)
        
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        

        # Camera intrinsic parameters will be set by set_camera_info method
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # A 4*4 homogeneous transformation matrix of the camera in the world frame
        self.T_world_camera_final = None

        # ArUco marker parameters
        self.marker_length = 0.1  # meters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Marker world positions (x, y, z) in meters
        self.marker_world_positions = {
            20: [0.6, 1.4, 0],
            21: [2.4, 1.4, 0], 
            22: [0.6, 0.6, 0],
            23: [2.4, 0.6, 0]
        }

        # The ArUco of the crtes
        self.crate_marker_length = 0.0375  # meters
        self.crate_aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.crate_aruco_params = cv2.aruco.DetectorParameters_create()
    
    def detect_camera_pose(self,frame_undistorted):
        corners, ids, _ = cv2.aruco.detectMarkers(frame_undistorted, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            T_world_camera_list = []
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id not in [20, 21, 22, 23]:
                    continue
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
                
                if (self.debug_camera):
                    print(f"Marker ID {marker_id} - T_camera_marker:")
                    print(f"{T_camera_marker}\n")
                # Compute T_world_camera
                T_world_camera = self.compute_world_camera_pose(marker_id, T_camera_marker)
                    

                T_world_camera_list.append((marker_id, T_world_camera))
                if (self.debug_camera):    
                    print(f"Marker ID {marker_id} - T_world_camera:")
                    print(f"{T_world_camera}\n")
                    print("-" * 50)
            
            if len(T_world_camera_list) > 0:
                if (self.debug_camera):
                    print(f"Computed {len(T_world_camera_list)} camera poses in world frame")
                T_sum = np.zeros((4, 4))
                for marker_id, T in T_world_camera_list:
                    T_sum += T
                self.T_world_camera_final = T_sum / len(T_world_camera_list)
                if (self.debug_camera):
                    print(f"{self.T_world_camera_final}\n")

    def detect_crates_poses(self, frame_undistorted):
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame_undistorted, self.crate_aruco_dict, parameters=self.crate_aruco_params
        )
        
        crate_poses = []  # List of (marker_id, T_world_crate) tuples
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.crate_marker_length, self.camera_matrix, self.dist_coeffs
            )
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id in [20, 21, 22, 23]: # Exclude playing arena ArUco 
                    continue
                # Draw detection and axis
                cv2.aruco.drawDetectedMarkers(frame_undistorted, corners)
                cv2.aruco.drawAxis(
                    frame_undistorted,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    self.crate_marker_length / 2
                )
                x, y = int(corners[i][0][0][0]), int(corners[i][0][0][1])
                cv2.putText(frame_undistorted, f"Iter: {i}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Get crate pose in camera frame
                T_camera_crate = self.compute_transformation_matrix(rvecs[i], tvecs[i])
                
                # Transform to world frame
                T_world_crate = self.T_world_camera_final @ T_camera_crate
                
                crate_poses.append((marker_id, T_world_crate))
                
                if self.debug_crates:
                    print(f"Crate ID {marker_id} - T_world_crate{i}:")
                    print(f"{T_world_crate}\n")
        
        return crate_poses      
          
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
    
    def set_camera_info(self, msg):
        if(self.fx is None): # To prevent it from reseting the camera parameters
            self.fx = msg.p[0]
            self.fy = msg.p[5]
            self.cx = msg.p[2]
            self.cy = msg.p[6]
            self.camera_matrix = np.array([[self.fx,     0,      self.cx],
                                        [  0,         self.fy,    self.cy],
                                        [  0,            0,        1    ]])
            self.dist_coeffs = np.array(msg.d)
            print("K:", self.camera_matrix)
            print("D:", self.dist_coeffs)

    def crate_detection(self, frame_undistorted):
        corners, ids, _ = cv2.aruco.detectMarkers(frame_undistorted, self.crate_aruco_dict, parameters=self.crate_aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.crate_marker_length, self.camera_matrix, self.dist_coeffs
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

    def image_callback(self, msg):
        if(self.fx is not None):
            
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Undistort the image
            #frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            frame_undistorted = frame
            
            
            
            if(self.T_world_camera_final is None): 
                self.detect_camera_pose(frame_undistorted)
            else:
                '''
                if(self.i == 0):
                    self.i =1
                '''  
                self.detect_crates_poses(frame_undistorted)
          
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