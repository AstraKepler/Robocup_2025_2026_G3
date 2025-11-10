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

            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(frame_undistorted, corners)
                cv2.aruco.drawAxis(
                    frame_undistorted,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    self.marker_length / 2
                )

                # Compute transformation matrix
                R, _ = cv2.Rodrigues(rvecs[i])
                t = tvecs[i].reshape((3,))
                T_cam_marker = np.eye(4)
                T_cam_marker[:3, :3] = R
                T_cam_marker[:3, 3] = t

                print(f"Marker ID {ids[i][0]}:\n{T_cam_marker}\n")

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