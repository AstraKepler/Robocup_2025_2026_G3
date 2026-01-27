#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image , CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
#from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
#import tf_transformations
#from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler

class CameraViewer(Node):
    def __init__(self):
        #Set this variable to True if you want to see the output of the print()'s function for debuging
        self.debug_camera = False
        self.debug_crates = False
        self.debug_robot = False

        super().__init__('camera_viewer')

        self.subscription_info = self.create_subscription(CameraInfo, '/camera/camera_info',self.set_camera_info,10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        #self.create_subscription(TFMessage,"/mecanum_drive_controller/tf_odometry",self.broadcast_map_to_odom,10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,"/camera/pose",10)
        
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

        # ArUco marker printed on the playing arena parameters
        self.marker_length = 0.1  # meters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Playing arena markers positions (x, y, z) in meters according to our world frame  (In the above right corner of the playing arena)
        self.marker_world_positions = {
            20: [0.6, 1.4, 0],
            21: [2.4, 1.4, 0], 
            22: [0.6, 0.6, 0],
            23: [2.4, 0.6, 0]
        }

        # Crates marker parameters
        self.crate_marker_length = 0.0375  # meters
        self.crate_aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.crate_aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Yellow crates aruco ID
        self.yellow = 10
        
        # Blue crates aruco ID
        self.blue = 126

        self.yellow_crate_poses = []    # List of  yellow crates position T_world_crate 
        self.blue_crate_poses = []      # List of  blue crates position T_world_crate 

        # Nearest YELLOW crate position to the robot
        self.nearest_yellow_crate = None

        # Nearest BLUE crate position to the robot
        self.nearest_blue_crate = None

        # Robot marker parameters
        self.robot_aruco_length = 0.1   #meter
        self.robot_aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.robot_aruco_params = cv2.aruco.DetectorParameters_create()

        # Robot aruco ID
        self.robot_ID = 17

        # A 4*4 homogeneous transformation matrix of the robot in the world frame (robot Position)
        self.robot_position = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

    
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
                if (self.debug_camera):
                    cv2.aruco.drawDetectedMarkers(frame_undistorted, corners)
                    cv2.aruco.drawAxis(
                        frame_undistorted,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[i],
                        tvecs[i],
                        self.marker_length / 2
                    )

                # compute T_camera_marker
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
        
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.crate_marker_length, self.camera_matrix, self.dist_coeffs
            )
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id in [20, 21, 22, 23, self.robot_ID]: # Excluding playing arena and robot markers 
                    continue
                # Draw detection and axis
                if self.debug_crates:
                    cv2.aruco.drawDetectedMarkers(frame_undistorted, corners)
                    cv2.aruco.drawAxis(
                        frame_undistorted,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[i],
                        tvecs[i],
                        self.crate_marker_length / 2
                    )
                
                if self.debug_crates:
                    x, y = int(corners[i][0][0][0]), int(corners[i][0][0][1])
                    cv2.putText(frame_undistorted, f"Iter: {i}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Get crate pose in camera frame
                T_camera_crate = self.compute_transformation_matrix(rvecs[i], tvecs[i])
                
                # Transform to world frame
                T_world_crate = self.T_world_camera_final @ T_camera_crate
                
                if marker_id == self.yellow:
                    self.yellow_crate_poses.append(T_world_crate)

                if marker_id == self.blue:
                    self.blue_crate_poses.append(T_world_crate)
                '''
                if self.debug_crates:
                    print(f"Crate ID {marker_id} - T_world_crate{i}:")
                    print(f"{T_world_crate}\n")  
                '''

    def detect_robot_pos(self, frame_undistorted):
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame_undistorted, self.robot_aruco_dict, parameters=self.robot_aruco_params
        )
        
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.robot_aruco_length, self.camera_matrix, self.dist_coeffs
            )
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id in [20, 21, 22, 23, self.yellow, self.blue]: # Excluding playing arena and crates markers 
                    continue
                # Draw detection and axis
                if self.debug_robot:
                    cv2.aruco.drawDetectedMarkers(frame_undistorted, corners)
                    cv2.aruco.drawAxis(
                        frame_undistorted,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[i],
                        tvecs[i],
                        self.robot_aruco_length / 2
                    )
                
                # Get robot posistion in camera frame
                T_camera_robot = self.compute_transformation_matrix(rvecs[i], tvecs[i])
                
                # Transform to world frame
                self.robot_position = self.T_world_camera_final @ T_camera_robot
                '''
                if self.debug_robot:
                    print(f"T_world_Robot( AruCo ID :{marker_id}) : ")
                    print(f"{self.robot_position}\n")  
                '''
                # Extracting the robot position 
                x = self.robot_position[0, 3]
                y = self.robot_position[1, 3]

                R = self.robot_position[:3, :3]

                if not np.isfinite(R).all():
                    self.get_logger().warn("Skip Nan x, y!")
                    return  # skip publish

                yaw = np.arctan2(R[1, 0], R[0, 0])

                if not np.isfinite(yaw):
                    self.get_logger().warn("Skip Nan yaw!")
                    return
                
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"   #IMPORTANT

                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y
                msg.pose.pose.position.z = 0.0

                qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
                msg.pose.pose.orientation.x = qx
                msg.pose.pose.orientation.y = qy
                msg.pose.pose.orientation.z = qz
                msg.pose.pose.orientation.w = qw
                msg.pose.covariance = [
                    float(0.02), float(0.0),  float(0.0),  float(0.0),  float(0.0),  float(0.0),
                    float(0.0),  float(0.02), float(0.0),  float(0.0),  float(0.0),  float(0.0),
                    float(0.0),  float(0.0),  float(1e6),  float(0.0),  float(0.0),  float(0.0),
                    float(0.0),  float(0.0),  float(0.0),  float(1e6),  float(0.0),  float(0.0),
                    float(0.0),  float(0.0),  float(0.0),  float(0.0),  float(1e6),  float(0.0),
                    float(0.0),  float(0.0),  float(0.0),  float(0.0),  float(0.0),  float(0.1)
                ]
                
                if not np.isfinite([x, y, yaw]).all():
                    #self.get_logger().warn("Skip Nan orientation")
                    return
                self.pose_pub.publish(msg)
                #self.get_logger().warn("Robot location published")


    def set_nearest_yellow_crate(self):
        distances = []
        # Get the robot x, y position
        robot_position_x = self.robot_position[0][-1]
        robot_position_y = self.robot_position[1][-1]
        # Get the x, y position for each yellow crate
        for crate in self.yellow_crate_poses:
            crate_position_x = crate[0][-1]
            crate_position_y = crate[1][-1]
            # Calculate the linear distances between each crate and the robot
            robot_crates_distance = (((robot_position_x-crate_position_x)**2)+ ((robot_position_y-crate_position_y)**2))**0.5
            distances.append(robot_crates_distance)
        # Get the nearest yellow crate to the robot !!!!!
        self.nearest_yellow_crate = self.yellow_crate_poses[distances.index(min(distances))]

    def set_nearest_blue_crate(self):
        distances = []
        # Get the robot x, y Position
        robot_position_x = self.robot_position[0][-1]
        robot_position_y = self.robot_position[1][-1]
        # Get the x, y position for each blue crate
        for crate in self.blue_crate_poses:
            crate_position_x = crate[0][-1]
            crate_position_y = crate[1][-1]
            # Calculate the linear distances between each crate and the robot
            robot_crates_distance = (((robot_position_x-crate_position_x)**2)+ ((robot_position_y-crate_position_y)**2))**0.5
            distances.append(robot_crates_distance)
        #Get the nearest blue crate to the robot
        self.nearest_blue_crate = self.blue_crate_poses[distances.index(min(distances))]


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
        if(self.fx is None): # To prevent from reseting the camera parameters
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


    def image_callback(self, msg):
        
        if(self.fx is not None):
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert ROS Image to OpenCV
            # Undistort the image
            #frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            frame_undistorted = frame #We set the camera setting without any distortion (perfect "pin hole") so the frame is ready to be useed without the previous step
            
            
            if(self.T_world_camera_final is None): # Check if the camera position is set regarding to the world frame.
                self.detect_camera_pose(frame_undistorted)
            else:
                self.detect_crates_poses(frame_undistorted) # We get the position of all crates in the playing arena.
                self.detect_robot_pos(frame_undistorted)    # We get the position of the robot.
                

          
            cv2.imshow("Camera Feed", frame_undistorted)  # Comment this to save some CPU power if you dont need the camera output to show up
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    viewer = CameraViewer()
    rclpy.spin(viewer)


if __name__ == '__main__':
    main()