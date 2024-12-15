# Lane-Tracking-and-vslam-on-turtlebot3
Team Member: Yihao Liao && Samer Fnis

## Project Part 1
### 1.Install Autorace Packages
1.Install the AutoRace 2020 meta package
```
cd ~/catkin_ws/src/
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
cd ~/catkin_ws && catkin_make
```
2.Install additional dependent packages
```
sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-opencv python3-opencv libopencv-dev ros-noetic-image-proc

```
### 2.Camera Calibration
#### 2.1 Intrinsic Camera Calibration
1.Launch roscore on PC
```
roscore
```
2.Trigger the camera on SBC
```
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```
3.Run a intrinsic camera calibration launch file on PC
```
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=calibration
```
4.Use the checkerboard to calibrate the camera, and click CALIBRATE

5.Click Save to save the intrinsic calibration data.

6.calibrationdata.tar.gz folder will be created at /tmp folder.

7.Extract calibrationdata.tar.gz folder, and open ost.yaml.

8.Copy and paste the data from ost.yaml to camerav2_320x240_30fps.yaml.
#### 2.2 Extrinsic Camera Calibration
1.Launch roscore on PC
```
roscore
```
2.Trigger the camera on SBC
```
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```
3.Use the command on PC
```
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
```
4.Run the extrinsic camera calibration launch file on PC.
```
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration
```
5.Execute rqt on PC
```
rqt
```
6.Click plugins > visualization > Image view; Multiple windows will be present.
7.Select /camera/image_extrinsic_calib/compressed and /camera/image_projected_compensated topics on each monitors.

8.Excute rqt_reconfigure on Remote PC
```
rosrun rqt_reconfigure rqt_reconfigure
```
9.Adjust parameters in /camera/image_projection and /camera/image_compensation_projection.
Change /camera/image_projection parameter value.

### Lane Detection
1.Place the TurtleBot3 inbetween yellow and white lanes.

NOTE: The lane detection filters yellow on the left side while filters white on the right side. Be sure that the yellow lane is on the left side of the robot.

2.Open a new terminal and launch Autorace Gazebo simulation. The roscore will be automatically launched with the roslaunch command.
```
roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch
```
3.Open a new terminal and launch the intrinsic calibration node.
```
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch
```
4.Open a new terminal and launch the extrinsic calibration node.
```
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch
```
5.Open a new terminal and launch the lane detection calibration node.
```
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration
```
6.Open a new terminal and launch the rqt.
```
rqt
```
7.Launch the rqt image viewer by selecting Plugins > Cisualization > Image view.
Multiple rqt plugins can be run.

8.Display three topics at each image viewer
/detect/image_lane/compressed

9.Open a new terminal and execute rqt_reconfigure.
```
rosrun rqt_reconfigure rqt_reconfigure
```
10.Click detect_lane then adjust parameters so that yellow and white colors can be filtered properly.

11.Open lane.yaml file located in turtlebot3_autorace_detect/param/lane/. You need to write modified values to the file. This will make the camera set its parameters as you set here from next launching.

12.Close the terminal or terminate with Ctrl + C on rqt_reconfigure and detect_lane terminals.

13.Open a new terminal and launch the lane detect node without the calibration option.
```
roslaunch turtlebot3_autorace_detect detect_lane.launch
```
13.Open a new terminal and launch the node below to start the lane following operation.
```
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
```

## Project Part 2
### 1. Camera Calibration

It will be the same as you did on the first part.

### 2. Modify the detect_lane.py file
Add this code into the maskYellowLane function and return yellow_coords in the end
```
        yellow_coords = cv2.findNonZero(mask)
        if yellow_coords is not None:
            yellow_coords = np.squeeze(yellow_coords).tolist()
        else:
            yellow_coords = []
```
Add this code into the maskWhiteLane function and return white_coords in the end
```
        white_coords = cv2.findNonZero(mask)
        if white_coords is not None:
            white_coords = np.squeeze(white_coords).tolist()
        else:
            white_coords = []
```
### 3.Create a node file
```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from detect_lane import DetectLane
import rospy
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image, CompressedImage
import tf
from cv_bridge import CvBridge, CvBridgeError
import json

class LaneDetector:
    def __init__(self):

        # Creating a CvBridge Instance
        self.cvBridge = CvBridge()

        # Subscribe to image and odometry data
        self.image_sub = rospy.Subscriber('/camera/image_projected_compensated', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Initialize detectors and other variables
        self.detect_lane = DetectLane()
        self.detect_lane.sub_image_type = 'raw'
        self.q = [0, 0, 0]  # 机器人位姿初始化

    def image_callback(self, image_msg):
        try:
            # Convert a ROS Image message to an OpenCV image
            print("comes to image_callback")
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            # Image processing and coordinate transformation
            self.process_image(cv_image)
            
            

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

    def odometry_callback(self, msg):
        # Update robot pose
        pose = msg.pose.pose
        orientation_q = pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.q = [pose.position.x, pose.position.y, yaw]


    def get_global_pose(self):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(3.0))
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            x, y, _ = trans
            roll, pitch, yaw = euler_from_quaternion(rot)
            return x, y, yaw
        except tf.Exception as e:
            rospy.logerr(f"Failed to get transform: {e}")
            return None

    def process_image(self, cv_image):
    # Camera intrinsic matrix
        K = np.array(
            [[ -3.3130611,0, 160.5], [0, -3.3130611 , 120.5], [0, 0, 1.]]
        )

        # Camera extrinsics (assuming known)
        R = np.eye(3)
        t = np.array([0, 0, 0.1]).reshape(-1, 1)

        try:
            valid_yellow, _, yellow_coords = self.detect_lane.maskYellowLane(cv_image)
            valid_white, _, white_coords = self.detect_lane.maskWhiteLane(cv_image)

            if not valid_yellow or not valid_white:
                rospy.logwarn("Lane detection returned invalid data.")
                return

            # print("Yellow lane coordinates:", yellow_coords)
            # print("White lane coordinates:", white_coords)

            # 数据简化
            N = 100
            simplified_white_coords = white_coords[::N] if white_coords else []
            simplified_yellow_coords = yellow_coords[::N] if yellow_coords else []

            if not simplified_white_coords or not simplified_yellow_coords:
                rospy.logwarn("Simplified coordinates are empty.")
                return

        except Exception as e:
            rospy.logerr(f"Error during lane processing: {e}")
            return

        points_img_white = np.array([[x, y, 1] for x, y in simplified_white_coords])
        points_img_yellow = np.array([[x, y, 1] for x, y in simplified_yellow_coords])

        # Convert image coordinates to ground coordinates
        points_real_white = np.linalg.inv(K) @ (R @ points_img_white.T + t)
        points_real_white /= points_real_white[2, :]  # 齐次坐标归一化
        points_real_white = points_real_white.T  # 转置为 N x 3

        points_real_yellow = np.linalg.inv(K) @ (R @ points_img_yellow.T + t)
        points_real_yellow /= points_real_yellow[2, :]
        points_real_yellow = points_real_yellow.T

        # Transformation from robot coordinate system to world coordinate system
        x_r, y_r, theta_r = self.q
        T_robot_to_world = np.array([
            [math.cos(theta_r), -math.sin(theta_r), x_r],
            [math.sin(theta_r), math.cos(theta_r), y_r],
            [0, 0, 1]
        ])
        T_world_to_robot = np.linalg.inv(T_robot_to_world)

        # Convert ground coordinates to robot coordinates
        points_robot_white = (T_world_to_robot @ points_real_white.T).T
        points_robot_yellow = (T_world_to_robot @ points_real_yellow.T).T
        
        slam_pose = self.get_global_pose()
        x_s, y_s, theta_s = slam_pose  # 来自 SLAM 系统

        # SLAM map coordinate transformation matrix
        T_robot_to_map = np.array([
            [math.cos(theta_s), -math.sin(theta_s), x_s],
            [math.sin(theta_s),  math.cos(theta_s), y_s],
            [0,                 0,                 1]
        ])

        # Convert to map coordinate system
        points_map_white = (T_robot_to_map @ points_robot_white.T).T
        points_map_yellow = (T_robot_to_map @ points_robot_yellow.T).T

        # Extract x, y coordinates
        x_map_white, y_map_white = points_map_white[:, 0], points_map_white[:, 1]
        x_map_yellow, y_map_yellow = points_map_yellow[:, 0], points_map_yellow[:, 1]
        
        print("points_robot_white:", points_map_white)
        
        # save points to file
        self.save_points_to("/home/yirenqiu/turtlebot3_ws/points.txt", zip(x_map_white, y_map_white), zip(x_map_yellow, y_map_yellow))

    
    def save_points_to(self, filename, points_white, points_yellow):
        """
        Save map points as text file
        :param filename: Path to save the file
        :param points_white: Points for white lanes, in the format [(x1, y1), (x2, y2), ...]
        :param points_yellow: Points for yellow lanes, in the format [(x1, y1), (x2, y2), ...]
        """
        with open(filename, 'w') as file:
            # Write white lane points
            file.write("White lane points:\n")
            for point in points_white:
                file.write(f"{point[0]}, {point[1]}\n")
            
            # Write yellow lane points
            file.write("\nYellow lane points:\n")
            for point in points_yellow:
                file.write(f"{point[0]}, {point[1]}\n")

        print(f"Points saved to {filename}")
    
    def main(self):
        rospy.spin()
        
        
if __name__ == "__main__":
    rospy.init_node("lane_detector", anonymous=True)
    ins = LaneDetector()
    ins.main()
```
