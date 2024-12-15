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

### 2. 
