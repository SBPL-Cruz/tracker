# tracker
Detect and locate dynamic objects (on conveyor belt), Kalman filter is applied.


## Setup
Clone the following to workspace  
1.Realsense camera: https://github.com/IntelRealSense/realsense-ros  
2.Robot model: https://github.com/SBPL-Cruz/wheeled_walker  
3.This code: https://github.com/SBPL-Cruz/tracker  
```
$ cd ~/catkin_ws/
$ catkin build
```
## Launch

### 1.Synchronize computers  
```
$ sudo /etc/init.d/ntp stop
$ sudo ntpdate 192.168.11.2
$ sudo /etc/init.d/ntp start
```
### 2. Launch robot  
```
$ ssh walker2@192.168.11.2 
Password: aa
$ su 
Password: aa
$ rosrun servo_ctrl canservo_svr
```
Open a new command window  
```
$ ssh walker2@192.168.11.2 
Password: aa
$ su 
Password: aa
$ ./walker_arm_controller_rightLimb
```
Then launch model  
```
$ssh cruiser@192.168.11.123
Password: aa
$ roslaunch wheeled_walker planning_context_walker.launch 
```
### 3. Launch camera
```
$ roslaunch realsense2_camera rs_rgbd.launch camera:=head_camera
```
### 4. Launch object tracker
```
$ roslaunch object_track tracker.launch
```
There are some parameters in launch file, which can be modified if necessary.  
Open rviz, open file catkin_ws/src/wheeled_walker/config/cruzr.rviz, choose the topics of interest

## Topic
"cyl_cloud": Cylinder fitted point clouds.  
"cropped_cloud": Cropped point clouds.  
"plane_cloud": Extracted plane point clouds.  
"ar_pose_marker": Cylinder ID and coordinates.  
"visualization_markers": cylinder Marker.  
"pose": Coordinates.  
**"pose_filtered": Filtered coordinates.**

