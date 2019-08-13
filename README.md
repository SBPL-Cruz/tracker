# tracker
Detect and locate objects(cylinder)

## Running
```
$ mkdir -p ~/catkin_ws/src   # Replace `catkin_ws` with the name of your workspace
$ cd ~/catkin_ws/
$ catkin_make
$ rosrun catkin_ws tracker
```
## Topic

"cyl_cloud": Cylinder fitted point clouds.  
"cropped_cloud": Cropped point clouds.  
"plane_cloud": Extracted plane point clouds.  
**"ar_pose_marker": Cylinder ID and coordinates.**
"pose": Coordinates.  
"pose_filtered": Filtered coordinates.  
