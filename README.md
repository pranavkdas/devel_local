# Latest version: 
### final.cpp - Object oriented method of Version 1 with following improvements :
- Updated scan matching algorithm
- Low latency 
- Included IMU correction
### Requirements :
- ROS cpp
- pcl
- libpointmatcher

### How to use
1. Clone the repo
```
git clone https://github.com/pranavkdas/devel_local.git
```
2. Go to src/pcl_publish.cpp and change the directory of map and scan cloud.
```
pcl::io::loadPCDFile ("/home/<your_pc_name>/<your_workspace>/src/relocalisation/data/kitti2.pcd", MapCloud);
pcl::io::loadPCDFile ("/home/<your_pc_name>/<your_workspace>/src/relocalisation/data/area.pcd", ScanCloud);
```
3. Build the package
```
catkin_make -j1
```
3. Launch the package using the following command
```
roslaunch relocalisation run.launch
```

### The main file to edit for modifications is : 
`final.cpp`

# Appendix: Version 1:
### How it works :
![idea](https://github.com/pranavkdas/Relocalisation/blob/main/relocalisation/idea.png)

