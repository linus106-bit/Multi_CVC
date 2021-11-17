# Multi-LiDAR Pointcloud Segmentation using curvex-voxel clustering with Real-Time Performance

## Install

``` bash
cd ~/catkin_ws/src
git clone https://github.com/linus106-bit\convex_clustering.git
cd ..
catkin_make --only-pkg-with-deps convex_clustering
```

### Gazebo Simulation


``` bash
roslaunch convex_clustering_bringup gazebo.launch
roslaunch convex_clustering_bringup model_main.launch
```
or (Using rosbag)
``` bash
cd ~/catkin_ws/src/convex_clustering
rosbag play -l new_test_2lidar.bag
```

### Curved-Voxel Clustering

``` bash
roslaunch convex_clustering_code cluster.launch
roslaunch convex_clustering_code merge.launch
```
