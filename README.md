# Multi-LiDAR Pointcloud Segmentation using curved-voxel clustering with Real-Time Performance

## Install

``` bash
cd ~/catkin_ws/src
git clone https://github.com/linus106-bit/Multi_CVC.git
cd ..
catkin_make --only-pkg-with-deps curved_clustering
```

### Gazebo Simulation


``` bash
roslaunch curved_clustering_bringup gazebo.launch
roslaunch curved_clustering_bringup model_main.launch
```
or (Using rosbag)
``` bash
cd ~/catkin_ws/src/curved_clustering
rosbag play -l new_test_2lidar.bag
```

### Curved-Voxel Clustering

``` bash
roslaunch curved_clustering_code cluster.launch
roslaunch curved_clustering_code merge.launch
```
