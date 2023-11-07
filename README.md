# Plane Seg
*Plane Seg PointNet2* is a library for fitting planes to noisy depth camera data. It is developed based on (https://github.com/ori-drs/plane_seg). The main idea is to use PointNet2 for plane points classification, and then cluster the plane points to planes. 

# Using It
**python env:** This package requires python 3.7.
```
conda env create -f cupy.yml
```

**Run-Time Application:** A sample application can be run and visualized in Rviz as follows:

```
conda activate cupy
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/home/minghan/mambaforge/envs/cupy/bin/python
source devel/setup.bash 
roslaunch plane_seg_ros view_plane_seg.launch
```

**Test program:** reads example point clouds (PCDs), processes them and executes the fitting algorithm:
In the first terminal:

```
conda activate cupy
source devel/setup.bash 
roslaunch plane_seg_ros cw_test2.launch
```
In the second terminal:

```
conda activate cupy
source devel/setup.bash 
roslaunch plane_seg_ros cw_test3.launch
```
In the third terminal:

```
conda activate cupy
source devel/setup.bash 
roslaunch plane_seg_ros cw_test1.launch
```

