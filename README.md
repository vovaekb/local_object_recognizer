# local_object_recognizer

ROS package implementing local pipeline for 3D object recognition from the paper A. Aldoma et al., "Tutorial: Point Cloud Library: Three-Dimensional Object Recognition and 6 DOF Pose Estimation," in IEEE Robotics & Automation Magazine, vol. 19, no. 3, pp. 80-91, Sept. 2012.

## Using package
Compiling catkin workspase:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Running recognition node:

```
rosrun local_object_recognizer recognizer _models_dir:=<models_dir> _gt_dir:=<ground_truth_dir> <options>
```

ground_truth_dir - Path to ground truth files.

Options are:

* _h=true:                                 Show help
* _scene_pcd:=<cloud.pcd>                  Scene point cloud (normal mode only)
* _model:                                  Model name to be used instead of reading models file
* _ransac:=true                            Use RANSAC in ICP step
* _rf_radius:=<rf_radius>                  RF radius
* _cg_size:=<cg_size>                      Correspondence grouping size
* _cg_thresh:=<cg_thresh>                  Correspondence grouping threshold
* _vis:=true                               Visualize recognition results
