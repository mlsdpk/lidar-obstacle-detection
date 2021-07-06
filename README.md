# lidar-obstacle-detection

3D object detection pipeline using Lidar. This project is based on ROS2 and SVL simulator.

## Dependencies

- SVL Simulator ([click here for installation instructions](https://www.svlsimulator.com/docs/installation-guide/installing-simulator/))

- ROS 2 Foxy Fitzroy ([click here for installation instructions](https://docs.ros.org/en/foxy/Installation.html))

- SVL Simulator ROS 2 Bridge ([click here for installation instructions](https://www.svlsimulator.com/docs/system-under-test/ros2-bridge/))

- PCL 1.11.1 ([click here for installation instructions](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html))

## Build Instructions

First, source the setup.bash file in the ROS 2 build workspace.

```
source ~/ros2_ws/install/setup.bash 
```

Clone this workspace to the desire directory.

```
git clone https://github.com/mlsdpk/lidar-obstacle-detection
```

Build the workspace.

```
cd lidar-obstacle-detection
colcon build
```

## Usage

> Before launching any launch files, make sure that SVL Simulator is launched. This project assumes using Lexus2016RXHybrid vehicle with Autoware.Auto Sensor Configurations. You can find more information [here](https://www.svlsimulator.com/docs/system-under-test/autoware-auto-instructions/) for running the SVL Simulator with Lexus2016RXHybrid vehicle.

### 1. Running the ROS2 LGSVL Bridge

In order to connect SVL Simulator with ROS, we need to use ROS2 LGSVL Bridge. Follow [this guide](https://www.svlsimulator.com/docs/system-under-test/ros2-bridge/) for installing ros2 lgsvl bridge.

```
source $(path/to/bridge/repository)/install/setup.bash
lgsvl_bridge
```

### 2. Launching Lexus2016RXHybrid vehicle urdf description

This package will launch all the necessary tf information for Lexus2016RXHybrid vehicle. This is a modified version of lexus_rx_450h_description package from [Autoware.Auto gitlab repo](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/urdf/lexus_rx_450h_description). You will see the rviz2 window pop up together with vehicle model.

```
source $(path/to/this/repository)/install/setup.bash
ros2 launch lexus_rx_450h_description lexus_rx_450h_visualisation.launch.py 
```

### 3. Launching pointcloud fusion node

This package will fuse incoming pointcloud data from two lidars and output as one single raw pointcloud data. Transformations of pointclouds from both lidars's frame to baselink of the vehicle is also taken care in this package.

```
source $(path/to/this/repository)/install/setup.bash
ros2 launch pointcloud_fusion pointcloud_fusion.launch.py
```

### 4. Launching filtering node

This project implements the voxel grid filtering for downsampling the raw fused pointcloud data by utilizing the popular PCL library.

```
source $(path/to/this/repository)/install/setup.bash
ros2 launch voxelgrid_filtering voxelgrid_filtering.launch.py
```

### 5. Launching segmentation node

In order to detect obstacles in the scene, removing unnecessary points from the pointcloud data is essential for the clustering algorithms. This package uses RANSAC algorithm to segment the ground points and the obstacle points from the fused filtered pointcloud data.

```
source $(path/to/this/repository)/install/setup.bash
ros2 launch planar_segmentation planar_segmentation.launch.py
```