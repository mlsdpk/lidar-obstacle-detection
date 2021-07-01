# lidar-obstacle-detection

3D object detection pipeline using Lidar. This project is based on ROS2 and SVL simulator.

## Dependencies

- SVL Simulator ([click here for installation instructions](https://www.svlsimulator.com/docs/installation-guide/installing-simulator/))

- ROS 2 Foxy Fitzroy ([click here for installation instructions](https://docs.ros.org/en/foxy/Installation.html))

- SVL Simulator ROS 2 Bridge ([click here for installation instructions](https://www.svlsimulator.com/docs/system-under-test/ros2-bridge/))

## Build Instructions

First, source the setup.bash file in the ROS 2 build workspace.

```
source ~/ros2_ws/install/setup.bash 
```

Close this workspace to the desire directory.

```
git clone https://github.com/mlsdpk/lidar-obstacle-detection
```

Build the workspace.

```
cd lidar-obstacle-detection
colcon build
```