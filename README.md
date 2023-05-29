# pointcloud-ros-open3d-interface
A pythonic interface to process point cloud from ROS (upgraded for OPEN3D 0.17.0)

This library allows to convert point cloud from ROS messages to OPEN3D data (and viceversa).

NOTE: tested on ROS Noetic (Ubuntu 20.04).

# HOW TO INSTALL
- clone repository
- install pypcd following instruction here (https://github.com/dimatura/pypcd, ROS installation)
- just catkin_make

# HOW TO USE
- source the workspace.
- run "processing.py" inside "src/open3d_conversions/src/open3d_conversions".

NOTE: plane segmentation is implemented (processing.py), you can substitute this part with your OPEN3D processing.
