# dynamic_mesh
A ROS package that creates a filtered moving mesh from an incoming pointcloud topic. 

<img src="demo.gif">

Author, Maintainer: Nikhil Sethi

## Build
```bash
# clone repo
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/sam-xl/dynamic_mesh.git

# install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build --symlink-install --merge-install
```
## Run
```bash
source install/setup.bash
ros2 run dynamic_mesh pcl_buffer_node --ros-args -r input_cloud:=/transformed_pointcloud -r output_mesh:=/cloud_mesh -p decay_time:=1
```

## Node
Subscribed topics:
- `input_cloud` (sensor_msgs/PointCloud2): The pointcloud from which the mesh will be made

Published topics:
- `output_mesh` (sensor_msgs/PointCloud2): The output mesh constructed from the pointcloud

Parameters:
- `pcl_frequency` (int, 400): The **expected** frequency in Hz at which the input_cloud topic is coming
- `decay_time` (int, 1): The number of seconds to keep buffering the pointcloud  
- `frame_id` (string, "world"): The frame in which the mesh topic will be published and transformed if `do_transform` is true
- `do_transform` (bool, false): Transform the pointcloud into `frame_id` before creating the mesh. This is often needed because pointclouds need to be in a fixed frame to create a mesh 

**Details**: The node works by accepting pointclouds on a topic, filtering them using voxel grids, and buffering them up into a FIFO buffer. The buffer length is determined by incoming frequncy and the decay time. 

The process is memory efficient because, each pointcloud is filtered before being pushed into the buffer, so the overall global buffer size remains small. 