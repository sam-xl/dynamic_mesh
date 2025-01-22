# dynamic_mesh
A ROS package that creates a filtered moving mesh from an incoming pointcloud topic. 

Subscribed topics:
- input_cloud (sensor_msgs/PointCloud2): The pointcloud from which the mesh will be made

Published topics:
- output_mesh (sensor_msgs/PointCloud2): The output mesh constructed from the pointcloud

Parameters:
- pcl_frequency (int, 400): The **expected** frequency in Hz at which the input_cloud topic is coming
- decay_time (int, 1): The number of seconds to keep buffering the pointcloud  
- frame_id (string, "world"): The frame in which the mesh topic will be published

## example usage
```bash
ros2 run dynamic_mesh pcl_buffer_node --ros-args -r input_cloud:=/transformed_pointcloud -r output_mesh:=/cloud_mesh -p decay_time:=1
```

<img src="demo.gif">