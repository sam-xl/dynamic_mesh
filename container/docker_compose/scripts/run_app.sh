#! /bin/bash

cd "${DEV_WORKSPACE}" || exit

source ~/.bashrc
source install/setup.bash

# launch processes here

ros2 run dynamic_mesh pcl_buffer_node --ros-args -r input_cloud:=/scancontrol_pointcloud -r output_mesh:=/cloud_mesh -p decay_time:=1 -p do_transform:=true -p frame_id:=ur10e_base_link

# Make sure there's a valid TF between laser_frame and frame_id
