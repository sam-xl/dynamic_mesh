#! /bin/bash

SOURCE_WS="$DEV_WORKSPACE"

USAGE="
build.sh usage

-t <path> \
SOURCE_WS: the location of the workspace you wish to cache inside an image

"

while getopts t:h flag; do
    case "${flag}" in
    t) SOURCE_WS=${OPTARG} ;;
    h)
        echo -e "${USAGE}"
        exit 0
        ;;
    *)
        echo 'Unknown parameter' >&2
        echo 'use -h for usage information'
        exit 1
        ;;
    esac
done

echo "Target workspace: ${SOURCE_WS}"

cd "${SOURCE_WS}" || exit

source ~/.bashrc
source "${SOURCE_WS}/install/setup.bash"

# launch processes here

ros2 run dynamic_mesh pcl_buffer_node --ros-args -r input_cloud:=/scancontrol_pointcloud -r output_mesh:=/cloud_mesh -p decay_time:=1 -p do_transform:=true -p frame_id:=ur10e_base_link

# Make sure there's a valid TF between laser_frame and frame_id
