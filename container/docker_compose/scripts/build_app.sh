#!/bin/bash

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

# Auto-build everything in the background:
source /opt/ros/humble/setup.bash

# install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build --symlink-install --merge-install
