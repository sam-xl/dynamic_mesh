#! /bin/bash

cd "${DEV_WORKSPACE}" || exit

# Auto-build everything in the background:
# export ROSDEP_SKIP_KEYS=snp_msgs 
# "${DEV_WORKSPACE}"/src/dynamic_mesh/setup.sh -t "${DEV_WORKSPACE}"

source /opt/ros/humble/setup.bash

# install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build --symlink-install --merge-install
