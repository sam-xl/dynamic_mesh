#! /bin/bash
set -e

export CONTAINER_SCRIPTS_DIR=${DEV_WORKSPACE}/src/dynamic_mesh/container/docker_compose/scripts

# ${CONTAINER_SCRIPTS_DIR}/bringup_container.sh
echo "Building app..." && ${CONTAINER_SCRIPTS_DIR}/build_app.sh
echo "Running app..." && ${CONTAINER_SCRIPTS_DIR}/run_app.sh
# ${CONTAINER_SCRIPTS_DIR}/dev_app.sh
