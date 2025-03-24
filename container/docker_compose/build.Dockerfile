###########################################
# docker build
#   -t user/repo:tag                # assign user, repo and tag. if tag is omitted, will default to 'latest'
#   --rm                            # (optional) clean up intermediate images
#   --build-context name=location
#   --build-arg name=value
#   -f Dockerfilename .             # build from file with context (. is current pwd)
#
# docker build -t erf2025.demo/dynamic_mesh_gen  --build-arg DYNAMIC_MESH_TARGET_WS=/workspaces/dynamic_mesh_ws --build-arg DYNAMIC_MESH_SRC_SUBDIR=src/dynamic_mesh -f build.Dockerfile .
#
###########################################
# docker push user/repo:tag
###########################################

# hadolint global ignore=DL3004

FROM scancontrol:latest

ARG DYNAMIC_MESH_TARGET_WS="/CHANGEME_ws"
ARG DYNAMIC_MESH_SRC_SUBDIR="src/dynamic_mesh"

USER ros

RUN sudo mkdir -p "${DYNAMIC_MESH_TARGET_WS}/${DYNAMIC_MESH_SRC_SUBDIR}" && sudo chown -R ros:ros "${DYNAMIC_MESH_TARGET_WS}"

WORKDIR ${DYNAMIC_MESH_TARGET_WS}/${DYNAMIC_MESH_SRC_SUBDIR}

COPY ${DYNAMIC_MESH_SRC_SUBDIR} ${DYNAMIC_MESH_TARGET_WS}/${DYNAMIC_MESH_SRC_SUBDIR}

WORKDIR ${DYNAMIC_MESH_TARGET_WS}

# Remember to source ROS here if the script requires ROS variables!
RUN [ "/bin/bash", "-c", "source /opt/ros/humble/setup.sh && ${DYNAMIC_MESH_TARGET_WS}/${DYNAMIC_MESH_SRC_SUBDIR}/container/docker_compose/scripts/build_app.sh -t ${DYNAMIC_MESH_TARGET_WS}" ]

ENTRYPOINT [ "/bin/bash", "-c", "tail -f /dev/null" ]
