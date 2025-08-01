#!/bin/bash

DOCKER_IMAGE=zenoh-carla-bridge-20240903
DOCKER_FILE=container/Dockerfile_carla_bridge

if [ ! "$(docker images -q ${DOCKER_IMAGE})" ]; then
    echo "${DOCKER_IMAGE} does not exist. Creating..."
    docker build --network host -f ${DOCKER_FILE} -t ${DOCKER_IMAGE} .
fi

rocker --nvidia --network host --privileged --x11 --user --name autoware_bridge --volume $(pwd):$HOME/autoware_carla_launch -- ${DOCKER_IMAGE}

