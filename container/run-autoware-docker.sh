#!/bin/bash

DOCKER_IMAGE=zenoh-autoware-20240903
DOCKER_FILE=container/Dockerfile_autoware

if [ ! "$(docker images -q ${DOCKER_IMAGE})" ]; then
    echo "${DOCKER_IMAGE} does not exist. Creating..."
    docker build --network host -f ${DOCKER_FILE} -t ${DOCKER_IMAGE} .
fi

rocker --network host --nvidia --privileged --x11 --user --name autoware --volume $(pwd):$HOME/autoware_carla_launch -- ${DOCKER_IMAGE}

