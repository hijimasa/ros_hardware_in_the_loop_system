#!/bin/bash
set -e

file_dir=$(cd "$(dirname "$0")" && pwd)

# Get parameter from system
user=$(id -un)
group=$(id -gn)
uid=$(id -u)
gid=$(id -g)

docker build -t ${user}/ros-jazzy-hils \
    --build-arg USER=ubuntu \
    --build-arg UID=${uid} \
    --build-arg GROUP=${group} \
    --build-arg GID=${gid} \
    ${file_dir}
