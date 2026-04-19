#!/bin/bash
set -e

file_dir=$(cd "$(dirname "$0")" && pwd)

# Get parameter from system
user=$(id -un)
group=$(id -gn)
uid=$(id -u)
gid=$(id -g)

# Forward proxy environment variables from the host (if set).
# Set HTTP_PROXY / HTTPS_PROXY in your shell before running this script
# if you are behind a corporate proxy.
PROXY_ARGS=""
for var in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
  val="${!var}"
  [ -n "$val" ] && PROXY_ARGS="${PROXY_ARGS} --build-arg ${var}=${val}"
done

docker build -t ${user}/ros-jazzy-hils \
    --build-arg USER=ubuntu \
    --build-arg UID=${uid} \
    --build-arg GROUP=${group} \
    --build-arg GID=${gid} \
    ${PROXY_ARGS} \
    ${file_dir}
