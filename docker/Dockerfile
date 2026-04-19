FROM ros:jazzy-ros-base-noble

# Arguments
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID=${UID}
ARG SHELL=/bin/bash

ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ──
RUN apt-get update -y -qq && \
    apt-get upgrade -y -qq --no-install-recommends && \
    apt-get install -y -qq \
      vim v4l-utils \
      software-properties-common \
      git wget curl unzip \
      python3-pip \
      libopencv-dev libeigen3-dev \
      libpcap-dev libyaml-cpp-dev && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

# ── ROS packages ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-cv-bridge \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-usb-cam \
    ros-jazzy-v4l2-camera \
    ros-jazzy-nmea-navsat-driver \
    ros-jazzy-teleop-twist-keyboard \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# ── Python dependencies for hils_bridge ──
RUN pip3 install --break-system-packages \
    pyserial \
    numpy

# ── Livox SDK2 (required by livox_ros_driver2) ──
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir Livox-SDK2/build && cd Livox-SDK2/build && \
    cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
          -DCMAKE_CXX_STANDARD=11 -DCMAKE_CXX_STANDARD_REQUIRED=ON \
          -DCMAKE_CXX_FLAGS="-include cstdint" \
          .. && \
    make -j$(nproc) && \
    make install && \
    cd / && rm -rf Livox-SDK2

# ── Velodyne driver (built from source for jazzy) ──
# Use --merge-install so install/ has a flat structure (lib/, share/, include/)
# that merges cleanly into /opt/ros/jazzy without creating nested setup.bash
# files that would break AMENT_PREFIX_PATH.
RUN mkdir -p /tmp/velodyne_ws/src && \
    cd /tmp/velodyne_ws/src && \
    git clone https://github.com/ros-drivers/velodyne.git -b ros2 && \
    cd /tmp/velodyne_ws && \
    . /opt/ros/jazzy/setup.sh && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r install/include /opt/ros/jazzy/ 2>/dev/null || true && \
    cp -r install/lib /opt/ros/jazzy/ && \
    cp -r install/share /opt/ros/jazzy/ && \
    rm -rf /tmp/velodyne_ws

RUN ldconfig

# ── Nvidia container runtime (optional) ──
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# ── User setup ──
RUN set -eux; \
  if getent passwd "${UID}" >/dev/null; then \
    base_user="$(getent passwd "${UID}" | cut -d: -f1)"; \
    if [ "${base_user}" != "${USER}" ]; then \
      useradd -o -u "${UID}" -g "${GID}" -M -d "/home/${base_user}" -s "${SHELL}" "${USER}" || true; \
    fi; \
  else \
    getent group "${GID}" >/dev/null || groupadd -g "${GID}" "${USER}"; \
    useradd -m -u "${UID}" -g "${GID}" -s "${SHELL}" "${USER}"; \
  fi; \
  mkdir -p /etc/sudoers.d; \
  echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USER}; \
  chmod 0440 /etc/sudoers.d/${USER}
RUN set -eux; \
  base_user="$(getent passwd "${UID}" | cut -d: -f1)"; \
  [ -e "/home/${USER}" ] || ln -s "/home/${base_user}" "/home/${USER}"

USER ${USER}
ENV USER=${USER}

WORKDIR /home/${USER}/colcon_ws

# ── Pre-clone sensor drivers into workspace ──
RUN mkdir -p src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo '[ -f ~/colcon_ws/install/setup.bash ] && source ~/colcon_ws/install/setup.bash' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
