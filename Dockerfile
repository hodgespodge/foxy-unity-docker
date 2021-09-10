FROM osrf/ros:foxy-desktop as builder

ENV UNITY_PATH="/opt/unity"
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO=foxy

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Global dependencies
RUN apt-get -q update \
    && apt-get -q install -y --no-install-recommends apt-utils \
    && apt-get -q install -y --no-install-recommends --allow-downgrades \
    ca-certificates \
    libasound2 \
    libc6-dev \
    libcap2 \
    libgconf-2-4 \
    libglu1 \
    libgtk-3-0 \
    libncurses5 \
    libnotify4 \
    libnss3 \
    libxtst6 \
    libxss1 \
    cpio \
    lsb-release \
    xvfb \
    xz-utils \
    vim\
    libcanberra-gtk-module\
    && apt-get clean

RUN apt-get -q update \
    && apt-get -q install -y --no-install-recommends --allow-downgrades zenity \
    atop \
    curl \
    git \
    git-lfs \
    openssh-client \
    wget \
    && git lfs install --system --skip-repo \
    && apt-get clean

#=======================================================================================
# Download and Install UnityHub
#=======================================================================================

# Download & extract AppImage
RUN wget --no-verbose -O /tmp/UnityHub.AppImage "https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage" \
    && chmod +x /tmp/UnityHub.AppImage \
    && cd /tmp \
    && /tmp/UnityHub.AppImage --appimage-extract \
    && cp -R /tmp/squashfs-root/* / \
    && rm -rf /tmp/squashfs-root /tmp/UnityHub.AppImage \
    && mkdir -p "$UNITY_PATH" \
    && mv /AppRun /opt/unity/UnityHub

# Alias to "unity-hub" with default params
RUN echo '#!/bin/bash\nxvfb-run -ae /dev/stdout /opt/unity/UnityHub --no-sandbox --headless "$@"' > /usr/bin/unity-hub \
    && chmod +x /usr/bin/unity-hub

# Accept
RUN mkdir -p "/root/.config/Unity Hub" \
    && touch "/root/.config/Unity Hub/eulaAccepted"

# Configure
RUN mkdir -p "${UNITY_PATH}/editors" \
    && unity-hub install-path --set "${UNITY_PATH}/editors/" \
    && find /tmp -mindepth 1 -delete

#=======================================================================================
# Download and Build example unity project
#=======================================================================================

RUN apt-get update &&\
    git clone https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example.git
    
COPY .gitmodules Robotics-Nav2-SLAM-Example/.gitmodules

RUN cd Robotics-Nav2-SLAM-Example &&\
    git submodule update --init --recursive

RUN cd /Robotics-Nav2-SLAM-Example/ros2_docker/colcon_ws &&\
    rosdep install --from-paths src --ignore-src -r -y &&\
    colcon build

RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc &&\
    echo 'echo "Sourced $ROS_DISTRO"' >> ~/.bashrc &&\
    echo 'source Robotics-Nav2-SLAM-Example/ros2_docker/colcon_ws/install/local_setup.bash' >> ~/.bashrc 

#=======================================================================================
# Install Robotics-Nav2-SLAM-Example Unity Version
#
# Note: version and changelog must BOTH be set when specifying version
#=======================================================================================
RUN /usr/bin/xvfb-run -ae /dev/stdout /opt/unity/UnityHub --no-sandbox --headless install --version 2020.3.11f1 -c 99c7afb366b3

#=======================================================================================
# [2020.x/2020.2.0/2020.2.1-webgl] Support GZip compression: https://github.com/game-ci/docker/issues/75
#=======================================================================================
RUN echo "$version-$module" | grep -q -v '^\(2020.1\|2020.2.0f\|2020.2.1f\).*-webgl' \
  && exit 0 \
  || echo 'export GZIP=-f' >> /usr/bin/unity-editor.d/webgl-2020.1-2.sh

#=======================================================================================
# [webgl] Support audio using ffmpeg (~99MB)
#=======================================================================================
RUN echo "$module" | grep -q -v 'webgl' \
    && exit 0 \
    || : \
    && apt-get update \
    && apt-get -q install -y --no-install-recommends --allow-downgrades \
    ffmpeg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

COPY scripts/ scripts
RUN cat scripts/license.sh >> ~/.bashrc