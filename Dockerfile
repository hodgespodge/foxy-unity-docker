FROM osrf/ros:foxy-desktop as builder


# Use environment variable to allow custom VNC passwords
ENV VNC_PASSWD=123456


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

#=======================================================================================
# VNC SERVER STUFF
#=======================================================================================


# for the VNC connection
EXPOSE 5900
# for the browser VNC client
EXPOSE 5901


# Make sure the dependencies are met
ENV APT_INSTALL_PRE="apt -o Acquire::ForceIPv4=true update && DEBIAN_FRONTEND=noninteractive apt -o Acquire::ForceIPv4=true install -y --no-install-recommends"
ENV APT_INSTALL_POST="&& apt clean -y && rm -rf /var/lib/apt/lists/*"
# Make sure the dependencies are met
RUN eval ${APT_INSTALL_PRE} tigervnc-standalone-server tigervnc-common fluxbox eterm xterm git net-tools python python-numpy ca-certificates scrot ${APT_INSTALL_POST}

# Install VNC. Requires net-tools, python and python-numpy
RUN git clone --branch v1.2.0 --single-branch https://github.com/novnc/noVNC.git /opt/noVNC
RUN git clone --branch v0.9.0 --single-branch https://github.com/novnc/websockify.git /opt/noVNC/utils/websockify
RUN ln -s /opt/noVNC/vnc.html /opt/noVNC/index.html

# Add menu entries to the container
RUN echo "?package(bash):needs=\"X11\" section=\"DockerCustom\" title=\"Xterm\" command=\"xterm -ls -bg black -fg white\"" >> /usr/share/menu/custom-docker && update-menus

# Set timezone to UTC
RUN ln -snf /usr/share/zoneinfo/UTC /etc/localtime && echo UTC > /etc/timezone

# Add in a health status
HEALTHCHECK --start-period=10s CMD bash -c "if [ \"`pidof -x Xtigervnc | wc -l`\" == "1" ]; then exit 0; else exit 1; fi"

# Add in non-root user
ENV UID_OF_DOCKERUSER 1000
RUN useradd -m -s /bin/bash -g users -u ${UID_OF_DOCKERUSER} dockerUser
RUN chown -R dockerUser:users /home/dockerUser && chown dockerUser:users /opt




RUN mkdir /opt/startup_scripts

USER dockerUser

# Copy various files to their respective places
COPY --chown=dockerUser:users scripts/container_startup.sh /opt/container_startup.sh
COPY --chown=dockerUser:users scripts/x11vnc_entrypoint.sh /opt/x11vnc_entrypoint.sh
# Subsequent images can put their scripts to run at startup here

RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc &&\
    echo 'echo "Sourced $ROS_DISTRO"' >> ~/.bashrc &&\
    echo 'source Robotics-Nav2-SLAM-Example/ros2_docker/colcon_ws/install/local_setup.bash' >> ~/.bashrc &&\   
    cat scripts/license.sh >> ~/.bashrc
    
RUN echo "alias rviz_example='ros2 launch unity_slam_example unity_slam_example.py' " >> ~/.bashrc &&\
    echo "alias unity_example='/opt/unity/editors/2020.3.11f1/Editor/Unity -projectPath /Robotics-Nav2-SLAM-Example/Nav2SLAMExampleProject/' " >> ~/.bashrc
    
    
ENTRYPOINT ["/opt/container_startup.sh"]


