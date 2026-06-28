# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# Updated to build the local ORB_SLAM2 workspace, including benchmark instrumentation.

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/root
ENV ROS_DISTRO=jazzy
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ORBSLAM_WS=/workspace

WORKDIR ${HOME}
SHELL ["/bin/bash", "-c"]

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics

RUN apt-get update && apt-get install -y \
    build-essential \
    ca-certificates \
    clang-format \
    cmake \
    curl \
    git \
    gnupg \
    ipython3 \
    locales \
    nano \
    python3-dev \
    python3-matplotlib \
    python3-numpy \
    python3-pip \
    python3-yaml \
    software-properties-common \
    vim \
    wget && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb && \
    apt-get update

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools \
    ros-jazzy-desktop && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-coverage \
    python3-flake8 \
    python3-pip \
    python3-pluggy \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-rospkg-modules \
    python3-setuptools \
    python3-wheel && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages \
    evo \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-runner

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update

RUN apt-get update && apt-get install -y \
    libavcodec-dev \
    libavformat-dev \
    libboost-all-dev \
    libboost-system-dev \
    libcanberra-gtk-module \
    libdc1394-dev \
    libeigen3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libgtk2.0-dev \
    libjpeg-dev \
    libpng-dev \
    libswscale-dev \
    libtbb-dev \
    libtbbmalloc2 \
    libtiff-dev \
    pkg-config && \
    rm -rf /var/lib/apt/lists/*

RUN git clone -b 3.4 https://github.com/opencv/opencv.git ${HOME}/opencv && \
    mkdir -p ${HOME}/opencv/build

WORKDIR ${HOME}/opencv/build
COPY docker/scripts/build_opencv.sh ${HOME}/scripts/build_opencv.sh
RUN chmod +x ${HOME}/scripts/build_opencv.sh && \
    bash ${HOME}/scripts/build_opencv.sh && \
    ldconfig

WORKDIR ${ORBSLAM_WS}
COPY . ${ORBSLAM_WS}

# Keep the local workspace buildable on Jazzy and make wrapper linking explicit.
RUN sed -i 's#<cv_bridge/cv_bridge.h>#<cv_bridge/cv_bridge.hpp>#g' \
      ros2-ORB_SLAM2/src/monocular/monocular-slam-node.hpp \
      ros2-ORB_SLAM2/src/rgbd/rgbd.cpp && \
    sed -i '/target_link_libraries(rgbd/,/)/ { /g2o/d; /DBoW2/d; }' ros2-ORB_SLAM2/CMakeLists.txt && \
    sed -i '/target_link_libraries(stereo/,/)/ { /g2o/d; /DBoW2/d; }' ros2-ORB_SLAM2/CMakeLists.txt && \
    perl -0pi -e 's/target_link_libraries\\(rgbd\\n/target_link_libraries(rgbd\\n  g2o\\n  DBoW2\\n/' ros2-ORB_SLAM2/CMakeLists.txt && \
    perl -0pi -e 's/target_link_libraries\\(stereo\\n/target_link_libraries(stereo\\n  g2o\\n  DBoW2\\n/' ros2-ORB_SLAM2/CMakeLists.txt

RUN /bin/bash -lc '\
    set -euo pipefail; \
    if [ ! -f Vocabulary/ORBvoc.txt ] && [ -f Vocabulary/ORBvoc.txt.tar.gz ]; then tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary; fi; \
    if [ ! -f Vocabulary/ORBvoc.txt ] && [ -f Vocabulary/ORBvoc.txt.tar ]; then tar -xf Vocabulary/ORBvoc.txt.tar -C Vocabulary; fi'

# Build and install the local ORB-SLAM2 core with the benchmark instrumentation.
RUN /bin/bash -lc '\
    set -euo pipefail; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    cmake -S ${ORBSLAM_WS} -B ${ORBSLAM_WS}/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
      -DCMAKE_CXX_STANDARD_LIBRARIES="-lboost_system"; \
    cmake --build ${ORBSLAM_WS}/build -j"$(nproc)"; \
    cmake --install ${ORBSLAM_WS}/build; \
    cp ${ORBSLAM_WS}/lib/libDBoW2.so /usr/local/lib; \
    cp ${ORBSLAM_WS}/Thirdparty/g2o/lib/libg2o.so /usr/local/lib; \
    ldconfig'

RUN mkdir -p ${HOME}/ws/src
WORKDIR ${HOME}/ws

RUN git clone --branch 4.1.0 --depth 1 https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \
    git clone --branch jazzy --depth 1 https://github.com/ros-perception/image_common.git src/image_common && \
    git clone --branch jazzy --depth 1 https://github.com/ros-perception/image_transport_plugins.git src/image_transport_plugins && \
    git clone --branch jazzy --depth 1 https://github.com/ros2/message_filters.git src/message_filters && \
    ln -s ${ORBSLAM_WS}/ros2-ORB_SLAM2 src/orbslam

RUN /bin/bash -lc '\
    set -euo pipefail; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys "python3-rospkg"'

# Build the ROS2 wrapper in the image so start_container/enter_container are ready to use.
RUN /bin/bash -lc '\
    set -euo pipefail; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    export LIBRARY_PATH=/usr/local/lib:${ORBSLAM_WS}/lib:${LIBRARY_PATH:-}; \
    export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}; \
    colcon build --base-paths ${HOME}/ws/src --packages-select orbslam --build-base ${HOME}/ws/build --install-base ${HOME}/ws/install; \
    ldconfig'

WORKDIR ${ORBSLAM_WS}
CMD ["/bin/bash", "-lc", "source /opt/ros/jazzy/setup.bash && [ -f /root/ws/install/setup.bash ] && source /root/ws/install/setup.bash; exec /bin/bash"]
