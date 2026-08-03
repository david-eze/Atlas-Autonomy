# ROS 2 Jazzy + Gazebo Harmonic development image for the robot_autonomy project.
#
# Build:
#   docker build -t robot_autonomy:jazzy -f docker/Dockerfile .
#
# Run (Linux, with GUI):
#   docker compose -f docker/docker-compose.yml up
#
# Run (headless benchmarks):
#   docker compose -f docker/docker-compose.yml run --rm robot bash
#
# NOTE: Gazebo Harmonic ships with Ubuntu 24.04 (noble).

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Core tooling and Python data-science stack.
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release wget gnupg curl git \
    python3-pip python3-vcstool python3-colcon-common-extensions python3-rosdep \
    python3-pytest python3-pytest-cov \
    python3-numpy python3-scipy python3-matplotlib python3-pil python3-opencv \
    python3-yaml python3-requests python3-websockets python3-flask python3-flask-cors \
    python3-sqlalchemy python3-psutil \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 simulation, navigation, SLAM, and tooling packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros2-control ros-jazzy-gazebo-plugins \
    ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs ros-jazzy-nav2-core ros-jazzy-nav2-util \
    ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-nav-core ros-jazzy-nav2-planner \
    ros-jazzy-nav2-controller ros-jazzy-nav2-recoveries ros-jazzy-nav2-behavior-tree \
    ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-nav2-rotation-shim-controller ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-collision-monitor \
    ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-slam-toolbox ros-jazzy-rtabmap ros-jazzy-rtabmap-ros \
    ros-jazzy-robot-localization ros-jazzy-tf2-tools ros-jazzy-xacro \
    ros-jazzy-urdf ros-jazzy-urdfdom ros-jazzy-angles ros-jazzy-diagnostic-updater \
    ros-jazzy-ros2bag ros-jazzy-rosbag2-storage-default-plugins ros-jazzy-rosbag2-transport \
    ros-jazzy-rviz2 ros-jazzy-rqt-graph ros-jazzy-rqt-tf-tree \
    ros-jazzy-cv-bridge ros-jazzy-vision-msgs ros-jazzy-vision-opencv \
    ros-jazzy-image-transport ros-jazzy-image-transport-plugins ros-jazzy-camera-info-manager \
    ros-jazzy-diagnostic-msgs ros-jazzy-nav-msgs ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs ros-jazzy-std-srvs ros-jazzy-action-msgs ros-jazzy-builtin-interfaces \
    ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime \
    ros-jazzy-ament-cmake-auto ros-jazzy-ament-cmake-gmock ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common ros-jazzy-ament-cmake-cppcheck ros-jazzy-ament-cmake-cpplint \
    ros-jazzy-ament-cmake-flake8 ros-jazzy-ament-cmake-pep257 ros-jazzy-ament-cmake-uncrustify \
    ros-jazzy-ament-cmake-xmllint ros-jazzy-ament-cmake-pytest ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-ros ros-jazzy-ament-index-python ros-jazzy-launch-testing \
    ros-jazzy-launch-testing-ament-cmake ros-jazzy-ros-testing ros-jazzy-geometry2 \
    && rm -rf /var/lib/apt/lists/*

# Python ML dependencies for the perceptual layer.
RUN pip3 install --no-cache-dir \
    ultralytics supervision onnxruntime torchvision pandas seaborn \
    imageio imageio-ffmpeg tqdm pyyaml websockets fastapi uvicorn pydantic

# Create the workspace.
WORKDIR /workspace
RUN mkdir -p /workspace/src

# Source ROS 2 on every shell.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/install/setup.bash" >> /root/.bashrc
RUN echo "alias build='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]