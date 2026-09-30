# ROS 2 Lyrical + the Nav2 packages ugv_navigation needs, for building and testing
# without a native Lyrical install (CLAUDE.md: target distro is Lyrical Luth).
#
#   docker build -t ugv-lyrical-nav2 -f src/ugv_navigation/docker/lyrical-nav2.Dockerfile \
#     src/ugv_navigation/docker
#   src/ugv_navigation/docker/test_in_lyrical.sh src/ugv_navigation
#
# dist-upgrade is required: the ros:lyrical-ros-base image lags the apt repo, and
# newer Nav2 on top of its older core libs crashes at startup with
# "undefined symbol: has_buffer_fields_std_msgs__msg__Header".
FROM ros:lyrical-ros-base

RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y --no-install-recommends \
    ros-lyrical-nav2-planner ros-lyrical-nav2-smac-planner \
    ros-lyrical-nav2-controller ros-lyrical-nav2-regulated-pure-pursuit-controller \
    ros-lyrical-nav2-behaviors ros-lyrical-nav2-bt-navigator ros-lyrical-nav2-behavior-tree \
    ros-lyrical-nav2-lifecycle-manager ros-lyrical-nav2-msgs \
    ros-lyrical-tf2-ros \
    ros-lyrical-ament-lint-auto ros-lyrical-ament-lint-common \
    ros-lyrical-ament-cmake-pytest ros-lyrical-ament-cmake-ros \
    python3-pytest python3-yaml python3-numpy python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*
