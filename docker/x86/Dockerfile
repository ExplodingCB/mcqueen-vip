# Workstation image: ROS 2 Jazzy on Ubuntu 24.04 with the Python tooling, the C
# toolchain for the gateway firmware and control core host builds, and the
# simulator. CUDA for training is layered on separately when a GPU is present.
#
#   docker build -t mcqueen-x86 -f docker/x86/Dockerfile .
#   docker run --rm -it -v "$PWD":/ws -w /ws mcqueen-x86
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake clang-format git python3-pip python3-venv \
    python3-numpy python3-scipy python3-yaml python3-pytest \
    ros-jazzy-rosbag2-storage-mcap ros-jazzy-foxglove-bridge \
    ros-jazzy-geometry-msgs ros-jazzy-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages --no-cache-dir cantools ruff pre-commit

WORKDIR /ws
COPY docker/x86/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
