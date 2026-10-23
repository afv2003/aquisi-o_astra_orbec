ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-usb-cam \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN printf '%s\n' '#!/bin/bash' 'set -e' \
    'source /opt/ros/jazzy/setup.bash' 'exec "$@"' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
