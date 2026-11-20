# Reproducible environment: ROS 2 Jazzy + Gazebo Harmonic + Nav2 + this workspace.
#   docker build -f docker/Dockerfile -t amr-nav:jazzy .
#   xhost +local:docker
#   docker run -it --rm --net=host --gpus all -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix amr-nav:jazzy \
#     ros2 launch amr_bringup bringup.launch.py
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    GZ_IP=127.0.0.1 \
    QT_ACCESSIBILITY=0

WORKDIR /ws
COPY src ./src

RUN apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
 && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/jazzy/setup.sh \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
