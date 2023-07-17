FROM ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y libv4l-dev v4l-utils pip libopencv-dev && \
    pip3 install Flask

RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime

COPY ros2_ws /ros2_ws

RUN /bin/bash -c '/ros_entrypoint.sh && source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build'

CMD ["/bin/bash"]
