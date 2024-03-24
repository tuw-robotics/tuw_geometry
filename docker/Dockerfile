ARG ROS_DISTRO=humble
ARG UBUNTU_DISTRO=jammy
ARG BASE_CONTAINER=ros:${ROS_DISTRO}-ros-base
ARG ROOT_CONTAINER=$BASE_CONTAINER 
FROM $ROOT_CONTAINER

ENV MY_WS=/opt/tuw

RUN apt update
RUN apt-get install -y ros-${ROS_DISTRO}-ament-cmake-* ros-${ROS_DISTRO}-ament-lint-*  ros-${ROS_DISTRO}-sensor-msgs
RUN apt install -y libopencv-dev 


RUN mkdir -p ${MY_WS}/src
COPY ./tuw_geometry ${MY_WS}/src/tuw_geometry
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash;  cd ${MY_WS}; rosdep install --from-paths src --ignore-src -r -y'  
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash;  cd ${MY_WS}; colcon build --symlink-install'  
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash;  . ${MY_WS}/install/setup.bash; ros2 run tuw_geometry test_geometry'  


RUN echo "colcon build --symlink-install" >> ~/.bash_history
RUN echo "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --cmake-args -DBUILD_TESTING=true" >> ~/.bash_history
RUN echo "colcon test --ctest-args tests" >> ~/.bash_history
RUN echo "ros2 run tuw_geometry test_geometry" >> ~/.bash_history
RUN echo "source ./install/setup.bash" >> ~/.bash_history

# setup entrypoint
COPY ./tuw_geometry/docker/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]