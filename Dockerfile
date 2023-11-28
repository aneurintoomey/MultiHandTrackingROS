#Official ros-base melodic image copied from:
#https://hub.docker.com/_/ros/tags

# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-core-bionic

# Use updated ROS key (expires June 2025)
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

#Add automatic setup sourcing to bash
RUN touch /root/.bashrc && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

#Install dependencies and upgrade python (ROS melodic uses 2.7 we have to use at least 3.8 for our service)
RUN apt-get update && apt-get install -y libopencv-dev ros-${ROS_DISTRO}-cv-bridge python3-pip python3.8

#Install dependencies
RUN python3.8 -m pip install --upgrade pip
RUN python3.8 -m pip install opencv-python mediapipe PyYaml catkin_pkg rospkg

#Copy Directory
ADD . /workspaces/multi_hand_tracking_ws

#Setup Workspace (This is a bit hacky because the ros_numpy packages aren't being found. The first command installs all dependencies but an old package the second line installs the new package from it's source)
RUN sudo apt update && sudo apt install -y ros-melodic-ros-numpy
RUN rosdep install --from-paths /workspaces/multi_hand_tracking_ws/src/ --ignore-src --rosdistro melodic

#Setup package for building
RUN rm -r /workspaces/multi_hand_tracking_ws/build
RUN rm -r /workspaces/multi_hand_tracking_ws/devel
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; cd /workspaces/multi_hand_tracking_ws; catkin_make; source devel/setup.bash'