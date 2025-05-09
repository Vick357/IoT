FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ROS2
# Create a workspace
WORKDIR /ros2_humble_ws/src

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

# Gazebo Fortress
# Set the working directory
WORKDIR /root

# Install the necessary packages
RUN apt-get update && apt-get install -y \ 
    lsb-release \ 
    wget \ 
    gnupg \
    x11-apps \
    libxext-dev \
    libxrender-dev \
    libxtst-dev 

RUN sudo apt-get install  -y ros-${ROS_DISTRO}-ros-gz

RUN apt install -y vim
RUN apt install -y htop
RUN apt install -y nvtop 

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_humble_ws/install/setup.bash" >> ~/.bashrc

# Set Gazebo models path environment variable
# see https://gazebosim.org/api/sim/8/resources.html
# Modify here to add your own models path
RUN echo "export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/ros2_humble_ws/src/" >> ~/.bashrc

# An alias to source the bashrc file
RUN  echo "alias sb='source ~/.bashrc'" >> ~/.bashrc

CMD [ "/ros2_humble_ws/src/init.sh"]
