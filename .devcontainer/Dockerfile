FROM osrf/ros:foxy-desktop

# Install base system dependencies
RUN apt update && apt install wget software-properties-common -y
RUN apt update && apt install ros-foxy-maliput-full -y

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Install webots
RUN wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
RUN apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
RUN apt update && apt install webots -y

CMD ["/bin/bash"]
