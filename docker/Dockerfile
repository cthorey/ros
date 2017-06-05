FROM osrf/ros:indigo-desktop-full

RUN apt-get update
RUN apt-get install -y ros-indigo-moveit
RUN apt-get install -y ros-indigo-moveit-full-pr2
RUN apt-get install ros-indigo-moveit-visual-tools

#catkin
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update
RUN apt-get install wget
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y python-catkin-tools
