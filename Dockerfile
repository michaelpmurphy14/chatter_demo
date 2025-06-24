# Use the official ROS Melodic base image
FROM ros:melodic

# Install catkin_tools, ROS Python libs, ROS CLI, and networking tools
RUN apt-get update && apt-get install -y \
     python-catkin-tools \
     python-rosdep \
     python-rospkg \
     ros-melodic-ros-comm \
     iputils-ping \
     netcat-openbsd \
  && rm -rf /var/lib/apt/lists/*

# Set up and build the Catkin workspace
WORKDIR /ros_ws
COPY . /ros_ws/src/chatter_demo
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  mkdir -p src && \
                  cd /ros_ws && \
                  catkin init && \
                  catkin config --extend /opt/ros/melodic && \
                  catkin build"

# At container start, source ROS and your workspace, then run listener
ENTRYPOINT [ "bash", "-lc", "\
  source /opt/ros/melodic/setup.bash && \
  source /ros_ws/devel/setup.bash && \
  export ROS_MASTER_URI=http://192.168.68.58:11311 && \
  export ROS_IP=$(hostname -I | awk '{print \$1}') && \
  rosrun chatter_demo listener.py" ]
