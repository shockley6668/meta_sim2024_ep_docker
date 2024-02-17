##########################################################################
FROM tb5zhh/icra-2023-client-base-carto:v2.0.0

# Add source codes into images and install dependencies
RUN pwd
ADD src /opt/ep_ws/src/rmus_solution

WORKDIR /opt/ep_ws
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/realsense.list
RUN apt-get update -q && \
     source /opt/workspace/devel_isolated/setup.bash && \
     ROS_OS_OVERRIDE=ubuntu:20.04:focal \
     DEBIAN_FRONTEND=noninteractive \        
     rosdep install -q --from-paths src --ignore-src --rosdistro noetic -y && \
     rm -rf /var/lib/apt/lists/* && \
     apt-get clean

# Install extra dependencies with apt
RUN apt-get update && \
     apt-get install -y --no-install-recommends \
     wget ros-noetic-depthimage-to-laserscan ros-noetic-map-server python3-tf-conversions ros-noetic-global-planner && \
     rm -rf /var/lib/apt/lists/* && apt-get clean
# RUN rosdep install --from-paths src --ignore-src -r -y

RUN sudo apt install cmake build-essential libssl-dev libhwloc-dev 

RUN wget https://github.com/zeromq/zeromq4-1/releases/download/v4.1.7/zeromq-4.1.7.tar.gz && \
tar -xvf zeromq-4.1.7.tar.gz && \
cd zeromq-4.1.7 && \
./configure --without-libsodium && \
sudo make && \
sudo make install && \
sudo ldconfig

RUN wget https://github.com/sqlite/sqlite/archive/refs/tags/vesion-3.45.1.tar.gz && \
tar -xvf vesion-3.45.1.tar.gz && \ 
cd sqlite-vesion-3.45.1 && \
CFLAGS="-DSQLITE_ENABLE_COLUMN_METADATA=1" ./configure && \
sudo make && \
sudo make install && \
sudo ldconfig

ADD googletest-1.10.x /opt/ep_ws/googletest-1.10.x
WORKDIR /opt/ep_ws/googletest-1.10.x
RUN ls && \
sudo mkdir build && \
cd build && \
cmake .. && \
make && \
sudo make install && \
sudo ldconfig

ADD BehaviorTree.CPP /opt/ep_ws/BehaviorTree.CPP
WORKDIR /opt/ep_ws/BehaviorTree.CPP
RUN sudo mkdir build; cd build && \
cmake .. && \
cmake --build . --parallel

ADD apriltag /opt/ep_ws/apriltag
WORKDIR /opt/ep_ws/apriltag
RUN sudo cmake -B build -DCMAKE_BUILD_TYPE=Release \
&&sudo cmake --build build --target install

ADD apriltag_ros /opt/ep_ws/src/apriltag_ros
ADD threshold_image /opt/ep_ws/src/threshold_image
WORKDIR /opt/ep_ws
RUN source /opt/workspace/devel_isolated/setup.bash && catkin_make install --use-ninja -DSETUPTOOLS_DEB_LAYOUT=OFF

#Install extra dependencies with pip
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple scipy



#RUN source /opt/workspace/devel_isolated/setup.bash && catkin_make install --use-ninja -DSETUPTOOLS_DEB_LAYOUT=ON
# Patch for keyboard
ADD ep_teleop /opt/ep_ws/src/ep_teleop

# Make client package
RUN source /opt/workspace/devel_isolated/setup.bash && catkin_make install --use-ninja -DSETUPTOOLS_DEB_LAYOUT=OFF
ENV ENV_ROBOT_MODE=sim

# Add start script
ADD start.sh /opt/start.sh
RUN echo "source /opt/ep_ws/devel/setup.bash" >> ~/.bashrc
CMD /opt/ros/noetic/env.sh /opt/ep_ws/devel/env.sh /opt/start.sh

#TODO oplin add command
RUN apt-get update && \
     apt-get install -y ros-noetic-teleop-twist-keyboard ros-noetic-rqt ros-noetic-rqt-common-plugins
