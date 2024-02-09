#!/bin/bash
SERVER_IMAGE=${SERVER_IMAGE:-rmus2022/server:result_pub_fix}
CLIENT_IMAGE=${CLIENT_IMAGE:-meta_sim/test:v1.1}
CLI_EXE=$@

xhost +

# docker pull $SERVER_IMAGE

docker network create net-sim

docker run -dit --rm --name ros-master --network net-sim ros:noetic-ros-core-focal roscore

docker run -dit --rm --name sim-server --network net-sim \
	--gpus all \
	-e ROS_MASTER_URI=http://ros-master:11311 \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e NO_AT_BRIDGE=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	$SERVER_IMAGE 

sleep 10
# docker run -it --name client_316 --network net-sim \
# 	--cpus=5.6 -m 8192M \
# 	-e ROS_MASTER_URI=http://ros-master:11311 \
# 	-e DISPLAY=$DISPLAY \
# 	-e QT_X11_NO_MITSHM=1 \
# 	-e NO_AT_BRIDGE=1 \
# 	-e LIBGL_ALWAYS_SOFTWARE=1 \
# 	-v /tmp/.X11-unix:/tmp/.X11-unix \
# 	$CLIENT_IMAGE $CLI_EXE
sleep 2
docker network connect net-sim client 
docker restart client 