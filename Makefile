.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := session

.PHONY: session
session:
	@CONT_NAME="${CONT_NAME}"
	@RUNTIME="${RUNTIME}"
	@TAG="${TAG}"
	@ENTRYPOINT="${ENTRYPOINT}"
	if [ "${CONT_NAME}" == "" ]; then
		CONT_NAME="test_container"
	fi
	if [ "${TAG}" == "" ]; then
		TAG="stable"
	fi
	IMG_NAME=ghcr.io/c-k-robotics/frc_ros_humble_cpu:$${TAG}
	if [ "${RUNTIME}" = "nvidia" ]; then
		echo "RUNTIME is set to nvidia"
		xhost +
		docker run \
			--name $${CONT_NAME} \
			--runtime nvidia \
			-it \
			--rm \
			--privileged \
			--net=host \
			--gpus all \
			-e NVIDIA_DRIVER_CAPABILITIES=all \
			-e DISPLAY=${DISPLAY} \
			-v /dev/bus/usb:/dev/bus/usb \
			--device-cgroup-rule='c 189:* rmw' \
			--device /dev/video0 \
			--volume=/dev/input:/dev/input \
			--volume=${HOME}/.Xauthority:/root/.Xauthority:rw \
			--volume=/tmp/.X11-unix/:/tmp/.X11-unix \
			--volume=${PWD}:/opt/ros2_ws/src/ \
			$${IMG_NAME} ${ENTRYPOINT}
	else
		xhost +
		docker run \
			--name $${CONT_NAME} \
			-it \
			--rm \
			--privileged \
			--net=host \
			-e DISPLAY=${DISPLAY} \
			-v /dev/bus/usb:/dev/bus/usb \
			--device-cgroup-rule='c 189:* rmw' \
			--device /dev/video0 \
			--volume=/dev/input:/dev/input \
			--volume=${HOME}/.Xauthority:/root/.Xauthority:rw \
			--volume=/tmp/.X11-unix/:/tmp/.X11-unix \
			--volume=${PWD}:/opt/ros2_ws/src/ \
			$${IMG_NAME} ${ENTRYPOINT}
	fi


.PHONY: join-session
join-session:
	@CONT_NAME="${CONT_NAME}"
	docker exec -it ${CONT_NAME} /bin/bash
