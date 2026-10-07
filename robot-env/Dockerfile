FROM ros:humble 

ENV DEBIANT_FRONTEND=noninteractive

# 🛠 Linux tools
RUN apt-get update && apt-get install -y \
	git \
	neovim \
	tmux \
	nano \
	wget \
	curl \
	net-tools \
	iputils-ping \
	usbutils \
	apt-transport-https \
	libgl1-mesa-glx \
	libgl1-mesa-dri \
	python3-pip \
	build-essential \
	python3-colcon-common-extensions \
	&& rm -rf /var/lib/apt/lists/*

# ⚙ ROS2 general tools

RUN apt-get update && apt-get install -y \
	ros-humble-rmw-cyclonedds-cpp \
	ros-humble-rosidl-default-generators \
	&& rm -rf /var/lib/apt/lists/*

# 💡 ROS2 Perception (LiDAR -CAMERA)
