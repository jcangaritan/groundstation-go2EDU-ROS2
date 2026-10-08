FROM ros:humble 

ENV DEBIANT_FRONTEND=noninteractive

# 🚧 ENVIROMENT SETUP

# ⛛ [INSTALL] LINUX APP
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

# ⛛ [INSTALL] C++ LINEAR ALGEBRA
RUN apt-get update && apt-get install -y \
	libeigen3-dev \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 GENERAL TOOLS
RUN apt-get update && apt-get install -y \
	ros-humble-rmw-cyclonedds-cpp \
	ros-humble-rosidl-default-generators \
	ros-humble-teleop-twist-keyboard \
	ros-humble-diagnostic-common-diagnostics \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 CAMERA TOOLS
RUN apt-get update && apt-get install -y \
	ros-humble-cv-bridge \
	ros-humble-image-transport \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 SPATIAL GEOMETRY
RUN apt-get update && apt-get install -y \
	ros-humble-tf2-geometry-msgs \
	ros-humble-xacro \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 POINTCLOUD GEOMETRY [PCL]
RUN apt-get update && apt-get install -y \
	ros-humble-perception-pcl \
	ros-humble-pcl-ros \
	ros-humble-pcl-conversions \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 NAVIGATION
RUN apt-get update && apt-get install -y \
	ros-humble-navigation2 \
	ros-humble-nav2-bringup \
	ros-humble-geographic-msgs \
	ros-humble-nmea-msgs \
	ros-humble-rctm-msgs \
	&& rm -rf /var/lib/apt/lists/*
