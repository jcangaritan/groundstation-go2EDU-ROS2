FROM ros:humble 

ENV DEBIAN_FRONTEND=noninteractive

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
	cmake \
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
	ros-humble-rtcm-msgs \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] LINUX GSTREAMER 
RUN apt-get update && apt-get install -y \
	libgstreamer1.0-dev \
	libgstreamer-plugins-base1.0-dev \
	libgstreamer-plugins-bad1.0-dev \
	gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good \
	gstreamer1.0-plugins-bad \
	gstreamer1.0-plugins-ugly \
	gstreamer1.0-libav \
	gstreamer1.0-tools \
	gstreamer1.0-x \
	gstreamer1.0-alsa \
	gstreamer1.0-gi \
	gstreamer1.0-gtk3 \
	gstreamer1.0-qt5 \
	gstreamer1.0-pulseaudio \
	&& rm -rf /var/lib/apt/lists/*

# ⚠ [DOWNLOAD & INSTALL] LIVOX SDK 
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
	cd /tmp/Livox-SDK2 && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make -j$(nproc) && \
	make install && \
	rm -rf /tmp/Livox-SDK2

# ⚠ [DOWNLOAD & INSTALL] CYCLONE <<CPP 0.10.x>>
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
	git clone -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git /tmp/cyclonedds-cxx && \
	cd /tmp/cyclonedds-cxx && \
	mkdir build && \
	cd build && \
	cmake .. \
		-DCMAKE_INSTALL_PREFIX=/usr/local \
		-DCycloneDDS_DIR=/opt/ros/humble && \
	make -j$(nproc) && \
	make install && \
	rm -rf /tmp/cyclonedds-cxx"

# ⚙ [SETUP] TMUX
RUN echo "set -g @plugin 'tmux-plugins/tpm'" > /root/.tmux.conf && \
    echo "set -g @plugin 'jimeh/tmux-themepack'" >> /root/.tmux.conf && \
    echo "set -g @themepack 'powerline/block/green'" >> /root/.tmux.conf && \
    echo "set-option -g default-command '/bin/bash'" >> /root/.tmux.conf && \
    echo "set-option -g default-terminal 'xterm-256color'" >> /root/.tmux.conf && \
    echo "run '~/.tmux/plugins/tpm/tpm'" >> /root/.tmux.conf

# ⚙ [SETUP] ADDING RED
RUN echo "force_color_prompt=yes" >> /root/.bashrc && \
    echo "export PS1='\[\033[01;31m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /root/.bashrc

# ▶ [SETUP] CONFIGURE MIDDLEWARE
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ▶ [SETUP] CONFIGURE LIBRARY PATH
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}

# ⛛ [INSTALL] PYTHON3 DEPENDENCIES
RUN pip3 install --no-cache-dir \
	numpy \
	scipy \
	pandas \
	pyserial \
	transforms3d \
	termcolor

WORKDIR /root/robinlab_ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV RCUTILS_COLORIZED_OUTPUT=1
ENV TERM=xterm-256color
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
