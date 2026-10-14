# ================================================================
#  DESKTOP :: Ground station (ASUS TUF A15, x86_64, RTX 4060)
#  Superset del robot + herramientas gráficas (RViz2, PlotJuggler, teleop).
#  Build: docker compose --profile gui build
# ================================================================

# 🚧 ENVIRONMENT SETUP
# ▶ [SETUP] BASE IMAGE (ya trae RViz2, rqt, Gazebo-msgs y demás herramientas de escritorio)
ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM ${BASE_IMAGE}

# ▶ [SETUP] DISABLE DIALOGUE (solo durante el build)
ARG DEBIAN_FRONTEND=noninteractive

# ⛛ [INSTALL] LINUX APP
RUN apt-get update && apt-get install -y \
	git \
	neovim \
	tmux \
	nano \
	wget \
	curl \
	htop \
	net-tools \
	iputils-ping \
	usbutils \
	tcpdump \
	libgl1-mesa-glx \
	libgl1-mesa-dri \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] BUILD TOOLS & C++ LIBS
RUN apt-get update && apt-get install -y \
	build-essential \
	cmake \
	python3-pip \
	python3-colcon-common-extensions \
	libeigen3-dev \
	libyaml-cpp-dev \
	libasio-dev \
	libboost-all-dev \
	libapr1-dev \
	libaprutil1-dev \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 GENERAL TOOLS
RUN apt-get update && apt-get install -y \
	ros-humble-rmw-cyclonedds-cpp \
	ros-humble-rosidl-default-generators \
	ros-humble-tf2-geometry-msgs \
	ros-humble-diagnostic-common-diagnostics \
	ros-humble-xacro \
	ros-humble-rosbag2-storage-mcap \
	ros-humble-iceoryx-binding-c \
	ros-humble-iceoryx-posh \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 VISUALIZATION & TELEOP (solo ground station)
RUN apt-get update && apt-get install -y \
	ros-humble-rviz2 \
	ros-humble-plotjuggler-ros \
	ros-humble-teleop-twist-keyboard \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 POINTCLOUD GEOMETRY [PCL]
RUN apt-get update && apt-get install -y \
	libpcl-dev \
	ros-humble-perception-pcl \
	ros-humble-pcl-ros \
	ros-humble-pcl-conversions \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 CAMERA TOOLS
RUN apt-get update && apt-get install -y \
	ros-humble-cv-bridge \
	ros-humble-image-transport \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 LOCALIZATION & GEO MESSAGES (sin Nav2: la odometría la da FAST-LIO2)
RUN apt-get update && apt-get install -y \
	ros-humble-robot-localization \
	ros-humble-geographic-msgs \
	ros-humble-nmea-msgs \
	ros-humble-rtcm-msgs \
	geographiclib-tools \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] LINUX GSTREAMER (completo, con sinks gráficos y de audio)
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
	gstreamer1.0-gl \
	gstreamer1.0-gtk3 \
	gstreamer1.0-qt5 \
	gstreamer1.0-pulseaudio \
	&& rm -rf /var/lib/apt/lists/*

# ▶ [SETUP] CONFIGURE MIDDLEWARE
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ▶ [SETUP] GEOGRAPHICLIB GEOIDS
RUN geographiclib-get-geoids egm96-5

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

# ⚙ [SETUP] NEOVIM
RUN mkdir -p /root/.config/nvim && \
	echo "autocmd VimEnter * NoMatchParen" >> /root/.config/nvim/init.vim && \
	echo "nnoremap p p \`]" >> /root/.config/nvim/init.vim

# ⚙ [SETUP] TMUX
RUN git clone https://github.com/tmux-plugins/tpm /root/.tmux/plugins/tpm

# Forcing /bin/bash to prevent unexpected behaviour
RUN echo "set -g @plugin 'tmux-plugins/tpm'" > /root/.tmux.conf && \
    echo "set -g @plugin 'jimeh/tmux-themepack'" >> /root/.tmux.conf && \
    echo "set -g @themepack 'powerline/block/green'" >> /root/.tmux.conf && \
    echo "set-option -g default-command '/bin/bash'" >> /root/.tmux.conf && \
    echo "set-option -g default-terminal 'xterm-256color'" >> /root/.tmux.conf && \
    echo "run '~/.tmux/plugins/tpm/tpm'" >> /root/.tmux.conf

# ⚙ [SETUP] ADDING GREEN (el portátil en verde, para distinguirlo del robot en rojo)
RUN echo "force_color_prompt=yes" >> /root/.bashrc && \
    echo "export PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /root/.bashrc

# ⚙ [SETUP] AUTO SOURCE
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/drivers_ws/install/setup.bash ]; then source /root/drivers_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /root/fast_lio_ws/install/setup.bash ]; then source /root/fast_lio_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /root/geospatial_ws/install/setup.bash ]; then source /root/geospatial_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "echo 'ROS2 Humble [GROUND STATION] & workspaces are ready.'" >> /root/.bashrc

# ⚙ [SETUP] MORE USED PATHS
RUN echo "alias ggeo='cd /root/geospatial_ws'" >> /root/.bashrc && \
    echo "alias gdata='cd /root/geospatial_ws/data'" >> /root/.bashrc && \
    echo "alias gsrc='cd /root/geospatial_ws/src/geospatial-nav/src'" >> /root/.bashrc

#RUN echo "export CYCLONEDDS_URI=file:///root/geospatial_ws/src/geospatial-nav/cyclonedds.xml" >> /root/.bashrc

# ▶ [SETUP] INPUT
WORKDIR /root/geospatial_ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV RCUTILS_COLORIZED_OUTPUT=1
ENV TERM=xterm-256color
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
