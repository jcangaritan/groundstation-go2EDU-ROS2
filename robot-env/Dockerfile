# ================================================================
#  ROBOT :: Unitree Go2 EDU (Jetson Orin NX, ARM64)
#  Imagen mínima y headless: sin RViz, sin PlotJuggler, sin GUI.
#  Build: docker compose --profile robot build
# ================================================================

# 🚧 ENVIRONMENT SETUP
# ▶ [SETUP] BASE IMAGE
# ros:humble es solo ros-core (sin rosbag2 ni herramientas tf2). Antes llegaban como
# dependencia de Nav2; sin Nav2 se usa ros-base, que incluye `ros2 bag` para grabar.
# Si algún día necesitas CUDA/TensorRT dentro del contenedor, cambia BASE_IMAGE
# por una imagen dustynv/ros:humble-... que coincida con tu L4T.
ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

# ▶ [SETUP] DISABLE DIALOGUE (solo durante el build)
ARG DEBIAN_FRONTEND=noninteractive

# ⛛ [INSTALL] LINUX APP
# Con --no-install-recommends hay que pedir ca-certificates a mano (git/curl/wget por https).
RUN apt-get update && apt-get install -y --no-install-recommends \
	ca-certificates \
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
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] BUILD TOOLS & C++ LIBS
RUN apt-get update && apt-get install -y --no-install-recommends \
	build-essential \
	cmake \
	python3-dev \
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
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-humble-rmw-cyclonedds-cpp \
	ros-humble-rosidl-default-generators \
	ros-humble-tf2-geometry-msgs \
	ros-humble-diagnostic-common-diagnostics \
	ros-humble-xacro \
	ros-humble-rosbag2-storage-mcap \
	ros-humble-iceoryx-binding-c \
	ros-humble-iceoryx-posh \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 POINTCLOUD GEOMETRY [PCL]
# perception-pcl es solo un metapaquete de pcl-ros + pcl-conversions + pcl-msgs.
RUN apt-get update && apt-get install -y --no-install-recommends \
	libpcl-dev \
	ros-humble-pcl-ros \
	ros-humble-pcl-conversions \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 CAMERA TOOLS
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-humble-cv-bridge \
	ros-humble-image-transport \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] ROS2 LOCALIZATION & GEO MESSAGES (sin Nav2: la odometría la da FAST-LIO2)
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-humble-robot-localization \
	ros-humble-geographic-msgs \
	ros-humble-nmea-msgs \
	ros-humble-rtcm-msgs \
	geographiclib-tools \
	&& rm -rf /var/lib/apt/lists/*

# ⛛ [INSTALL] LINUX GSTREAMER (headless: sin gtk3, qt5, x, gl, alsa, pulseaudio, ugly)
RUN apt-get update && apt-get install -y --no-install-recommends \
	libgstreamer1.0-dev \
	libgstreamer-plugins-base1.0-dev \
	libgstreamer-plugins-bad1.0-dev \
	gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good \
	gstreamer1.0-plugins-bad \
	gstreamer1.0-libav \
	gstreamer1.0-tools \
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

# ⚠ [DOWNLOAD & INSTALL] UNITREE SDK2 (desactivado)
#RUN git clone https://github.com/jcangaritan/unitree_sdk2.git /tmp/unitree_sdk2 && \
#	cd /tmp/unitree_sdk2 && \
#	mkdir build && \
#	cd build && \
#	cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
#	make -j$(nproc) && \
#	make install && \
#	rm -f /usr/local/lib/libddsc.so* && \
#	rm -rf /tmp/unitree_sdk2

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

# ⚙ [SETUP] ADDING RED (el robot siempre en rojo, para no confundirlo con el portátil)
RUN echo "force_color_prompt=yes" >> /root/.bashrc && \
    echo "export PS1='\[\033[01;31m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /root/.bashrc

# ⚙ [SETUP] AUTO SOURCE
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/drivers_ws/install/setup.bash ]; then source /root/drivers_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /root/fast_lio_ws/install/setup.bash ]; then source /root/fast_lio_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /root/geospatial_ws/install/setup.bash ]; then source /root/geospatial_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "echo 'ROS2 Humble [ROBOT] & workspaces are ready.'" >> /root/.bashrc

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
