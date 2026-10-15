#!/usr/bin/env bash

# 🔑 [FAILSAFE] DETECT AND STOP THE SCRIPT
set -euo pipefail

# ⛛ [GENERATE] CREATE FILES IF THEY DON'T EXIST 
mkdir -p "$HOME/workspaces/drivers_ws/build_drivers" "$HOME/workspaces/drivers_ws/install_drivers"
mkdir -p "$HOME/workspaces/fast_lio_ws/build_fastlio" "$HOME/workspaces/fast_lio_ws/install_fastlio"
mkdir -p "$HOME/workspaces/unitree_ws/build_unitree" "$HOME/workspaces/unitree_ws/install_unitree"
mkdir -p "$(pwd)/build_docker" "$(pwd)/install_docker"
mkdir -p "$HOME/docker_data"
mkdir -p "$HOME/robinlab_config"

# ▶ [MOUNT] CREATING VOLUMES

missing=0

# 💡[CHECK]: LOOK FOR THIS PATHS
for d in \
	"$HOME/workspaces/drivers_ws/src/livox_ros_driver2" \
	"$HOME/workspaces/drivers_ws/src/ublox_ros2_ardusimple" \
	"$HOME/workspaces/fast_lio_ws/src/FAST_LIO_ROS2" \
	"$HOME/workspaces/unitree_ws/src/unitree_ros2"; do
	
	# ❓[CHECK] FOLDER EXIST
	if [ ! -d "$d" ]; then
		# ❌ [CHECK] DON'T
		echo " ❌ [CHECK]: THE FOLDER < $d > IS MISSING, CLONE IT BEFORE BUILD THE CONTAINER"
		missing=1

	fi
done

# ❓[IF] FOLDERS ARE READY 
if [ "$missing" -eq 0 ]; then
	# ✅ [IF] FOLDERS ARE READY
	echo " ✅ [CONFIG] VOLUMES AND REPOS ARE READY"
else
	echo " ❌ [CONFIG] VOLUMES ARE READY BUT THERE ARE MISSING REPOS, CHECK THE INSTALLATION GUIDE"
fi
	echo " 🚀 [RUNNING] (GROUND STATION ONLY) BEFORE BUILDING THE GUI: xhost +local:root"



