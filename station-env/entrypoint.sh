#!/usr/bin/env bash

set -e

# ▶ [SETUP] ROS2 base
source /opt/ros/humble/setup.bash

# ▶ [SETUP] WORKSPACES

for ws in /root/drivers_ws /root/fast_lio_ws /root/robinlab_ws; do
	if [ -f "$ws/install/setup.bash" ]; then
		source "$ws/install/setup.bash"
	fi
done

# ▶ [SETUP] CycloneDDS: CUSTOMIZED XML
if [ -z "${CYCLONE_URI:-}" ] && [ -f /root/robinlab_ws/station-env/cyclonedds.xml ]; then
	export CYCLONEDDS_URI=file:///root/robinlab_ws/station-env/cyclonedds.xml
fi

exec "$@"
