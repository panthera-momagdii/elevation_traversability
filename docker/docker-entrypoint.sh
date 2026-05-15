#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS}/install/setup.bash"

# Auto-pick CycloneDDS XML if one was mounted and the user didn't already set CYCLONEDDS_URI.
if [[ -z "${CYCLONEDDS_URI}" && -f "/config/cyclonedds.xml" ]]; then
    export CYCLONEDDS_URI="file:///config/cyclonedds.xml"
fi

# If user supplied a CycloneDDS config but forgot to switch RMW, do it for them.
if [[ -n "${CYCLONEDDS_URI}" && -z "${RMW_IMPLEMENTATION}" ]]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi

echo "[entrypoint] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default rmw_fastrtps_cpp>}"
[[ -n "${CYCLONEDDS_URI}" ]] && echo "[entrypoint] CYCLONEDDS_URI=${CYCLONEDDS_URI}"

LAUNCH_ARGS=()

if [[ -n "${PARAMS_FILE}" && -f "${PARAMS_FILE}" ]]; then
    echo "[entrypoint] using params_file: ${PARAMS_FILE}"
    LAUNCH_ARGS+=("params_file:=${PARAMS_FILE}")
elif [[ -n "${PARAMS_FILE}" ]]; then
    echo "[entrypoint] PARAMS_FILE='${PARAMS_FILE}' not found, falling back to packaged config" >&2
fi

LAUNCH_ARGS+=("rviz:=${RVIZ:-false}")

if [[ $# -eq 0 ]]; then
    exec ros2 launch elevation_traversability elevation_traversability.launch.py \
        "${LAUNCH_ARGS[@]}"
else
    exec "$@"
fi
