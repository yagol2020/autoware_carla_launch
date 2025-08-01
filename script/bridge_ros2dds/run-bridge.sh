#!/usr/bin/env bash
set -e

PYTHON_AGENT_PATH=${AUTOWARE_CARLA_ROOT}/external/zenoh_carla_bridge/carla_agent
# Log folder
LOG_PATH=bridge_log/`date '+%Y-%m-%d_%H:%M:%S'`/
mkdir -p ${LOG_PATH}

# Note bridge should run later because it needs to configure Carla sync setting.
# Python script will overwrite the settings if bridge run first.
parallel --verbose --lb ::: \
        "sleep 5 && RUST_LOG=z=info ${AUTOWARE_CARLA_ROOT}/external/zenoh_carla_bridge/target/release/zenoh_carla_bridge \
                --mode ros2 --zenoh-listen tcp/0.0.0.0:7447 \
                --zenoh-config ${ZENOH_CARLA_BRIDGE_CONFIG} \
                --carla-address ${CARLA_SIMULATOR_IP} 2>&1 \
                | tee ${LOG_PATH}/bridge.log" \
        # "poetry -C ${PYTHON_AGENT_PATH} run python3 ${PYTHON_AGENT_PATH}/main.py \
        #         --host ${CARLA_SIMULATOR_IP} --rolename ${VEHICLE_NAME} --position 101.078339,133.431702,2.0,6.854668,7.250748,0.001844 \
        #         2>&1 | tee ${LOG_PATH}/vehicle.log"
