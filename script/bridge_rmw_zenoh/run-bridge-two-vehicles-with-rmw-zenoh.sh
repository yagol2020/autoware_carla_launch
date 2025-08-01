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
                --mode rmw-zenoh --zenoh-listen tcp/0.0.0.0:7447 \
                --zenoh-config ${RMW_ZENOH_CARLA_BRIDGE_CONFIG} \
                --carla-address ${CARLA_SIMULATOR_IP} --slowdown 2 2>&1 \
                | tee ${LOG_PATH}/bridge.log" \
        "poetry -C ${PYTHON_AGENT_PATH} run python3 ${PYTHON_AGENT_PATH}/main.py \
                --host ${CARLA_SIMULATOR_IP} --rolename 'v1' \
                --position 87.687683,145.671295,0.300000,0.000000,90.000053,0.000000 \
                2>&1 | tee ${LOG_PATH}/vehicle1.log" \
        "sleep 1 && poetry -C ${PYTHON_AGENT_PATH} run python3 ${PYTHON_AGENT_PATH}/main.py \
                --host ${CARLA_SIMULATOR_IP} --rolename 'v2' \
                --position 92.109985,227.220001,0.300000,0.000000,-90.000298,0.000000 \
                2>&1 | tee ${LOG_PATH}/vehicle2.log"
