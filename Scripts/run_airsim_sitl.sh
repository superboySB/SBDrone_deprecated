#!/bin/bash
# 判断当前环境是否有设置instance_num，如果没有则默认为0
# [ ! -z "$instance_num" ] && instance_num="0"
export PX4_SIM_MODEL=iris
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR=$PARENT_DIR/PX4-Autopilot/ROMFS/px4fmu_common
instance_path=$PARENT_DIR/PX4-Autopilot/build/px4_sitl_default
BIN_DIR=$PARENT_DIR/PX4-Autopilot/build/px4_sitl_default/bin/px4
TEST_DATA=$PARENT_DIR/PX4-Autopilot/test_data
working_dir="$instance_path/instance_$instance_num"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
pushd "$working_dir" &>/dev/null
$BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA