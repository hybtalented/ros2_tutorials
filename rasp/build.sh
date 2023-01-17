#!/bin/bash
###
# @Author Youbiao He hybtalented@163.com
# @Date 2022-10-26
 # @LastEditors Youbiao He
 # @LastEditTime 2023-01-17
 # @FilePath /rasp/build.sh
# @Description
###
CURRENT_DIR=$(
  cd "$(dirname $0)"
  pwd
)

CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=${CURRENT_DIR}/rasp-cross-compile.cmake"
TEST=""
if [ -n "$TEST" ]; then
  CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON --debug-find"
fi

export ROS2_INSTALL_PATH=$RASP_SYSROOT_DIR/opt/ros/rolling
source $ROS2_INSTALL_PATH/setup.bash
colcon build $@ --merge-install \
  --cmake-force-configure \
  --install-base ${CURRENT_DIR}/install \
  --build-base ${CURRENT_DIR}/build \
  --cmake-args $CMAKE_ARGS
