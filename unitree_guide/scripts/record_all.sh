#!/usr/bin/env bash
BASE_DIR="$1"
SLAM_PREFIX="$2"
TS="$(date +%Y%m%d_%H%M%S)"
HOST="$HOSTNAME"
USER_NAME="$USER"

OUT_DIR="${BASE_DIR}/${SLAM_PREFIX}"
mkdir -p "$OUT_DIR"
rosbag record -a -O "${OUT_DIR}/${HOST}_${USER_NAME}_${TS}"