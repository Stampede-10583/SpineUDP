#!/usr/bin/env bash
# Copy the Orin's Spine logs to this machine (e.g. the driver station laptop, after a match or in
# the pits). Optional: the logs are always kept on the Orin; see "Log retrieval" in README.md.
#
#   scripts/pull_logs.sh [user@host] [dest_dir]
#
# Defaults: ${SPINE_ORIN_USER:-nvidia}@${SPINE_ORIN_HOST:-10.105.83.11}, remote directory
# ${SPINE_ORIN_LOG_DIR:-~/SpineUDP/ros_ws/spine_logs}, destination ./spine_logs/<host>/.
# rsync over SSH (delta transfer, so re-running is cheap even after log rotation renames files). Nothing is deleted
# on either side.
set -euo pipefail

TARGET="${1:-${SPINE_ORIN_USER:-nvidia}@${SPINE_ORIN_HOST:-10.105.83.11}}"
REMOTE_DIR="${SPINE_ORIN_LOG_DIR:-~/SpineUDP/ros_ws/spine_logs}"
DEST="${2:-./spine_logs/${TARGET#*@}}"

command -v rsync >/dev/null || { echo "error: rsync not installed" >&2; exit 1; }
mkdir -p "${DEST}"
echo "Pulling ${TARGET}:${REMOTE_DIR}/ -> ${DEST}/"
rsync -av -e "ssh -o ConnectTimeout=5" "${TARGET}:${REMOTE_DIR}/" "${DEST}/"
