#!/bin/bash
# ──────────────────────────────────────────────────────────────────────────────
# run_gui.sh — Launch GUI nodes with CycloneDDS cross-version compatibility
#
# Usage:
#   ./run_gui.sh                         # demo_controller_gui (default)
#   ./run_gui.sh demo_controller_gui
#   ./run_gui.sh motion_editor_gui
#
# Ensures Jazzy (24.04) GUI can communicate with Humble (22.04) robot nodes
# by forcing CycloneDDS with permissive CDR encoding and disabled SharedMemory.
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Resolve cyclone_dds_gui.xml — check installed share path first, then source tree
SHARE_XML="$(ros2 pkg prefix rtc_controller_manager 2>/dev/null || true)/share/rtc_controller_manager/config/cyclone_dds_gui.xml"
SOURCE_XML="${SCRIPT_DIR}/../../rtc_controller_manager/config/cyclone_dds_gui.xml"

if [[ -f "$SHARE_XML" ]]; then
    DDS_XML="$SHARE_XML"
elif [[ -f "$SOURCE_XML" ]]; then
    DDS_XML="$(realpath "$SOURCE_XML")"
else
    echo "[ERROR] cyclone_dds_gui.xml not found" >&2
    exit 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${DDS_XML}"

GUI_NODE="${1:-demo_controller_gui}"

echo "[GUI] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[GUI] CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "[GUI] Launching: ${GUI_NODE}"

ros2 run ur5e_bringup "${GUI_NODE}" "${@:2}"
