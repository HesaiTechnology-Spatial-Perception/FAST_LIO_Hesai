#!/usr/bin/env bash
# Copyright 2026 Hesai Technology. All rights reserved.
# SPDX-License-Identifier: GPL-2.0
#
# pcap_to_rosbag_ros2.sh
# Convert a Hesai PCAP file to a FAST-LIO2-ready rosbag2 (ROS 2).
#
# Pipeline:
#   input.pcap
#     → Hesai ROS 2 Driver (source_type: 2)
#     → /lidar_points + /lidar_imu
#     → ros2 bag record
#     → output/  (rosbag2 directory)
#
# Usage:
#   bash pcap_to_rosbag_ros2.sh \
#     --model      jt128 \
#     --pcap       /data/input.pcap \
#     --correction /data/correction.csv \
#     --firetime   /data/firetime.csv \
#     --output     /data/output \
#     --driver-ws  ~/hesai_ros2_ws
#
# Requirements:
#   - ROS 2 Humble sourced
#   - Hesai ROS 2 Driver installed in --driver-ws
#   - Python 3 with PyYAML  (pip3 install pyyaml)
#
# Note: --output is a DIRECTORY path (rosbag2 format), not a .bag file.

set -euo pipefail

# ── colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YEL='\033[0;33m'; BLU='\033[0;34m'; NC='\033[0m'
info()  { echo -e "${BLU}[INFO]${NC}  $*"; }
ok()    { echo -e "${GRN}[OK]${NC}    $*"; }
warn()  { echo -e "${YEL}[WARN]${NC}  $*"; }
die()   { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

# ── defaults ──────────────────────────────────────────────────────────────────
MODEL=""
PCAP=""
CORRECTION=""
FIRETIME=""
OUTPUT=""
DRIVER_WS=""
LIDAR_TOPIC="/lidar_points"
IMU_TOPIC="/lidar_imu"
TOPIC_WAIT_TIMEOUT=30
SILENCE_TIMEOUT=4
PLAY_RATE=1.0

# ── argument parsing ──────────────────────────────────────────────────────────
usage() {
    grep '^#' "$0" | grep -v '^#!/' | sed 's/^# \{0,2\}//'
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --model)       MODEL="$2";       shift 2 ;;
        --pcap)        PCAP="$2";        shift 2 ;;
        --correction)  CORRECTION="$2";  shift 2 ;;
        --firetime)    FIRETIME="$2";    shift 2 ;;
        --output)      OUTPUT="$2";      shift 2 ;;
        --driver-ws)   DRIVER_WS="$2";   shift 2 ;;
        --play-rate)   PLAY_RATE="$2";   shift 2 ;;
        --lidar-topic) LIDAR_TOPIC="$2"; shift 2 ;;
        --imu-topic)   IMU_TOPIC="$2";   shift 2 ;;
        -h|--help)     usage ;;
        *) die "Unknown argument: $1" ;;
    esac
done

# ── validation ────────────────────────────────────────────────────────────────
[[ -n "$MODEL" ]]      || die "--model is required (jt16 or jt128)"
[[ -n "$PCAP" ]]       || die "--pcap is required"
[[ -n "$CORRECTION" ]] || die "--correction is required"
[[ -n "$FIRETIME" ]]   || die "--firetime is required"
[[ -n "$OUTPUT" ]]     || die "--output is required"
[[ -n "$DRIVER_WS" ]]  || die "--driver-ws is required"

[[ "$MODEL" == "jt16" || "$MODEL" == "jt128" ]] || die "--model must be jt16 or jt128"
[[ -f "$PCAP" ]]       || die "PCAP file not found: $PCAP"
[[ -f "$CORRECTION" ]] || die "Correction file not found: $CORRECTION"
[[ -f "$FIRETIME" ]]   || die "Firetime file not found: $FIRETIME"
[[ -d "$DRIVER_WS" ]]  || die "Driver workspace not found: $DRIVER_WS"

command -v ros2       >/dev/null 2>&1 || die "ros2 not found. Source your ROS 2 workspace."
command -v python3    >/dev/null 2>&1 || die "python3 not found."
python3 -c "import yaml" 2>/dev/null  || die "PyYAML not found. Run: pip3 install pyyaml"

# ── find driver config ────────────────────────────────────────────────────────
DRIVER_CONFIG=""
for candidate in \
    "$DRIVER_WS/src/HesaiLidar_ROS_2.0/config/config.yaml" \
    "$DRIVER_WS/src/hesai_ros_driver/config/config.yaml" \
    "$DRIVER_WS/src/HesaiLidar_ROS2/config/config.yaml"; do
    [[ -f "$candidate" ]] && { DRIVER_CONFIG="$candidate"; break; }
done
[[ -n "$DRIVER_CONFIG" ]] || die "Cannot find Hesai ROS 2 Driver config.yaml in $DRIVER_WS."

info "Driver config: $DRIVER_CONFIG"

# ── check output doesn't already exist ───────────────────────────────────────
[[ -e "$OUTPUT" ]] && die "Output already exists: $OUTPUT. Remove it first or choose a different path."

# ── temp files and cleanup ────────────────────────────────────────────────────
TMP_CONFIG=$(mktemp /tmp/hesai_pcap_config_XXXXXX.yaml)
CONFIG_BACKUP="${DRIVER_CONFIG}.pcap_tool_backup"
DRIVER_PID=""
BAG_PID=""

cleanup() {
    info "Cleaning up..."
    [[ -n "$BAG_PID" ]]    && kill "$BAG_PID"    2>/dev/null || true
    [[ -n "$DRIVER_PID" ]] && kill "$DRIVER_PID" 2>/dev/null || true
    [[ -f "$CONFIG_BACKUP" ]] && mv "$CONFIG_BACKUP" "$DRIVER_CONFIG" && info "Restored driver config."
    rm -f "$TMP_CONFIG"
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ── generate PCAP driver config ───────────────────────────────────────────────
info "Generating PCAP driver config..."
python3 - <<PYEOF
import yaml, sys

with open("$DRIVER_CONFIG") as f:
    cfg = yaml.safe_load(f)

if not cfg or "lidar" not in cfg or not cfg["lidar"]:
    print("[ERROR] Unexpected driver config structure.", file=sys.stderr)
    sys.exit(1)

drv = cfg["lidar"][0]["driver"]
drv["source_type"] = 2
drv.pop("lidar_udp_type", None)
drv["pcap_type"] = {
    "pcap_path":             "$PCAP",
    "correction_file_path":  "$CORRECTION",
    "firetime_file_path":    "$FIRETIME",
    "pcap_play_synchronization": True,
    "pcap_play_in_loop":    False,
    "play_rate_":            $PLAY_RATE,
}

ros = drv.setdefault("ros", {})
ros["ros_send_point_cloud_topic"] = "$LIDAR_TOPIC"
ros["ros_send_imu_topic"]         = "$IMU_TOPIC"
ros["send_point_cloud_ros"]       = True
ros["send_imu_ros"]               = True

with open("$TMP_CONFIG", "w") as f:
    yaml.dump(cfg, f, default_flow_style=False)
PYEOF
ok "Config generated."

# ── source driver workspace and back up config ────────────────────────────────
# shellcheck disable=SC1090
source "$DRIVER_WS/install/setup.bash"

cp "$DRIVER_CONFIG" "$CONFIG_BACKUP"
cp "$TMP_CONFIG"    "$DRIVER_CONFIG"
info "Backed up original config → $CONFIG_BACKUP"

# ── start driver ──────────────────────────────────────────────────────────────
info "Starting Hesai ROS 2 Driver in PCAP mode..."
ros2 launch hesai_ros_driver start.launch.py &
DRIVER_PID=$!
sleep 3   # allow driver to initialize

# ── wait for topics ───────────────────────────────────────────────────────────
info "Waiting for topics (timeout: ${TOPIC_WAIT_TIMEOUT}s)..."
deadline=$(( $(date +%s) + TOPIC_WAIT_TIMEOUT ))
while true; do
    pc_ok=false; imu_ok=false
    topics=$(ros2 topic list 2>/dev/null || true)
    echo "$topics" | grep -q "^${LIDAR_TOPIC}$"  && pc_ok=true
    echo "$topics" | grep -q "^${IMU_TOPIC}$"    && imu_ok=true
    "$pc_ok" && "$imu_ok" && break
    [[ $(date +%s) -lt $deadline ]] || die "Topics did not appear within ${TOPIC_WAIT_TIMEOUT}s."
    sleep 1
done
ok "Topics found: $LIDAR_TOPIC  $IMU_TOPIC"

# ── quick field check ─────────────────────────────────────────────────────────
info "Checking required point cloud fields..."
FIELDS=$(timeout 5 ros2 topic echo "$LIDAR_TOPIC" --once 2>/dev/null | grep "name:" | awk '{print $2}' | tr '\n' ' ' || true)
for f in ring timestamp; do
    echo "$FIELDS" | grep -q "$f" || warn "Field '$f' not detected. FAST-LIO2 may fail."
done
ok "Fields: $FIELDS"

# ── start ros2 bag record ─────────────────────────────────────────────────────
OUTPUT_PARENT=$(dirname "$OUTPUT")
[[ -d "$OUTPUT_PARENT" ]] || mkdir -p "$OUTPUT_PARENT"
info "Recording rosbag2 → $OUTPUT"
ros2 bag record -o "$OUTPUT" "$LIDAR_TOPIC" "$IMU_TOPIC" &
BAG_PID=$!
sleep 1

# ── wait for PCAP to finish ───────────────────────────────────────────────────
info "Recording... (waiting for PCAP playback to finish)"
silence=0
while true; do
    if timeout "$SILENCE_TIMEOUT" ros2 topic echo "$LIDAR_TOPIC" --once > /dev/null 2>&1; then
        silence=0
    else
        (( silence++ ))
        info "No data for ${silence}×${SILENCE_TIMEOUT}s..."
        [[ $silence -ge 2 ]] && break
    fi
done
ok "PCAP playback finished."

# ── stop recording ─────────────────────────────────────────────────────────────
kill "$BAG_PID" 2>/dev/null && wait "$BAG_PID" 2>/dev/null || true
BAG_PID=""
sleep 1   # allow rosbag2 to flush
ok "rosbag2 recording stopped."

# ── validate output ────────────────────────────────────────────────────────────
info "Validating output..."
[[ -d "$OUTPUT" ]] || die "Output directory not found: $OUTPUT. Recording may have failed."

BAG_INFO=$(ros2 bag info "$OUTPUT" 2>/dev/null || true)
echo "$BAG_INFO" | grep -q "$LIDAR_TOPIC" || warn "/lidar_points not found in bag."
echo "$BAG_INFO" | grep -q "$IMU_TOPIC"   || warn "/lidar_imu not found in bag."

DURATION=$(echo "$BAG_INFO" | grep -i "duration:" | awk '{print $2}' || echo "unknown")
SIZE=$(du -sh "$OUTPUT" | awk '{print $1}')
ok "Bag: $OUTPUT  |  size: $SIZE  |  duration: $DURATION"

# ── print next steps ───────────────────────────────────────────────────────────
echo ""
echo -e "${GRN}════════════════════════════════════════════════════${NC}"
echo -e "${GRN} Conversion complete!${NC}"
echo -e "${GRN}════════════════════════════════════════════════════${NC}"
echo ""
echo "Output bag: $OUTPUT"
echo ""
echo "Next steps — run FAST-LIO2:"
echo ""
echo "  Terminal 1:"
echo "    cd ~/fast_lio2_ws && source install/setup.bash"
echo "    ros2 launch fast_lio mapping_${MODEL}.launch.py"
echo ""
echo "  Terminal 2:"
echo "    ros2 bag play $OUTPUT"
echo ""
echo -e "${YEL}Notes:${NC}"
echo "  - Verify imu_gyr_unit in config/${MODEL}.yaml matches driver output (deg or rad)."
echo "  - If timestamp_unit is wrong, motion undistortion will fail."
echo "  - Run tools/check_input.py to validate before starting FAST-LIO2:"
echo "      ros2 bag play $OUTPUT &"
echo "      ros2 run fast_lio check_input.py --model $MODEL"
echo ""
