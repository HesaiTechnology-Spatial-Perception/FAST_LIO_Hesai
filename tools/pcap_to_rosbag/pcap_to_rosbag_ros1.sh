#!/usr/bin/env bash
# Copyright 2026 Hesai Technology. All rights reserved.
# SPDX-License-Identifier: GPL-2.0
#
# pcap_to_rosbag_ros1.sh
# Convert a Hesai PCAP file to a FAST-LIO2-ready rosbag (ROS 1).
#
# Pipeline:
#   input.pcap
#     → Hesai ROS Driver (source_type: 2)
#     → /lidar_points + /lidar_imu
#     → rosbag record
#     → output.bag
#
# Quick start:
#   pcap_to_rosbag_ros1.sh /data/JT128/input.pcap
#
# The model is inferred from a JT16/JT32/JT128 path. The script also discovers
# the driver workspace, calibration files, and output path. With exactly one
# PCAP in the current directory, no argument is needed. Use --help to see all
# overrides and --dry-run to inspect the resolved values without starting ROS.
#
# Requirements:
#   - ROS 1 (Melodic or Noetic) sourced
#   - Hesai ROS Driver installed in --driver-ws
#   - Python 3 with PyYAML  (pip3 install pyyaml)

set -euo pipefail

# ── colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YEL='\033[0;33m'; BLU='\033[0;34m'; NC='\033[0m'
info()  { echo -e "${BLU}[INFO]${NC}  $*"; }
ok()    { echo -e "${GRN}[OK]${NC}    $*"; }
warn()  { echo -e "${YEL}[WARN]${NC}  $*"; }
die()   { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

# ── defaults ─────────────────────────────────────────────────────────────────
MODEL=""
PCAP=""
CORRECTION=""
FIRETIME=""
OUTPUT=""
DRIVER_WS=""
LIDAR_TOPIC="/lidar_points"
IMU_TOPIC="/lidar_imu"
TOPIC_WAIT_TIMEOUT=30      # seconds to wait for topics to appear
SILENCE_TIMEOUT=4          # seconds of silence to declare PCAP done
PLAY_RATE=1.0
TIMESTAMP_OFFSET=0.0
DRY_RUN=false

# ── argument parsing ─────────────────────────────────────────────────────────
usage() {
    cat <<'EOF'
Usage: pcap_to_rosbag_ros1.sh [PCAP] [options]

Quick start:
  pcap_to_rosbag_ros1.sh /data/JT128/input.pcap

With exactly one PCAP in a JT16/JT32/JT128 directory, PCAP may be omitted.
Defaults are printed before conversion. Use --dry-run to inspect them safely.

Options:
  --model MODEL              jt16, jt32, jt64p, or jt128 (normally inferred)
  --pcap PATH                Alternative to the positional PCAP path
  --correction PATH          Angle correction file (normally discovered)
  --firetime PATH            Firetime correction file (normally discovered)
  --output PATH              Output .bag path
  --driver-ws PATH           Hesai ROS 1 driver workspace
  --play-rate RATE           PCAP playback speed (default: 1.0)
  --timestamp-offset SEC     Add seconds to driver timestamps (default: 0)
  --lidar-topic TOPIC        Point cloud topic (default: /lidar_points)
  --imu-topic TOPIC          IMU topic (default: /lidar_imu)
  --dry-run                  Resolve inputs only; do not start ROS
  -h, --help                 Show this help

The driver workspace may also be set with HESAI_DRIVER_WS.
EOF
    exit 0
}

command -v python3    >/dev/null 2>&1 || die "python3 not found."
python3 -c "import yaml" 2>/dev/null  || die "PyYAML not found. Run: pip3 install pyyaml"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
pcap_parse_args "$@"
pcap_resolve_defaults 1

if [[ "$DRY_RUN" == true ]]; then
    ok "Dry run complete; ROS was not started and no files were changed."
    exit 0
fi

[[ ! -e "$OUTPUT" ]] || die "Output already exists: $OUTPUT"
[[ -f "$DRIVER_WS/devel/setup.bash" ]] || die "ROS 1 driver workspace is not built: $DRIVER_WS/devel/setup.bash missing"
command -v roslaunch  >/dev/null 2>&1 || die "roslaunch not found. Source your ROS workspace."
command -v rosbag     >/dev/null 2>&1 || die "rosbag not found."
command -v rostopic   >/dev/null 2>&1 || die "rostopic not found."

info "Driver config: $DRIVER_CONFIG"

# ── temp files and cleanup ────────────────────────────────────────────────────
TMP_CONFIG=$(mktemp /tmp/hesai_pcap_config_XXXXXX.yaml)
CONFIG_BACKUP="${DRIVER_CONFIG}.pcap_tool_backup"
DRIVER_PID=""
BAG_PID=""

cleanup() {
    info "Cleaning up..."
    [[ -n "$BAG_PID" ]]    && kill "$BAG_PID"    2>/dev/null || true
    [[ -n "$DRIVER_PID" ]] && kill "$DRIVER_PID" 2>/dev/null || true
    # restore original config
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
# Keep flat fields for older driver releases and pcap_type for newer releases.
drv["pcap_path"] = "$PCAP"
drv["correction_file_path"] = "$CORRECTION"
drv["firetimes_path"] = "$FIRETIME"
drv["pcap_play_synchronization"] = True
drv["pcap_play_in_loop"] = False
drv["play_rate_"] = $PLAY_RATE
drv["ros_timestamp_offset"] = $TIMESTAMP_OFFSET
drv["pcap_type"] = {
    "pcap_path":             "$PCAP",
    "correction_file_path":  "$CORRECTION",
    "firetimes_path":        "$FIRETIME",
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

ok "Config generated: $TMP_CONFIG"

# ── source driver workspace and back up config ────────────────────────────────
info "Sourcing driver workspace: $DRIVER_WS"
# shellcheck disable=SC1090
set +u
source "$DRIVER_WS/devel/setup.bash"
set -u

cp "$DRIVER_CONFIG" "$CONFIG_BACKUP"
cp "$TMP_CONFIG"    "$DRIVER_CONFIG"
info "Backed up original config → ${CONFIG_BACKUP}"

# ── start rosbag recorder before playback ────────────────────────────────────
OUTPUT_DIR=$(dirname "$OUTPUT")
[[ -d "$OUTPUT_DIR" ]] || mkdir -p "$OUTPUT_DIR"
info "Recording rosbag → $OUTPUT"
rosbag record -O "$OUTPUT" "$LIDAR_TOPIC" "$IMU_TOPIC" &
BAG_PID=$!
sleep 1

# ── start driver ──────────────────────────────────────────────────────────────
info "Starting Hesai ROS Driver in PCAP mode..."
roslaunch hesai_ros_driver start.launch &
DRIVER_PID=$!

# ── wait for topics ───────────────────────────────────────────────────────────
info "Waiting for topics (timeout: ${TOPIC_WAIT_TIMEOUT}s)..."
deadline=$(( $(date +%s) + TOPIC_WAIT_TIMEOUT ))
while true; do
    pc_ok=false; imu_ok=false
    topics=$(rostopic list 2>/dev/null || true)
    echo "$topics" | grep -q "^${LIDAR_TOPIC}$"  && pc_ok=true
    echo "$topics" | grep -q "^${IMU_TOPIC}$"    && imu_ok=true
    "$pc_ok" && "$imu_ok" && break
    [[ $(date +%s) -lt $deadline ]] || die "Topics did not appear within ${TOPIC_WAIT_TIMEOUT}s. Check driver config and PCAP file."
    sleep 1
done
ok "Topics found: $LIDAR_TOPIC  $IMU_TOPIC"

# ── quick field check ─────────────────────────────────────────────────────────
info "Checking required point cloud fields..."
FIELDS=$(timeout 5 rostopic echo -n 1 "$LIDAR_TOPIC" 2>/dev/null | grep "name:" | awk '{print $2}' | tr '\n' ' ' || true)
for f in ring timestamp; do
    echo "$FIELDS" | grep -q "$f" || warn "Field '$f' not detected. FAST-LIO2 may fail."
done
ok "Fields: $FIELDS"

# ── wait for PCAP to finish ───────────────────────────────────────────────────
info "Recording... (waiting for PCAP playback to finish)"
silence=0
while true; do
    if timeout "$SILENCE_TIMEOUT" rostopic echo -n 1 "$LIDAR_TOPIC" > /dev/null 2>&1; then
        silence=0
    else
        (( silence += 1 ))
        info "No data for ${silence}×${SILENCE_TIMEOUT}s..."
        [[ $silence -ge 2 ]] && break
    fi
done
ok "PCAP playback finished."

# ── stop recording ─────────────────────────────────────────────────────────────
kill "$BAG_PID" 2>/dev/null && wait "$BAG_PID" 2>/dev/null || true
BAG_PID=""
ok "rosbag recording stopped."

# ── validate output bag ───────────────────────────────────────────────────────
info "Validating output bag..."
[[ -f "$OUTPUT" ]] || die "Output bag not found: $OUTPUT. Recording may have failed."

BAG_INFO=$(rosbag info "$OUTPUT" 2>/dev/null)
echo "$BAG_INFO" | grep -q "$LIDAR_TOPIC" || warn "/lidar_points not found in bag."
echo "$BAG_INFO" | grep -q "$IMU_TOPIC"   || warn "/lidar_imu not found in bag."
echo "$BAG_INFO" | grep "$LIDAR_TOPIC" | grep -Eq 'messages: [1-9][0-9]*' || die "$LIDAR_TOPIC has no messages in bag."
echo "$BAG_INFO" | grep "$IMU_TOPIC" | grep -Eq 'messages: [1-9][0-9]*' || die "$IMU_TOPIC has no messages in bag."

DURATION=$(echo "$BAG_INFO" | grep "duration:" | awk '{print $2}')
SIZE=$(du -sh "$OUTPUT" | awk '{print $1}')
ok "Bag: $OUTPUT  |  size: $SIZE  |  duration: $DURATION"

# ── print next steps ──────────────────────────────────────────────────────────
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
echo "    cd ~/fast_lio_ws && source devel/setup.bash"
echo "    roslaunch fast_lio mapping_${MODEL}.launch"
echo ""
echo "  Terminal 2:"
echo "    rosbag play $OUTPUT"
echo ""
echo -e "${YEL}Notes:${NC}"
echo "  - Keep imu_gyr_unit: auto (recommended); use deg/rad only as a manual override."
echo "  - If timestamp_unit in config/${MODEL}.yaml is wrong, motion undistortion will fail."
echo "  - Run tools/check_input.py to validate the bag before starting FAST-LIO2:"
echo "      rosbag play $OUTPUT &"
echo "      python3 tools/check_input.py --model $MODEL"
echo ""
