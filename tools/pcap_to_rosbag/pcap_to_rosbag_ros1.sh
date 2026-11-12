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
#   pcap_to_rosbag_ros1.sh /data/JT128/input.pcap --correction /data/calibration/device.csv
#
# The model is inferred from a JT16/JT32/JT128 path. The script also discovers
# the driver workspace and output path; calibration must be supplied explicitly. With exactly one
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
PLAY_RATE=1.0
TIMESTAMP_OFFSET=0.0
DRY_RUN=false

# ── argument parsing ─────────────────────────────────────────────────────────
usage() {
    cat <<'EOF'
Usage: pcap_to_rosbag_ros1.sh [PCAP] [options]

Quick start:
  pcap_to_rosbag_ros1.sh /data/JT128/input.pcap --correction /data/calibration/device.csv

With exactly one PCAP in a JT16/JT32/JT128 directory, PCAP may be omitted.
Defaults are printed before conversion. Use --dry-run to inspect them safely.

Options:
  --model MODEL              jt16, jt32, jt64p, or jt128 (normally inferred)
  --pcap PATH                Alternative to the positional PCAP path
  --correction PATH          Required device-matched angle correction file
  --firetime PATH            Explicit firetime correction file, if required
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
DRIVER_PID=""
BAG_PID=""
MASTER_PID=""
MASTER_LOG=""

cleanup() {
    local code=$?
    trap - EXIT INT TERM
    info "Cleaning up..."
    pcap_stop_process "$BAG_PID"
    pcap_stop_process "$DRIVER_PID"
    pcap_stop_process "$MASTER_PID"
    [[ -z "$MASTER_LOG" || "$code" != 0 ]] || rm -f -- "$MASTER_LOG"
    rm -f -- "$TMP_CONFIG"
    exit "$code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ── generate PCAP driver config ───────────────────────────────────────────────
info "Generating PCAP driver config..."
python3 - "$DRIVER_CONFIG" "$TMP_CONFIG" "$PCAP" "$CORRECTION" "$FIRETIME" "$PLAY_RATE" "$TIMESTAMP_OFFSET" "$LIDAR_TOPIC" "$IMU_TOPIC" <<'PYEOF'
import yaml, sys
source, target, pcap, correction, firetime, rate, offset, lidar_topic, imu_topic = sys.argv[1:]

with open(source) as f:
    cfg = yaml.safe_load(f)

if not cfg or "lidar" not in cfg or not cfg["lidar"]:
    print("[ERROR] Unexpected driver config structure.", file=sys.stderr)
    sys.exit(1)

cfg["lidar"] = cfg["lidar"][:1]  # Convert one selected capture; do not start unrelated live entries.
drv = cfg["lidar"][0]["driver"]
drv["source_type"] = 2
# Keep flat fields for older driver releases and pcap_type for newer releases.
drv["pcap_path"] = pcap
drv["correction_file_path"] = correction
drv["firetimes_path"] = firetime
drv["pcap_play_synchronization"] = True
drv["pcap_play_in_loop"] = False
drv["play_rate_"] = float(rate)
drv["ros_timestamp_offset"] = float(offset)
drv["pcap_type"] = {
    "pcap_path":             pcap,
    "correction_file_path":  correction,
    "firetimes_path":        firetime,
    "pcap_play_synchronization": True,
    "pcap_play_in_loop":    False,
    "play_rate_":            float(rate),
}

ros = cfg["lidar"][0].setdefault("ros", {})
ros["ros_send_point_cloud_topic"] = lidar_topic
ros["ros_send_imu_topic"]         = imu_topic
ros["send_point_cloud_ros"]       = True
ros["send_imu_ros"]               = True

drv["ros"] = dict(ros)  # Compatibility with legacy ROS 1 driver layouts.

with open(target, "w") as f:
    yaml.dump(cfg, f, default_flow_style=False)
PYEOF

ok "Config generated: $TMP_CONFIG"

# ── source driver workspace ────────────────────────────────
info "Sourcing driver workspace: $DRIVER_WS"
# shellcheck disable=SC1090
set +u
source "$DRIVER_WS/devel/setup.bash"
set -u


pcap_ensure_ros1_master

# ── start rosbag recorder before playback ────────────────────────────────────
OUTPUT_DIR=$(dirname "$OUTPUT")
[[ -d "$OUTPUT_DIR" ]] || mkdir -p "$OUTPUT_DIR"
info "Recording rosbag → $OUTPUT"
python3 "$(pcap_tool_path launch_process.py)" rosbag record -O "$OUTPUT" "$LIDAR_TOPIC" "$IMU_TOPIC" &
BAG_PID=$!
sleep 1

# ── start driver ──────────────────────────────────────────────────────────────
info "Starting Hesai ROS Driver in PCAP mode..."
python3 "$(pcap_tool_path launch_process.py)" rosrun hesai_ros_driver hesai_ros_driver_node _config_path:="$TMP_CONFIG" &
DRIVER_PID=$!

# The selected driver must exit normally at PCAP EOF (IsPlayEnded).
# Drivers that stay alive indefinitely require an EOF-capable build; silence is
# deliberately not accepted as successful conversion.
info "Waiting for driver EOF; Ctrl+C cancels without reporting success."
while kill -0 "$DRIVER_PID" 2>/dev/null; do
    kill -0 "$BAG_PID" 2>/dev/null || die "Recorder stopped before driver EOF"
    sleep 1
done
wait "$DRIVER_PID" || die "Driver exited with an error"
DRIVER_PID=""
sleep 2
ok "Driver reached EOF."

# ── stop recording ─────────────────────────────────────────────────────────────
stop_process "$BAG_PID" 30 5 || die "Recorder failed or timed out while closing the bag"
BAG_PID=""
ok "rosbag recording stopped."

# ── validate output bag ───────────────────────────────────────────────────────
info "Validating output bag..."
[[ -f "$OUTPUT" ]] || die "Output bag not found: $OUTPUT. Recording may have failed."

python3 "$(pcap_tool_path bag_metadata.py)" "$OUTPUT" --ros 1 \
    --lidar-topic "$LIDAR_TOPIC" --imu-topic "$IMU_TOPIC" --validate-timestamps
ok "Bag closed and topic counts and message timestamps validated: $OUTPUT"

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
echo "  - Prefer explicit imu_gyr_unit from the driver contract; auto is a heuristic."
echo "  - If timestamp_unit in config/${MODEL}.yaml is wrong, motion undistortion will fail."
echo "  - Run tools/check_input.py to validate the bag before starting FAST-LIO2:"
echo "      rosbag play $OUTPUT &"
echo "      python3 tools/check_input.py --model $MODEL"
echo ""
