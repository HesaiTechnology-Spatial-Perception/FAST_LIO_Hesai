#!/usr/bin/env bash
# Shared argument parsing and safe defaults for the PCAP conversion scripts.
# This file is sourced by pcap_to_rosbag_ros1.sh / pcap_to_rosbag_ros2.sh.

pcap_require_value() {
    [[ $# -ge 2 && -n "${2:-}" ]] || die "$1 requires a value"
}

pcap_parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --model)            pcap_require_value "$@"; MODEL="$2"; shift 2 ;;
            --pcap)             pcap_require_value "$@"; PCAP="$2"; shift 2 ;;
            --correction)       pcap_require_value "$@"; CORRECTION="$2"; shift 2 ;;
            --firetime)         pcap_require_value "$@"; FIRETIME="$2"; shift 2 ;;
            --output)           pcap_require_value "$@"; OUTPUT="$2"; shift 2 ;;
            --driver-ws)        pcap_require_value "$@"; DRIVER_WS="$2"; shift 2 ;;
            --play-rate)        pcap_require_value "$@"; PLAY_RATE="$2"; shift 2 ;;
            --timestamp-offset) pcap_require_value "$@"; TIMESTAMP_OFFSET="$2"; shift 2 ;;
            --lidar-topic)      pcap_require_value "$@"; LIDAR_TOPIC="$2"; shift 2 ;;
            --imu-topic)        pcap_require_value "$@"; IMU_TOPIC="$2"; shift 2 ;;
            --dry-run)          DRY_RUN=true; shift ;;
            -h|--help)          usage ;;
            -*)                 die "Unknown option: $1 (run with --help)" ;;
            *)
                [[ -z "$PCAP" ]] || die "Only one positional PCAP path is allowed"
                PCAP="$1"
                shift
                ;;
        esac
    done
}

pcap_find_driver_config() {
    local workspace="$1"
    local candidate
    for candidate in \
        "$workspace/src/HesaiLidar_ROS_2.0/config/config.yaml" \
        "$workspace/src/hesai_ros_driver/config/config.yaml" \
        "$workspace/src/HesaiLidar_ROS/config/config.yaml" \
        "$workspace/src/HesaiLidar_ROS2/config/config.yaml"; do
        if [[ -f "$candidate" ]]; then
            DRIVER_CONFIG="$candidate"
            return 0
        fi
    done
    return 1
}

pcap_pick_single_file() {
    local label="$1"
    shift
    local matches=()
    local item
    for item in "$@"; do
        [[ -f "$item" ]] && matches+=("$item")
    done
    if [[ ${#matches[@]} -eq 1 ]]; then
        printf '%s\n' "${matches[0]}"
        return 0
    fi
    if [[ ${#matches[@]} -gt 1 ]]; then
        warn "Multiple ${label} files found; choose one explicitly:" >&2
        printf '  %s\n' "${matches[@]}" >&2
    fi
    return 1
}

pcap_config_paths() {
    local key="$1"
    python3 - "$DRIVER_CONFIG" "$key" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as stream:
    cfg = yaml.safe_load(stream) or {}
drv = ((cfg.get("lidar") or [{}])[0].get("driver") or {})
key = sys.argv[2]
for section in (drv, drv.get("pcap_type") or {}, drv.get("lidar_udp_type") or {}):
    value = section.get(key, "")
    if isinstance(value, str) and value:
        print(value)
PY
}

# File names and the driver's previous configuration cannot prove device identity.
# Calibration selection is explicit, including when a single candidate exists.
pcap_find_calibration() {
    warn "Calibration cannot be verified from MODEL alone; pass a device-matched --correction (and --firetime if required)." >&2
    return 1
}

pcap_tool_path() {
    if [[ -f "$SCRIPT_DIR/../$1" ]]; then
        printf '%s\n' "$SCRIPT_DIR/../$1"
    else
        printf '%s\n' "$SCRIPT_DIR/$1"
    fi
}

pcap_resolve_defaults() {
    local ros_major="$1"

    [[ "$PLAY_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
        die "--play-rate must be a positive number"
    awk -v rate="$PLAY_RATE" 'BEGIN { exit !(rate > 0) }' || die "--play-rate must be greater than zero"
    [[ "$TIMESTAMP_OFFSET" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || \
        die "--timestamp-offset must be a number"

    if [[ -z "$PCAP" ]]; then
        shopt -s nullglob
        local local_pcaps=("$PWD"/*.pcap "$PWD"/*.PCAP)
        shopt -u nullglob
        PCAP=$(pcap_pick_single_file "PCAP" "${local_pcaps[@]}") || \
            die "No unique PCAP found in $PWD; pass the file path as the first argument"
    fi
    [[ -f "$PCAP" ]] || die "PCAP file not found: $PCAP"
    PCAP=$(readlink -f "$PCAP")

    if [[ -z "$MODEL" ]]; then
        local lower_path
        lower_path=$(printf '%s' "$PCAP" | tr '[:upper:]' '[:lower:]')
        case "$lower_path" in
            *jt128*) MODEL="jt128" ;;
            *jt32*)  MODEL="jt32" ;;
            *jt64p*) MODEL="jt64p" ;;
            *jt16*)  MODEL="jt16" ;;
            *) die "Cannot infer LiDAR model from '$PCAP'; pass --model jt16|jt32|jt64p|jt128" ;;
        esac
    fi
    [[ "$MODEL" == "jt16" || "$MODEL" == "jt32" || "$MODEL" == "jt64p" || "$MODEL" == "jt128" ]] || \
        die "--model must be jt16, jt32, jt64p, or jt128"

    if [[ -z "$DRIVER_WS" && -n "${HESAI_DRIVER_WS:-}" ]]; then
        DRIVER_WS="$HESAI_DRIVER_WS"
    fi
    if [[ -z "$DRIVER_WS" ]]; then
        local candidates=("$PWD" "$(dirname "$PWD")")
        if [[ "$ros_major" == "1" ]]; then
            candidates+=("${HOME}/hesai_ros_ws" "${HOME}/catkin_ws" "${HOME}/fastlio2_for_JT")
        else
            candidates+=("${HOME}/hesai_ros2_ws" "${HOME}/ros2_ws" "${HOME}/livox_ws")
        fi
        local workspace
        for workspace in "${candidates[@]}"; do
            if [[ -d "$workspace" ]] && pcap_find_driver_config "$workspace"; then
                DRIVER_WS="$workspace"
                break
            fi
        done
    fi
    [[ -n "$DRIVER_WS" && -d "$DRIVER_WS" ]] || \
        die "Cannot auto-find the Hesai driver workspace; set HESAI_DRIVER_WS or pass --driver-ws"
    DRIVER_WS=$(readlink -f "$DRIVER_WS")
    pcap_find_driver_config "$DRIVER_WS" || \
        die "Cannot find Hesai driver config.yaml under $DRIVER_WS/src"

    [[ -n "$CORRECTION" ]] || die "Pass --correction with the calibration file matching this device; automatic calibration selection is disabled"
    [[ -f "$CORRECTION" ]] || die "Correction file not found: $CORRECTION"
    CORRECTION=$(readlink -f "$CORRECTION")
    [[ -z "$FIRETIME" || -f "$FIRETIME" ]] || die "Firetime file not found: $FIRETIME"
    [[ -z "$FIRETIME" ]] || FIRETIME=$(readlink -f "$FIRETIME")

    if [[ -z "$OUTPUT" ]]; then
        if [[ "$ros_major" == "1" ]]; then
            OUTPUT="${PCAP%.*}_ros1.bag"
        else
            OUTPUT="${PCAP%.*}_ros2_bag"
        fi
    elif [[ "$ros_major" == "1" && "$OUTPUT" != *.bag ]]; then
        OUTPUT="${OUTPUT}.bag"
    fi
    OUTPUT=$(readlink -m "$OUTPUT")

    info "Resolved conversion inputs:"
    printf '  model:      %s\n' "$MODEL"
    printf '  pcap:       %s\n' "$PCAP"
    printf '  correction: %s\n' "$CORRECTION"
    printf '  firetime:   %s\n' "${FIRETIME:-not required}"
    printf '  driver ws:  %s\n' "$DRIVER_WS"
    printf '  output:     %s\n' "$OUTPUT"
}


# Share the bounded process-group shutdown with the mapping entry point.
source "$(pcap_tool_path process_control.sh)"

pcap_stop_process() {
    stop_process "$1" 10 2 || true
}

pcap_ros1_master_ready() {
    python3 - <<'MASTER_CHECK'
import os, socket, sys, xmlrpc.client
socket.setdefaulttimeout(1.0)
try:
    result = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/fastlio_pcap_probe')
    sys.exit(0 if result[0] == 1 else 1)
except (OSError, xmlrpc.client.Error):
    sys.exit(1)
MASTER_CHECK
}

pcap_ensure_ros1_master() {
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
    pcap_ros1_master_ready && return 0
    local port deadline
    port=$(python3 - <<'MASTER_PORT'
import os,socket,sys
from urllib.parse import urlsplit
uri=urlsplit(os.environ['ROS_MASTER_URI'])
local={'localhost','127.0.0.1','::1',socket.gethostname(),socket.getfqdn()}
if uri.scheme != 'http' or uri.hostname not in local:
    sys.exit('ROS master is unavailable and not local; start the configured ROS_MASTER_URI server.')
print(uri.port or 11311)
MASTER_PORT
    ) || die "Cannot start ROS master for $ROS_MASTER_URI"
    MASTER_LOG=$(mktemp /tmp/fastlio_roscore_XXXXXX.log)
    info "Starting owned ROS master at $ROS_MASTER_URI (log: $MASTER_LOG)"
    python3 "$(pcap_tool_path launch_process.py)" roscore -p "$port" > "$MASTER_LOG" 2>&1 &
    MASTER_PID=$!
    deadline=$((SECONDS + 15))
    while ! pcap_ros1_master_ready; do
        kill -0 "$MASTER_PID" 2>/dev/null || die "ROS master exited; see $MASTER_LOG"
        (( SECONDS < deadline )) || die "ROS master readiness timed out; see $MASTER_LOG"
        sleep 0.2
    done
}
