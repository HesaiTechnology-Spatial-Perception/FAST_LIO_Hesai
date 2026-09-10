#!/usr/bin/env bash
# One-command customer entry point for Hesai LiDAR FAST-LIO2.

set -euo pipefail

info() { printf '[INFO] %s\n' "$*"; }
die()  { printf '[ERROR] %s\n' "$*" >&2; exit 1; }

usage() {
    cat <<'EOF'
Usage: run_fastlio.sh MODEL MODE [INPUT] [options]

MODEL:
  jt16 | jt32 | jt64p | jt128 | mt60

MODE:
  live          Run the Hesai driver and FAST-LIO2
  bag PATH      Run FAST-LIO2 and play an existing rosbag
  pcap PATH     Convert the PCAP, then run FAST-LIO2 and play the bag

Examples:
  ./tools/run_fastlio.sh jt128 live
  ./tools/run_fastlio.sh jt32 bag /data/jt32.bag
  ./tools/run_fastlio.sh mt60 bag /data/mt60.bag
  ./tools/run_fastlio.sh jt128 pcap /data/JT128/input.pcap

Options:
  --ros 1|2             ROS version (normally detected from the branch/environment)
  --fastlio-ws PATH     Built FAST-LIO2 workspace
  --driver-ws PATH      Built Hesai driver workspace
  --output PATH         PCAP conversion output path
  --play-rate RATE      Bag playback speed (default: 1.0)
  --save-map [PATH]     Save a PCD map (default: ~/slam_ws/src/slam_maps/...)
  --no-rviz             Run without RViz
  --dry-run             Print resolved actions without starting anything
  -h, --help            Show this help

Environment alternatives: FAST_LIO_WS, HESAI_DRIVER_WS.
EOF
    exit 0
}

# ── Customer arguments ───────────────────────────────────────────────────────
[[ $# -gt 0 ]] || usage
[[ "$1" != "-h" && "$1" != "--help" ]] || usage

MODEL="$1"
shift
[[ "$MODEL" == "jt16" || "$MODEL" == "jt32" || "$MODEL" == "jt64p" || "$MODEL" == "jt128" || "$MODEL" == "mt60" ]] || \
    die "MODEL must be jt16, jt32, jt64p, jt128, or mt60"

[[ $# -gt 0 ]] || die "MODE is required: live, bag, or pcap"
MODE="$1"
shift
[[ "$MODE" == "live" || "$MODE" == "bag" || "$MODE" == "pcap" ]] || \
    die "MODE must be live, bag, or pcap"

INPUT=""
if [[ "$MODE" != "live" && $# -gt 0 && "$1" != -* ]]; then
    INPUT="$1"
    shift
fi

ROS_MAJOR=""
FASTLIO_WS="${FAST_LIO_WS:-}"
DRIVER_WS="${HESAI_DRIVER_WS:-}"
OUTPUT=""
PLAY_RATE=1.0
SAVE_MAP=false
MAP_PATH=""
RVIZ=true
DRY_RUN=false

require_value() {
    [[ $# -ge 2 && -n "${2:-}" ]] || die "$1 requires a value"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ros)        require_value "$@"; ROS_MAJOR="$2"; shift 2 ;;
        --fastlio-ws) require_value "$@"; FASTLIO_WS="$2"; shift 2 ;;
        --driver-ws)  require_value "$@"; DRIVER_WS="$2"; shift 2 ;;
        --output)     require_value "$@"; OUTPUT="$2"; shift 2 ;;
        --play-rate)  require_value "$@"; PLAY_RATE="$2"; shift 2 ;;
        --save-map)
            SAVE_MAP=true
            if [[ $# -ge 2 && "$2" != -* ]]; then
                MAP_PATH="$2"
                shift 2
            else
                shift
            fi
            ;;
        --no-rviz)    RVIZ=false; shift ;;
        --dry-run)    DRY_RUN=true; shift ;;
        -h|--help)    usage ;;
        *)            die "Unknown option: $1 (run with --help)" ;;
    esac
done

[[ "$MODE" == "live" || -n "$INPUT" ]] || die "$MODE mode requires an input path"
[[ "$MODE" == "live" || -e "$INPUT" ]] || die "Input not found: $INPUT"
[[ "$MODEL" != "mt60" || "$MODE" != "pcap" ]] || \
    die "MT60 PCAP conversion is not registered; use live mode or an existing rosbag"
[[ "$MODE" == "pcap" || -z "$OUTPUT" ]] || die "--output is only valid in pcap mode"
[[ "$PLAY_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "--play-rate must be a positive number"
awk -v rate="$PLAY_RATE" 'BEGIN { exit !(rate > 0) }' || die "--play-rate must be greater than zero"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# ── Environment discovery ────────────────────────────────────────────────────
detect_ros() {
    if [[ -n "$ROS_MAJOR" ]]; then
        [[ "$ROS_MAJOR" == "1" || "$ROS_MAJOR" == "2" ]] || die "--ros must be 1 or 2"
        return
    fi
    if [[ -f "$REPO_ROOT/launch/mapping_${MODEL}.launch.py" ]]; then
        ROS_MAJOR=2
    elif [[ -f "$REPO_ROOT/launch/mapping_${MODEL}.launch" ]]; then
        ROS_MAJOR=1
    elif [[ "${ROS_VERSION:-}" == "1" || "${ROS_VERSION:-}" == "2" ]]; then
        ROS_MAJOR="$ROS_VERSION"
    elif command -v ros2 >/dev/null 2>&1 && ! command -v rosversion >/dev/null 2>&1; then
        ROS_MAJOR=2
    elif command -v rosversion >/dev/null 2>&1 && ! command -v ros2 >/dev/null 2>&1; then
        ROS_MAJOR=1
    else
        die "Cannot detect ROS version; pass --ros 1 or --ros 2"
    fi
}

workspace_is_built() {
    local workspace="$1"
    if [[ "$ROS_MAJOR" == "1" ]]; then
        [[ -f "$workspace/devel/setup.bash" || -f "$workspace/install/setup.bash" ]] && \
            [[ -x "$workspace/devel/lib/fast_lio/fastlio_mapping" || -x "$workspace/install/lib/fast_lio/fastlio_mapping" ]]
    else
        [[ -f "$workspace/install/setup.bash" ]] && \
            [[ -x "$workspace/install/fast_lio/lib/fast_lio/fastlio_mapping" ]] && \
            [[ -f "$workspace/install/fast_lio/share/fast_lio/launch/mapping_${MODEL}.launch.py" ]]
    fi
}

find_fastlio_workspace() {
    if [[ -n "$FASTLIO_WS" ]]; then
        [[ -d "$FASTLIO_WS" ]] || die "FAST-LIO2 workspace not found: $FASTLIO_WS"
        workspace_is_built "$FASTLIO_WS" || \
            die "FAST-LIO2 workspace is not built for $MODEL on ROS $ROS_MAJOR: $FASTLIO_WS"
        FASTLIO_WS=$(readlink -f "$FASTLIO_WS")
        return
    fi

    local current="$REPO_ROOT" candidate
    for _ in 1 2 3 4; do
        current=$(dirname "$current")
        workspace_is_built "$current" && { FASTLIO_WS="$current"; return; }
    done

    if [[ "$ROS_MAJOR" == "1" ]]; then
        for candidate in "$HOME/slam_ws" "$HOME/fast_lio_ws" "$HOME/fastlio2_for_JT" "$HOME/catkin_ws"; do
            workspace_is_built "$candidate" && { FASTLIO_WS="$candidate"; return; }
        done
    else
        for candidate in "$HOME/slam_ws" "$HOME/fast_lio_ws" "$HOME/ros2_ws"; do
            workspace_is_built "$candidate" && { FASTLIO_WS="$candidate"; return; }
        done
    fi
    die "Cannot find a built FAST-LIO2 workspace; pass --fastlio-ws or set FAST_LIO_WS"
}

find_driver_workspace() {
    [[ "$MODE" == "bag" ]] && return
    if [[ -n "$DRIVER_WS" ]]; then
        [[ -d "$DRIVER_WS" ]] || die "Hesai driver workspace not found: $DRIVER_WS"
        DRIVER_WS=$(readlink -f "$DRIVER_WS")
        return
    fi
    local candidate
    if [[ "$ROS_MAJOR" == "1" ]]; then
        for candidate in "$FASTLIO_WS" "$HOME/hesai_ros_ws" "$HOME/fastlio2_for_JT" "$HOME/catkin_ws"; do
            [[ -f "$candidate/devel/setup.bash" && -d "$candidate/src/hesai_ros_driver" ]] && \
                { DRIVER_WS="$candidate"; return; }
        done
    else
        for candidate in "$FASTLIO_WS" "$HOME/hesai_ros2_ws" "$HOME/ros2_ws"; do
            [[ -f "$candidate/install/setup.bash" ]] || continue
            [[ -d "$candidate/src/HesaiLidar_ROS_2.0" || -d "$candidate/src/hesai_ros_driver" ]] && \
                { DRIVER_WS="$candidate"; return; }
        done
    fi
    die "Cannot find a built Hesai driver workspace; pass --driver-ws or set HESAI_DRIVER_WS"
}

source_workspace() {
    local setup
    if [[ "$ROS_MAJOR" == "1" ]]; then
        setup="$FASTLIO_WS/devel/setup.bash"
        [[ -f "$setup" ]] || setup="$FASTLIO_WS/install/setup.bash"
    else
        setup="$FASTLIO_WS/install/setup.bash"
    fi
    set +u
    # shellcheck disable=SC1090
    source "$setup"
    set -u
}

find_converter() {
    local name="pcap_to_rosbag_ros${ROS_MAJOR}.sh"
    if [[ -x "$SCRIPT_DIR/pcap_to_rosbag/$name" ]]; then
        CONVERTER="$SCRIPT_DIR/pcap_to_rosbag/$name"
    elif [[ -x "$SCRIPT_DIR/$name" ]]; then
        CONVERTER="$SCRIPT_DIR/$name"
    else
        die "PCAP converter not found: $name"
    fi
}

detect_ros
find_fastlio_workspace
find_driver_workspace

if [[ "$MODE" == "pcap" ]]; then
    find_converter
    if [[ -z "$OUTPUT" ]]; then
        [[ "$ROS_MAJOR" == "1" ]] && OUTPUT="${INPUT%.*}_ros1.bag" || OUTPUT="${INPUT%.*}_ros2_bag"
    fi
fi

if [[ "$SAVE_MAP" == true && -z "$MAP_PATH" ]]; then
    MAP_PATH="$HOME/slam_ws/src/slam_maps/fast_lio_${MODEL}_$(date +%Y%m%d_%H%M%S).pcd"
fi
[[ -z "$MAP_PATH" || "$MAP_PATH" == *.pcd ]] || MAP_PATH="${MAP_PATH}.pcd"
[[ -z "$MAP_PATH" ]] || MAP_PATH=$(readlink -m "$MAP_PATH")

info "model:       $MODEL"
info "mode:        $MODE"
info "ROS:         $ROS_MAJOR"
info "FAST-LIO ws: $FASTLIO_WS"
[[ -z "$DRIVER_WS" ]] || info "driver ws:   $DRIVER_WS"
[[ -z "$INPUT" ]] || info "input:       $INPUT"
[[ -z "$OUTPUT" ]] || info "output:      $OUTPUT"
[[ -z "$MAP_PATH" ]] || info "PCD map:     $MAP_PATH"
info "RViz:        $RVIZ"
info "play rate:   $PLAY_RATE"

if [[ "$DRY_RUN" == true ]]; then
    if [[ "$MODE" == "pcap" ]]; then
        "$CONVERTER" "$INPUT" --model "$MODEL" --driver-ws "$DRIVER_WS" --output "$OUTPUT" --play-rate "$PLAY_RATE" --dry-run
    fi
    info "Dry run complete; nothing was started or changed."
    exit 0
fi

source_workspace

FASTLIO_PID=""
DRIVER_PID=""
TEMP_CONFIG=""
MAP_SAVE_ATTEMPTED=false
IMU_UNIT_OVERRIDE=""

detect_bag_imu_unit() {
    [[ "$MODE" == "bag" || "$MODE" == "pcap" ]] || return 0
    info "Detecting IMU units from the bag before starting FAST-LIO2..."
    if [[ "$ROS_MAJOR" == "1" ]]; then
        IMU_UNIT_OVERRIDE=$(python3 - "$INPUT" <<'PY' | tail -n 1
import math, rosbag, sys
with rosbag.Bag(sys.argv[1]) as bag:
    for _, msg, _ in bag.read_messages(topics=['/lidar_imu']):
        a = msg.linear_acceleration
        norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        if math.isfinite(norm) and norm >= 0.1:
            print('deg' if norm < 4.0 else 'rad')
            break
PY
        )
    else
        IMU_UNIT_OVERRIDE=$(python3 - "$INPUT" <<'PY' | tail -n 1
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic != '/lidar_imu':
        continue
    msg = deserialize_message(data, Imu)
    a = msg.linear_acceleration
    norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
    if math.isfinite(norm) and norm >= 0.1:
        print('deg' if norm < 4.0 else 'rad')
        break
PY
        )
    fi
    [[ "$IMU_UNIT_OVERRIDE" == "deg" || "$IMU_UNIT_OVERRIDE" == "rad" ]] || \
        die "Cannot determine IMU units from $INPUT; verify /lidar_imu or use live mode"
    info "Bag IMU unit: $IMU_UNIT_OVERRIDE"
}

prepare_map_config() {
    if [[ "$SAVE_MAP" != true && -z "$IMU_UNIT_OVERRIDE" ]]; then
        return 0
    fi
    local source_config
    if [[ "$MODEL" == "mt60" ]]; then
        source_config="$REPO_ROOT/config/MT60.yaml"
    else
        source_config="$REPO_ROOT/config/${MODEL}.yaml"
    fi
    if [[ ! -f "$source_config" ]]; then
        local config_name="${MODEL}.yaml"
        [[ "$MODEL" != "mt60" ]] || config_name="MT60.yaml"
        source_config=$(find "$FASTLIO_WS" -type f -path "*/fast_lio/config/${config_name}" -print -quit 2>/dev/null)
    fi
    [[ -f "$source_config" ]] || die "Cannot find config/${MODEL}.yaml for map saving"
    mkdir -p "$(dirname "$MAP_PATH")"
    TEMP_CONFIG=$(mktemp "/tmp/fast_lio_${MODEL}_save_XXXXXX.yaml")
    python3 - "$source_config" "$TEMP_CONFIG" "$ROS_MAJOR" "$MAP_PATH" "$IMU_UNIT_OVERRIDE" <<'PY'
import sys
import yaml

source, target, ros_major, map_path, imu_unit = sys.argv[1:]
with open(source, encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
params = config if ros_major == "1" else config["/**"]["ros__parameters"]
if imu_unit:
    params.setdefault("common", {})["imu_gyr_unit"] = imu_unit
if map_path:
    params["map_file_path"] = map_path
    pcd = params.setdefault("pcd_save", {})
    pcd["pcd_save_en"] = True
    pcd.setdefault("interval", -1)
    pcd.setdefault("leaf_size", 0.0)
with open(target, "w", encoding="utf-8") as stream:
    yaml.safe_dump(config, stream, sort_keys=False)
PY
}

save_map_now() {
    if [[ "$SAVE_MAP" != true || "$MAP_SAVE_ATTEMPTED" != false || -z "$FASTLIO_PID" ]]; then
        return 0
    fi
    kill -0 "$FASTLIO_PID" 2>/dev/null || return 0
    MAP_SAVE_ATTEMPTED=true
    info "Saving PCD map to $MAP_PATH..."
    if [[ "$ROS_MAJOR" == "1" ]]; then
        timeout 20 rosservice call /map_save "{}" || true
    else
        timeout 20 ros2 service call /map_save std_srvs/srv/Trigger "{}" || true
    fi
}

cleanup() {
    save_map_now
    [[ -z "$FASTLIO_PID" ]] || kill "$FASTLIO_PID" 2>/dev/null || true
    [[ -z "$DRIVER_PID" ]] || kill "$DRIVER_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    [[ -z "$TEMP_CONFIG" ]] || rm -f "$TEMP_CONFIG"
}
trap cleanup EXIT INT TERM

if [[ "$MODE" != "pcap" ]]; then
    detect_bag_imu_unit
    prepare_map_config
fi

# ── Process lifecycle ────────────────────────────────────────────────────────
start_fastlio() {
    info "Starting FAST-LIO2 ($MODEL)..."
    if [[ "$ROS_MAJOR" == "1" ]]; then
        if [[ -n "$TEMP_CONFIG" ]]; then
            roslaunch fast_lio "mapping_${MODEL}.launch" rviz:="$RVIZ" config_file:="$TEMP_CONFIG" &
        else
            roslaunch fast_lio "mapping_${MODEL}.launch" rviz:="$RVIZ" &
        fi
    else
        if [[ -n "$TEMP_CONFIG" ]]; then
            ros2 launch fast_lio "mapping_${MODEL}.launch.py" rviz:="$RVIZ" config_file:="$TEMP_CONFIG" &
        else
            ros2 launch fast_lio "mapping_${MODEL}.launch.py" rviz:="$RVIZ" &
        fi
    fi
    FASTLIO_PID=$!
    sleep 3
    kill -0 "$FASTLIO_PID" 2>/dev/null || die "FAST-LIO2 failed to start"
}

start_driver() {
    info "Starting Hesai driver..."
    set +u
    if [[ "$ROS_MAJOR" == "1" ]]; then
        # shellcheck disable=SC1090
        source "$DRIVER_WS/devel/setup.bash"
        set -u
        roslaunch hesai_ros_driver start.launch &
    else
        # shellcheck disable=SC1090
        source "$DRIVER_WS/install/setup.bash"
        set -u
        ros2 run hesai_ros_driver hesai_ros_driver_node &
    fi
    DRIVER_PID=$!
    sleep 3
    kill -0 "$DRIVER_PID" 2>/dev/null || die "Hesai driver failed to start"
}

play_bag() {
    info "Playing bag: $INPUT"
    if [[ "$ROS_MAJOR" == "1" ]]; then
        rosbag play --quiet -r "$PLAY_RATE" "$INPUT"
    else
        ros2 bag play --rate "$PLAY_RATE" "$INPUT"
    fi
}

finish_bag_processing() {
    info "Bag playback finished; waiting 5 seconds for FAST-LIO2 to drain queued sensor data..."
    sleep 5
    kill -0 "$FASTLIO_PID" 2>/dev/null || die "FAST-LIO2 stopped before queued bag data was processed"
}

case "$MODE" in
    live)
        start_driver
        start_fastlio
        info "Mapping is running. Press Ctrl+C to stop."
        wait "$FASTLIO_PID"
        ;;
    bag)
        start_fastlio
        play_bag
        finish_bag_processing
        save_map_now
        ;;
    pcap)
        info "Converting PCAP to rosbag..."
        "$CONVERTER" "$INPUT" --model "$MODEL" --driver-ws "$DRIVER_WS" --output "$OUTPUT" --play-rate "$PLAY_RATE"
        INPUT="$OUTPUT"
        detect_bag_imu_unit
        prepare_map_config
        start_fastlio
        play_bag
        finish_bag_processing
        save_map_now
        ;;
esac
