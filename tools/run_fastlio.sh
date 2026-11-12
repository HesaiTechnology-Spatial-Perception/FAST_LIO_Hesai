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
  ./tools/run_fastlio.sh jt128 pcap /data/JT128/input.pcap --correction /data/calibration/device.csv

Options:
  --ros 1|2             ROS version (normally detected from the branch/environment)
  --fastlio-ws PATH     Built FAST-LIO2 workspace
  --driver-ws PATH      Built Hesai driver workspace
  --output PATH         PCAP conversion output path
  --play-rate RATE      Bag playback speed (default: 1.0)
  --imu-gyr-unit UNIT   rad|deg|auto; overrides YAML (default: keep YAML)
  --save-map [PATH]     Save into a new run directory unless PATH is supplied
  --map-mode MODE       raw | voxel | 1hz | all (default: model YAML, normally 1hz)
  --map-leaf-size SIZE  Voxel/all export size in metres (model default: 0.1)
  --config PATH         Override the model YAML with an external user config
  --correction PATH     Required device-matched calibration in pcap mode
  --firetime PATH       Explicit firetime file for pcap mode, if required
  --timestamp-offset SEC  PCAP-only driver clock translation; use a verified value
  --log-dir PATH        Run logs, effective YAML and machine-readable results
  --drain-timeout SEC   Maximum wait for complete bag processing (default: 120)
  --save-timeout SEC    Map-save service timeout (default: 120)
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
IMU_UNIT_OVERRIDE=""
MAP_MODE=""
MAP_LEAF_SIZE=""
CONFIG_OVERRIDE=""
CORRECTION=""
FIRETIME=""
TIMESTAMP_OFFSET=""
RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
RUN_DIR=""
DRAIN_TIMEOUT=120
SAVE_TIMEOUT=120

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
        --imu-gyr-unit) require_value "$@"; IMU_UNIT_OVERRIDE="$2"; shift 2 ;;
        --map-mode) require_value "$@"; MAP_MODE="$2"; shift 2 ;;
        --map-leaf-size) require_value "$@"; MAP_LEAF_SIZE="$2"; shift 2 ;;
        --config) require_value "$@"; CONFIG_OVERRIDE="$2"; shift 2 ;;
        --correction) require_value "$@"; CORRECTION="$2"; shift 2 ;;
        --firetime) require_value "$@"; FIRETIME="$2"; shift 2 ;;
        --timestamp-offset) require_value "$@"; TIMESTAMP_OFFSET="$2"; shift 2 ;;
        --log-dir) require_value "$@"; RUN_DIR="$2"; shift 2 ;;
        --drain-timeout) require_value "$@"; DRAIN_TIMEOUT="$2"; shift 2 ;;
        --save-timeout) require_value "$@"; SAVE_TIMEOUT="$2"; shift 2 ;;
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

[[ -z "$MAP_MODE" || "$MAP_MODE" == "raw" || "$MAP_MODE" == "voxel" || "$MAP_MODE" == "1hz" || "$MAP_MODE" == "all" ]] || die "Invalid --map-mode"
[[ "$SAVE_MAP" == true || ( -z "$MAP_MODE" && -z "$MAP_LEAF_SIZE" ) ]] || die "Map options require --save-map"
[[ -z "$CONFIG_OVERRIDE" || -f "$CONFIG_OVERRIDE" ]] || die "Config not found: $CONFIG_OVERRIDE"
[[ "$MODE" != "pcap" || -n "$CORRECTION" ]] || die "pcap mode requires --correction for this device"
for seconds in "$DRAIN_TIMEOUT" "$SAVE_TIMEOUT"; do
    [[ "$seconds" =~ ^[1-9][0-9]*$ ]] || die "Timeouts must be positive whole seconds"
done
[[ "$MODE" == "live" || -n "$INPUT" ]] || die "$MODE mode requires an input path"
[[ -z "$IMU_UNIT_OVERRIDE" || "$IMU_UNIT_OVERRIDE" == "rad" || "$IMU_UNIT_OVERRIDE" == "deg" || "$IMU_UNIT_OVERRIDE" == "auto" ]] || \
    die "--imu-gyr-unit must be rad, deg, or auto"
[[ "$MODE" == "live" || -e "$INPUT" ]] || die "Input not found: $INPUT"
[[ "$MODEL" != "mt60" || "$MODE" != "pcap" ]] || \
    die "MT60 PCAP conversion is not registered; use live mode or an existing rosbag"
[[ "$MODE" == "pcap" || -z "$TIMESTAMP_OFFSET" ]] || die "--timestamp-offset is only valid in pcap mode"
[[ "$MODE" == "pcap" || -z "$OUTPUT" ]] || die "--output is only valid in pcap mode"
[[ "$PLAY_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "--play-rate must be a positive number"
awk -v rate="$PLAY_RATE" 'BEGIN { exit !(rate > 0) }' || die "--play-rate must be greater than zero"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/process_control.sh"
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
    MAP_PATH="$HOME/slam_ws/src/slam_maps/fastlio_${MODEL}_${RUN_ID}/map.pcd"
fi
[[ -z "$MAP_PATH" || "$MAP_PATH" == *.pcd ]] || MAP_PATH="${MAP_PATH}.pcd"
[[ -z "$MAP_PATH" ]] || MAP_PATH=$(readlink -m "$MAP_PATH")

[[ -n "$RUN_DIR" ]] || RUN_DIR="$HOME/slam_ws/logs/fastlio_${MODEL}_${RUN_ID}"
RUN_DIR=$(readlink -m "$RUN_DIR")
CONVERTER_ARGS=()
[[ -z "$CORRECTION" ]] || CONVERTER_ARGS+=(--correction "$CORRECTION")
[[ -z "$FIRETIME" ]] || CONVERTER_ARGS+=(--firetime "$FIRETIME")
[[ -z "$TIMESTAMP_OFFSET" ]] || CONVERTER_ARGS+=(--timestamp-offset "$TIMESTAMP_OFFSET")
info "run logs:    $RUN_DIR"
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
info "IMU unit:    ${IMU_UNIT_OVERRIDE:-keep YAML; auto uses the node multi-sample heuristic}"

if [[ "$DRY_RUN" == true ]]; then
    if [[ "$MODE" == "pcap" ]]; then
        "$CONVERTER" "$INPUT" --model "$MODEL" --driver-ws "$DRIVER_WS" --output "$OUTPUT" --play-rate "$PLAY_RATE" "${CONVERTER_ARGS[@]}" --dry-run
    fi
    info "Dry run complete; nothing was started or changed."
    exit 0
fi

source_workspace
# Installed tools live under lib/fast_lio; assets live in the package share.
ASSET_ROOT="$REPO_ROOT"
if [[ ! -d "$ASSET_ROOT/config" ]]; then
    if [[ "$ROS_MAJOR" == 2 ]]; then ASSET_ROOT=$(ros2 pkg prefix --share fast_lio)
    else ASSET_ROOT=$(rospack find fast_lio); fi
fi

FASTLIO_PID=""
DRIVER_PID=""
TEMP_CONFIG=""
MAP_SAVE_ATTEMPTED=false
PLAYER_PID=""
RVIZ_PID=""
MAP_SAVED=false
RUN_COMPLETE=false

prepare_map_config() {
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
    [[ -z "${CONFIG_OVERRIDE:-}" ]] || source_config="$CONFIG_OVERRIDE"
    [[ -f "$source_config" ]] || die "Cannot find config/${MODEL}.yaml for runtime configuration"
    [[ -z "$MAP_PATH" ]] || mkdir -p "$(dirname "$MAP_PATH")"
    TEMP_CONFIG=$(mktemp "/tmp/fast_lio_${MODEL}_save_XXXXXX.yaml")
    local config_args=("$source_config" "$TEMP_CONFIG" --ros "$ROS_MAJOR")
    [[ -z "$IMU_UNIT_OVERRIDE" ]] || config_args+=(--imu-gyr-unit "$IMU_UNIT_OVERRIDE")
    [[ -z "$MAP_PATH" ]] || config_args+=(--save-map "$MAP_PATH")
    [[ -z "${MAP_MODE:-}" ]] || config_args+=(--map-mode "$MAP_MODE")
    [[ -z "${MAP_LEAF_SIZE:-}" ]] || config_args+=(--map-leaf-size "$MAP_LEAF_SIZE")
    [[ -z "${RUN_ID:-}" ]] || config_args+=(--run-id "$RUN_ID")
    [[ "${MODE:-live}" == live || "$ROS_MAJOR" == 1 ]] || config_args+=(--reliable-input)
    [[ -z "${RUN_DIR:-}" ]] || config_args+=(--log-dir "$RUN_DIR")
    [[ "${RVIZ:-true}" == true ]] || config_args+=(--no-preview)
    python3 "$SCRIPT_DIR/prepare_runtime_config.py" "${config_args[@]}"
}

save_map_now() {
    [[ "$SAVE_MAP" == true && "$MAP_SAVED" != true ]] || return 0
    MAP_SAVE_ATTEMPTED=true
    info "Saving and verifying map files..."
    timeout --signal=TERM --kill-after=5 "$((SAVE_TIMEOUT + 35))" python3 "$SCRIPT_DIR/mapping_control.py" save --ros "$ROS_MAJOR" \
        --run-id "$RUN_ID" --timeout "$SAVE_TIMEOUT" --output "$RUN_DIR/map_save.json" || return 1
    MAP_SAVED=true
}


cleanup() {
    local result=$?
    trap - EXIT INT TERM
    set +e
    stop_process "$PLAYER_PID"; [[ "$?" != 124 || "$result" != 0 ]] || result=124
    stop_process "$DRIVER_PID"; [[ "$?" != 124 || "$result" != 0 ]] || result=124
    if [[ "$SAVE_MAP" == true && "$MAP_SAVE_ATTEMPTED" == false && -n "$FASTLIO_PID" ]]; then
        save_map_now || { [[ "$result" != 0 ]] || result=1; }
    fi
    stop_process "$FASTLIO_PID"
    local node_result=$?
    if [[ -n "$FASTLIO_PID" ]]; then
        python3 - "$RUN_DIR/node_status.json" "$RUN_ID" <<'STATUS'
import json,sys
try:
    status=json.load(open(sys.argv[1]))
    if status.get("run_id") != sys.argv[2]: raise ValueError("Node status run ID mismatch")
    sys.exit(int(status["exit_code"]))
except (OSError,ValueError,KeyError) as error:
    print("[ERROR] Missing/invalid final node status:",error,file=sys.stderr)
    sys.exit(1)
STATUS
        local status_result=$?
        [[ "$status_result" == 0 ]] || node_result="$status_result"
    fi
    [[ "$node_result" == 0 || "$result" != 0 ]] || result="$node_result"
    stop_process "$RVIZ_PID"
    [[ -z "$TEMP_CONFIG" ]] || rm -f -- "$TEMP_CONFIG"
    python3 - "$RUN_DIR" "$result" "$RUN_COMPLETE" "$MAP_SAVED" <<'RESULT'
import json,sys
from pathlib import Path
folder=Path(sys.argv[1])
result={'exit_code':int(sys.argv[2]),'input_complete':sys.argv[3]=='true',
        'map_saved':sys.argv[4]=='true','quality_assessed':False}
for name in ('input_metadata','drain','map_save','node_status'):
    path=folder/(name+'.json')
    if path.exists():
        try: result[name]=json.loads(path.read_text())
        except ValueError: result[name]={'error':'invalid result JSON'}
(folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
RESULT
    info "Run result: $RUN_DIR/result.json"
    exit "$result"
}

mkdir -p "$RUN_DIR"
[[ ! -e "$RUN_DIR/result.json" ]] || die "Run directory already contains results: $RUN_DIR"
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Validate configuration before starting a driver or converting a PCAP.
prepare_map_config
cp "$TEMP_CONFIG" "$RUN_DIR/effective.yaml"
python3 "$SCRIPT_DIR/check_config.py" --config "$TEMP_CONFIG" --model "$MODEL" --ros "$ROS_MAJOR"
readarray -t INPUT_TOPICS < <(python3 - "$TEMP_CONFIG" "$ROS_MAJOR" <<'TOPICS'
import sys,yaml
with open(sys.argv[1]) as f: p=yaml.safe_load(f)
if sys.argv[2]=='2': p=p['/**']['ros__parameters']
print(p['common'].get('lid_topic','/lidar_points'))
print(p['common'].get('imu_topic','/lidar_imu'))
TOPICS
)
LIDAR_TOPIC="${INPUT_TOPICS[0]}"
IMU_TOPIC="${INPUT_TOPICS[1]}"
python3 - "$REPO_ROOT" "$RUN_DIR" "$MODEL" "$MODE" "$INPUT" "$RUN_ID" <<'RUN'
import json,subprocess,sys
from pathlib import Path
repo,folder,model,mode,source,run_id=sys.argv[1:]
def git(*args):
    p=subprocess.run(['git','-C',repo,*args],text=True,capture_output=True)
    return p.stdout.strip() if p.returncode==0 else None
Path(folder,'run.json').write_text(json.dumps({'model':model,'mode':mode,'input':source,'run_id':run_id,
    'branch':git('branch','--show-current'),'commit':git('rev-parse','HEAD'),
    'working_tree_dirty':bool(git('status','--porcelain'))},indent=2)+'\n')
RUN

start_fastlio() {
    info "Starting FAST-LIO2 ($MODEL); log: $RUN_DIR/mapping.log"
    if [[ "$ROS_MAJOR" == "1" ]]; then
        python3 "$SCRIPT_DIR/launch_process.py" roslaunch fast_lio "mapping_${MODEL}.launch" \
            rviz:=false config_file:="$TEMP_CONFIG" > "$RUN_DIR/mapping.log" 2>&1 &
    else
        python3 "$SCRIPT_DIR/launch_process.py" \
            "$FASTLIO_WS/install/fast_lio/lib/fast_lio/fastlio_mapping" \
            --ros-args --params-file "$TEMP_CONFIG" > "$RUN_DIR/mapping.log" 2>&1 &
    fi
    FASTLIO_PID=$!
    timeout --signal=TERM --kill-after=5 35 python3 "$SCRIPT_DIR/mapping_control.py" ready --ros "$ROS_MAJOR" --run-id "$RUN_ID" --timeout 30
    if [[ "$RVIZ" == true ]]; then
        if [[ "$ROS_MAJOR" == "1" ]]; then
            python3 "$SCRIPT_DIR/launch_process.py" rosrun rviz rviz -d "$ASSET_ROOT/rviz_cfg/loam_livox.rviz" > "$RUN_DIR/rviz.log" 2>&1 &
        else
            python3 "$SCRIPT_DIR/launch_process.py" ros2 run rviz2 rviz2 -d "$ASSET_ROOT/rviz/fastlio.rviz" > "$RUN_DIR/rviz.log" 2>&1 &
        fi
        RVIZ_PID=$!
    fi
}

start_driver() {
    info "Starting Hesai driver..."
    set +u
    if [[ "$ROS_MAJOR" == "1" ]]; then
        source "$DRIVER_WS/devel/setup.bash"
        set -u
        python3 "$SCRIPT_DIR/launch_process.py" rosrun hesai_ros_driver hesai_ros_driver_node > "$RUN_DIR/driver.log" 2>&1 &
    else
        source "$DRIVER_WS/install/setup.bash"
        set -u
        python3 "$SCRIPT_DIR/launch_process.py" ros2 run hesai_ros_driver hesai_ros_driver_node > "$RUN_DIR/driver.log" 2>&1 &
    fi
    DRIVER_PID=$!
}

play_bag() {
    info "Playing bag: $INPUT"
    if [[ "$ROS_MAJOR" == "1" ]]; then
        python3 "$SCRIPT_DIR/launch_process.py" rosbag play --quiet --wait-for-subscribers -r "$PLAY_RATE" \
            "$INPUT" --topics "$LIDAR_TOPIC" "$IMU_TOPIC" > "$RUN_DIR/playback.log" 2>&1 &
    else
        local player_name="fastlio_player_${RUN_ID}"
        python3 - "$RUN_DIR/player_qos.yaml" "$LIDAR_TOPIC" "$IMU_TOPIC" <<'QOS'
import sys,yaml
with open(sys.argv[1],'w') as stream:
    yaml.safe_dump({topic: {'reliability':'reliable','durability':'volatile',
                          'history':'keep_last','depth':1000} for topic in sys.argv[2:]},stream)
QOS
        python3 "$SCRIPT_DIR/launch_process.py" ros2 bag play --rate "$PLAY_RATE" --start-paused \
            --disable-keyboard-controls --qos-profile-overrides-path "$RUN_DIR/player_qos.yaml" "$INPUT" --topics "$LIDAR_TOPIC" "$IMU_TOPIC" \
            --remap "__node:=$player_name" > "$RUN_DIR/playback.log" 2>&1 &
        PLAYER_PID=$!
        python3 "$SCRIPT_DIR/mapping_control.py" resume --ros 2 --run-id "$RUN_ID" --timeout 30 \
            --player-name "$player_name" --lidar-topic "$LIDAR_TOPIC" --imu-topic "$IMU_TOPIC"
    fi
    [[ -n "$PLAYER_PID" ]] || PLAYER_PID=$!
    wait "$PLAYER_PID" || die "Bag player failed; see $RUN_DIR/playback.log"
    PLAYER_PID=""
}

finish_bag_processing() {
    timeout --signal=TERM --kill-after=5 "$((DRAIN_TIMEOUT + 35))" python3 "$SCRIPT_DIR/mapping_control.py" drain --ros "$ROS_MAJOR" --run-id "$RUN_ID" \
        --metadata "$RUN_DIR/input_metadata.json" --timeout "$DRAIN_TIMEOUT" --output "$RUN_DIR/drain.json"
    RUN_COMPLETE=true
}

case "$MODE" in
    live)
        start_fastlio
        start_driver
        info "Mapping is running. Press Ctrl+C to save and stop."
        wait_live "$FASTLIO_PID" "$DRIVER_PID"
        ;;
    bag|pcap)
        if [[ "$MODE" == "pcap" ]]; then
            "$CONVERTER" "$INPUT" --model "$MODEL" --driver-ws "$DRIVER_WS" --output "$OUTPUT" \
                --play-rate "$PLAY_RATE" "${CONVERTER_ARGS[@]}" --lidar-topic "$LIDAR_TOPIC" --imu-topic "$IMU_TOPIC"
            INPUT="$OUTPUT"
        fi
        python3 "$SCRIPT_DIR/bag_metadata.py" "$INPUT" --ros "$ROS_MAJOR" \
            --lidar-topic "$LIDAR_TOPIC" --imu-topic "$IMU_TOPIC" --validate-timestamps > "$RUN_DIR/input_metadata.json"
        start_fastlio
        play_bag
        finish_bag_processing
        save_map_now
        ;;
esac
