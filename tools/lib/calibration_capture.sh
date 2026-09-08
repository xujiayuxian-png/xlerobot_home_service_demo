#!/usr/bin/env bash
set -Eeuo pipefail

# shellcheck source=tools/lib/common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

usage() {
  cat <<'EOF'
Usage:
  tools/calibrate capture servo --hardware [--fresh] [--config PATH] [--web-port PORT]
  tools/calibrate capture base --hardware [--fresh] [--config PATH] [--web-port PORT]
  tools/calibrate capture head-camera --hardware [--fresh|--resume] [--config PATH] [--web-port PORT]
  tools/calibrate capture right-handeye --hardware [--fresh|--resume] [--config PATH] [--web-port PORT]

Starts one live calibration capture workspace at http://127.0.0.1:8080.
The command only collects raw measurements; the normal tools/calibrate
subcommands perform the strict public validation and save the public draft.
EOF
}

if (($# == 0)); then
  usage
  exit 2
fi
if [[ $1 == -h || $1 == --help ]]; then
  usage
  exit 0
fi

workflow=$1
shift
hardware=false
fresh=false
resume=false
web_port=8080
while (($#)); do
  case $1 in
    --hardware)
      hardware=true
      shift
      ;;
    --fresh)
      fresh=true
      shift
      ;;
    --resume)
      resume=true
      shift
      ;;
    --config)
      (($# >= 2)) || die '--config requires a path'
      XLEROBOT_CONFIG=$2
      shift 2
      ;;
    --web-port)
      (($# >= 2)) || die '--web-port requires a TCP port'
      web_port=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown calibration capture option: $1"
      ;;
  esac
done

case $workflow in
  servo|base|head-camera|right-handeye) ;;
  *) die 'capture workflow must be servo, base, head-camera, or right-handeye' ;;
esac
$fresh && $resume && die '--fresh and --resume are mutually exclusive'
$resume && [[ $workflow != head-camera && $workflow != right-handeye ]] && \
  die '--resume is valid only for head-camera or right-handeye samples'
if ! [[ $web_port =~ ^[0-9]+$ ]] || ((web_port < 1 || web_port > 65535)); then
  die '--web-port must be an integer from 1 to 65535'
fi
$hardware || die \
  'live calibration requires the explicit --hardware flag; no device was opened'

load_local_env
require_config

state_root=$(absolute_from_repo "$(config_get calibration.state_root .xlerobot)")
unit=$(config_get robot.unit_id '')
[[ $unit =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] || \
  die 'robot.unit_id must contain only letters, digits, dot, underscore, or hyphen'
capture_root="$state_root/units/$unit/capture"
history_root="$capture_root/logs"
workflow_id=${workflow//-/_}
work_root="$capture_root/calibration_work/$workflow_id"
if $fresh && [[ -e $work_root || -L $work_root ]]; then
  archive_root="$capture_root/archive"
  archived="$archive_root/${workflow_id}-$(date -u +%Y%m%dT%H%M%SZ)"
  [[ ! -e $archived && ! -L $archived ]] || die \
    "capture archive already exists: $archived"
  mkdir -p "$archive_root"
  mv -- "$work_root" "$archived"
  note "previous capture archived without deletion: $archived"
fi

existing="$work_root/result.yaml"
if [[ $workflow == head-camera || $workflow == right-handeye ]]; then
  existing="$work_root/samples.yaml"
fi
if ! $fresh && ! $resume && [[ -e $existing || -L $existing ]]; then
  die "existing $workflow capture found at $existing; use --resume to append visual samples or --fresh to archive it"
fi

mkdir -p "$capture_root" "$history_root"
source_robot_workspace
printf -v calibrate_q '%q' "$repo_root/tools/calibrate"
printf -v config_q '%q' "$XLEROBOT_CONFIG"

launch_file=''
follow_up=''
launch_args=(
  "hardware_enabled:=true"
  "artifact_root:=$capture_root"
  "task_history_root:=$history_root"
  "unit_id:=$unit"
  "right_bus:=$(config_get robot.devices.right_arm /dev/right_arm)"
  "left_bus:=$(config_get robot.devices.left_arm /dev/left_arm)"
  "web_bind_host:=$(config_get services.bind_host 0.0.0.0)"
  "web_port:=$web_port"
)

serial=$(config_get robot.devices.head_camera_serial '')
[[ -z $serial ]] || launch_args+=("d455_serial:=$serial")

case $workflow in
  servo)
    launch_file=servo_calibration.launch.py
    # Offer the known zero only from this unit's verified active runtime.
    # This remains an explicit web choice; no EEPROM values are written.
    runtime="$state_root/units/$unit/runtime"
    if [[ -f $runtime/manifest.yaml ]] && verify_calibration_runtime >/dev/null; then
      existing_version=$(python3 - "$runtime/manifest.yaml" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    print(yaml.safe_load(stream)['active_version'])
PY
      )
      launch_args+=(
        "existing_servo_file:=$runtime/servos.yaml"
        "existing_servo_version:=$existing_version"
      )
      note "optional existing zero reference: $existing_version (not applied automatically)"
    fi
    result="$capture_root/calibration_work/servo/result.yaml"
    printf -v result_q '%q' "$result"
    follow_up="$calibrate_q servo --input $result_q --config $config_q"
    ;;
  base)
    launch_file=base_geometry_calibration.launch.py
    result="$capture_root/calibration_work/base_geometry/result.yaml"
    printf -v result_q '%q' "$result"
    follow_up="$calibrate_q base --input $result_q --config $config_q"
    ;;
  head-camera|right-handeye)
    "$repo_root/tools/calibrate" render --for "$workflow" \
      --config "$XLEROBOT_CONFIG" >/dev/null
    runtime="$state_root/units/$unit/draft/runtime/$workflow_id"
    launch_args+=(
      "geometry_file:=$runtime/geometry.yaml"
      "servo_calibration_file:=$runtime/servos.yaml"
      "controllers_file:=$runtime/controllers.yaml"
    )
    if [[ $workflow == head-camera ]]; then
      launch_file=d455_extrinsic_calibration.launch.py
    else
      launch_file=right_handeye_calibration.launch.py
    fi
    result="$capture_root/calibration_work/$workflow_id/samples.yaml"
    printf -v result_q '%q' "$result"
    follow_up="$calibrate_q $workflow --samples $result_q --config $config_q"
    ;;
esac

note "starting LIVE $workflow calibration capture; motors may be powered or move"
note "capture UI: http://127.0.0.1:$web_port"
note "after capture exits, validate into the public draft with: $follow_up"
exec ros2 launch xlerobot_bringup "$launch_file" "${launch_args[@]}"
