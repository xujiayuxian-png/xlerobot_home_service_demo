#!/usr/bin/env bash
set -Eeuo pipefail

# shellcheck source=tools/lib/common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

usage() {
  cat <<'EOF'
Usage:
  tools/calibrate capture servo --hardware [--leader] [--fresh|--resume] [--config PATH] [--web-port PORT]
  tools/calibrate capture base --hardware [--fresh] [--config PATH] [--web-port PORT]
  tools/calibrate capture head-camera --hardware [--fresh|--resume] [--config PATH] [--web-port PORT]
  tools/calibrate capture right-handeye --hardware [--fresh|--resume] [--config PATH] [--web-port PORT]

Starts one live calibration capture workspace at http://127.0.0.1:8080.
The head-camera and hand-eye pages automatically capture and save passing
drafts; hand-eye includes independent held-out validation. Servo/base workflows
print a tools/calibrate command to validate the capture.
No capture workspace activates calibration or replaces the Demo runtime.
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
leader=false
fresh=false
resume=false
web_port=8080
web_host=''
hover_web_port=8082
while (($#)); do
  case $1 in
    --hardware)
      hardware=true
      shift
      ;;
    --leader)
      leader=true
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
    --web-host)
      (($# >= 2)) || die '--web-host requires a bind address'
      web_host=$2
      shift 2
      ;;
    --hover-web-port)
      (($# >= 2)) || die '--hover-web-port requires a port'
      hover_web_port=$2
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
  servo|base|head-camera|right-handeye|hover) ;;
  *) die 'capture workflow must be servo, base, head-camera, or right-handeye' ;;
esac
$fresh && $resume && die '--fresh and --resume are mutually exclusive'
$leader && [[ $workflow != servo ]] && die '--leader is only valid for servo capture'
$resume && [[ $workflow == base ]] && die '--resume is not supported for base capture'
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
$leader && workflow_id=leader_servo
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
  "state_root:=$state_root"
  "repo_root:=$repo_root"
  "task_history_root:=$history_root"
  "unit_id:=$unit"
  "right_bus:=$(config_get robot.devices.right_arm /dev/right_arm)"
  "left_bus:=$(config_get robot.devices.left_arm /dev/left_arm)"
  "web_bind_host:=${web_host:-$(config_get services.bind_host 0.0.0.0)}"
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
    if ! $leader && [[ -f $runtime/manifest.yaml ]] && verify_calibration_runtime >/dev/null; then
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
    if $leader; then
      launch_args+=("leader_only:=true" "servo_capture_directory:=leader_servo"
        "leader_port:=$(config_get robot.devices.leader_arm /dev/right_master_arm)")
      note 'Leader-only capture: six Leader IDs, no Follower/head/wheel bus opened; not hardware accepted'
      follow_up="Leader result: $work_root/result.yaml; pass it explicitly with tools/act collect --leader-calibration PATH --hardware"
    fi
    ;;
  base)
    launch_file=base_geometry_calibration.launch.py
    result="$capture_root/calibration_work/base_geometry/result.yaml"
    printf -v result_q '%q' "$result"
    follow_up="$calibrate_q base --input $result_q --config $config_q"
    ;;
  head-camera|right-handeye|hover)
    render_for=$workflow
    [[ $workflow != hover ]] || render_for=right-handeye
    "$repo_root/tools/calibrate" render --for "$render_for" \
      --config "$XLEROBOT_CONFIG" >/dev/null
    runtime="$state_root/units/$unit/draft/runtime/${render_for//-/_}"
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
    [[ $workflow != hover ]] || launch_args+=("hover_mode:=true" "hover_web_port:=$hover_web_port")
    result="$capture_root/calibration_work/$workflow_id/samples.yaml"
    printf -v result_q '%q' "$result"
    follow_up="$calibrate_q $workflow --samples $result_q --config $config_q"
    ;;
esac

if [[ $workflow == head-camera ]]; then
  note 'starting head-camera workspace; only the head holds torque; click Start in the web page for automatic motion'
elif [[ $workflow == right-handeye ]]; then
  note 'starting hand-eye workspace; only right arm/gripper and head hold torque; wheels and left arm are excluded'
  note 'click Start in the web page: 20 fitting poses, frozen fit, then 6 held-out validation poses'
else
  note "starting LIVE $workflow calibration capture; motors may be powered or move"
fi
note "capture UI: http://127.0.0.1:$web_port"
if [[ $workflow == right-handeye ]]; then
  note 'automatic capture saves a draft only after independent validation passes; active calibration is unchanged'
  note 'fit.yaml, training.yaml, heldout.yaml and validation.yaml remain in the capture directory; do not refit all samples together'
elif [[ $workflow == head-camera ]]; then
  note 'automatic capture saves a passing head-camera draft; it never activates it'
  note "optional offline re-solve after stopping capture: $follow_up"
else
  note "after capture exits, validate into the public draft with: $follow_up"
fi
exec ros2 launch xlerobot_bringup "$launch_file" "${launch_args[@]}"
