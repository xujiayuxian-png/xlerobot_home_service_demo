#!/usr/bin/env bash

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)

die() {
  echo "ERROR: $*" >&2
  exit 2
}

note() {
  echo "==> $*"
}

gpu_driver_available() {
  local executable
  executable=$(command -v nvidia-smi || true)
  # Non-interactive SSH sessions on WSL may omit this directory from PATH.
  if [[ -z $executable && -x /usr/lib/wsl/lib/nvidia-smi ]]; then
    executable=/usr/lib/wsl/lib/nvidia-smi
  fi
  [[ -n $executable ]] && "$executable" >/dev/null 2>&1
}

source_environment() {
  # ROS and Python activation scripts are not guaranteed to be nounset-safe.
  # Preserve the caller's setting instead of weakening it for the rest of the
  # public command.
  local environment_file=$1
  local restore_nounset=false
  case $- in
    *u*) restore_nounset=true; set +u ;;
  esac
  # shellcheck disable=SC1090
  source "$environment_file"
  if $restore_nounset; then
    set -u
  fi
}

load_local_env() {
  # A command-line/environment override set by the caller outranks the
  # repository-local .env. Save it before sourcing that file.
  local requested_config=${XLEROBOT_CONFIG-}
  if [[ -f "$repo_root/.env" ]]; then
    set -a
    source_environment "$repo_root/.env"
    set +a
  fi
  if [[ -n $requested_config ]]; then
    XLEROBOT_CONFIG=$requested_config
  else
    : "${XLEROBOT_CONFIG:=$repo_root/config/local.yaml}"
  fi
  if [[ $XLEROBOT_CONFIG != /* ]]; then
    XLEROBOT_CONFIG="$repo_root/$XLEROBOT_CONFIG"
  fi
  export XLEROBOT_CONFIG
}

config_python() {
  # Configuration is a host-side concern. In particular, the isolated ACT
  # environment intentionally contains only inference dependencies and does
  # not own the public YAML parser used by every entry point.
  command -v python3 || die 'python3 is required'
}

require_config() {
  [[ -f $XLEROBOT_CONFIG ]] || die \
    "missing $XLEROBOT_CONFIG; copy config/local.example.yaml to config/local.yaml"
  local schema
  schema=$(config_get schema '') || die 'PyYAML is required; run tools/setup first'
  [[ $schema == xlerobot_demo/v1 ]] || die \
    "unsupported config schema '${schema:-missing}' (expected xlerobot_demo/v1)"
  local robot_unit calibration_unit collection_root dataset_id dataset_root expected
  robot_unit=$(config_get robot.unit_id '')
  calibration_unit=$(config_get calibration.unit '')
  [[ -n $robot_unit && $robot_unit == "$calibration_unit" ]] || die \
    'robot.unit_id and calibration.unit must be the same nonempty unit identity'
  collection_root=$(config_get data.collection_root '')
  dataset_id=$(config_get data.dataset_id '')
  dataset_root=$(config_get data.dataset_root '')
  [[ -n $collection_root && -n $dataset_id && -n $dataset_root ]] || die \
    'data.collection_root, data.dataset_id, and data.dataset_root are required'
  expected="${collection_root%/}/datasets/$dataset_id"
  [[ $dataset_root == "$expected" ]] || die \
    "data.dataset_root must equal data.collection_root/datasets/data.dataset_id ($expected)"
  local required_key required_value
  for required_key in \
    robot.site_id \
    robot.devices.lidar \
    robot.devices.right_arm \
    robot.devices.left_arm \
    robot.devices.leader_arm \
    robot.devices.wrist_camera \
    site.map \
    site.places \
    services.lm_studio_url \
    services.classical_url \
    services.act_url \
    services.bind_host \
    transfer.ssh_host \
    transfer.ssh_user \
    transfer.remote_repo_path \
    models.vlm \
    models.person_detector \
    models.act_training_output \
    data.repo_id \
    demo.object_id \
    demo.source_place \
    demo.recipient_id; do
    required_value=$(config_get "$required_key" '')
    [[ -n $required_value ]] || die "config value $required_key is required"
  done
  local service_url
  for required_key in services.lm_studio_url services.classical_url services.act_url; do
    service_url=$(config_get "$required_key" '')
    [[ $service_url == http://* || $service_url == https://* ]] || die \
      "config value $required_key must be an HTTP(S) URL"
  done
  local backend voice_enabled web_enabled web_port act_checkpoint act_manifest
  backend=$(config_get demo.grasp_backend '')
  case $backend in act|centroid|gpd) ;; *) die \
    'demo.grasp_backend must be act, centroid, or gpd' ;; esac
  voice_enabled=$(config_get demo.voice '')
  web_enabled=$(config_get demo.web '')
  [[ $voice_enabled == true || $voice_enabled == false ]] || die \
    'demo.voice must be true or false'
  [[ $web_enabled == true || $web_enabled == false ]] || die \
    'demo.web must be true or false'
  web_port=$(config_get demo.web_port '')
  if [[ ! $web_port =~ ^[0-9]+$ ]] || \
      ((web_port < 1 || web_port > 65535)); then
    die 'demo.web_port must be an integer from 1 to 65535'
  fi
  act_checkpoint=$(config_get models.act_checkpoint '')
  act_manifest=$(config_get models.act_manifest '')
  [[ -n $act_checkpoint && -n $act_manifest ]] || die \
    'models.act_checkpoint and models.act_manifest are required'
}

source_robot_workspace() {
  [[ -f /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed'
  [[ -f $repo_root/ros2_ws/install/setup.bash ]] || die \
    'workspace is not built; run tools/setup robot'
  source_environment /opt/ros/jazzy/setup.bash
  source_environment "$repo_root/ros2_ws/install/setup.bash"
  if [[ -f $repo_root/.venv/robot/bin/activate ]]; then
    source_environment "$repo_root/.venv/robot/bin/activate"
  fi
}

verify_calibration_runtime() {
  local package_root="$repo_root/ros2_ws/src/xlerobot_calibration_tools"
  PYTHONPATH="$package_root${PYTHONPATH:+:$PYTHONPATH}" \
    XLEROBOT_DEMO_ROOT="$repo_root" \
    python3 -m xlerobot_calibration_tools.runtime_gate \
      --repo-root "$repo_root" --config "$XLEROBOT_CONFIG"
}

verify_act_checkpoint() {
  local checkpoint=$1
  local manifest
  manifest=$(absolute_from_repo "$(config_get models.act_manifest '')")
  [[ -n $checkpoint && -n $manifest ]] || die \
    'models.act_checkpoint and models.act_manifest are required'
  python3 "$repo_root/tools/lib/verify_model_files.py" \
    --manifest "$manifest" \
    --root "$checkpoint"
}

voice_kws_dir() {
  local value=.xlerobot/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20
  if [[ -f ${XLEROBOT_CONFIG:-} ]]; then
    value=$(config_get models.kws_dir "$value")
  fi
  absolute_from_repo "$value"
}

voice_whisper_dir() {
  local value=.xlerobot/models/faster-whisper-small
  if [[ -f ${XLEROBOT_CONFIG:-} ]]; then
    value=$(config_get models.whisper_dir "$value")
  fi
  absolute_from_repo "$value"
}

voice_models_command() {
  local python=$1
  shift
  "$python" "$repo_root/tools/lib/voice_models.py" \
    --manifest "$repo_root/assets/models/voice-runtime.manifest.json" \
    --kws-dir "$(voice_kws_dir)" \
    --whisper-dir "$(voice_whisper_dir)" \
    "$@"
}

voice_model_config_is_pinned() {
  [[ $(config_get models.kws_model_id '') == \
      sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20 ]] &&
    [[ $(config_get models.whisper_repo_id '') == \
      Systran/faster-whisper-small ]] &&
    [[ $(config_get models.whisper_revision '') == \
      536b0662742c02347bc0e980a01041f333bce120 ]]
}

vlm_config_is_pinned() {
  [[ $(config_get models.vlm '') == qwen/qwen3-vl-4b ]] &&
    [[ $(config_get models.vlm_lm_studio_version '') == 0.4.12+1 ]] &&
    [[ $(config_get models.vlm_lm_studio_revision '') == 4 ]] &&
    [[ $(config_get models.vlm_descriptor_sha256 '') == \
      e8869e92a4e59986e9aeac41fb91491d4267ab7b13aa0dbe3d7fd7c6da8e3db3 ]] &&
    [[ $(config_get models.vlm_artifact_repo_id '') == \
      lmstudio-community/Qwen3-VL-4B-Instruct-GGUF ]] &&
    [[ $(config_get models.vlm_artifact_revision '') == \
      9eaf9988fe9b5e33541dc614622c36d5e90dd509 ]] &&
    [[ $(config_get models.vlm_quantization '') == Q4_K_M ]]
}

person_model_path() {
  local value=.xlerobot/models/yolov8n.pt
  if [[ -f ${XLEROBOT_CONFIG:-} ]]; then
    value=$(config_get models.person_detector "$value")
  fi
  absolute_from_repo "$value"
}

person_model_command() {
  local python=$1
  shift
  "$python" "$repo_root/tools/lib/person_model.py" \
    --manifest "$repo_root/assets/models/manifest.yaml" \
    --output "$(person_model_path)" \
    "$@"
}

config_get() {
  local key=$1
  local default=${2-}
  local python
  python=$(config_python)
  "$python" - "$XLEROBOT_CONFIG" "$key" "$default" <<'PY'
import os
from pathlib import Path
import sys

try:
    import yaml
except ImportError as exc:
    raise SystemExit('PyYAML is not installed') from exc

path = Path(sys.argv[1])
key = sys.argv[2]
default = sys.argv[3]
try:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
except (OSError, yaml.YAMLError) as exc:
    raise SystemExit(f'cannot read config {path}: {exc}') from exc
value = data
for part in key.split('.'):
    if not isinstance(value, dict) or part not in value:
        value = default
        break
    value = value[part]
if isinstance(value, bool):
    print('true' if value else 'false')
elif value is None:
    print(default)
elif isinstance(value, (str, int, float)):
    print(os.path.expandvars(os.path.expanduser(str(value))))
else:
    raise SystemExit(f'config value {key} must be a scalar')
PY
}

absolute_from_repo() {
  local value=$1
  if [[ $value == /* ]]; then
    printf '%s\n' "$value"
  else
    printf '%s\n' "$repo_root/$value"
  fi
}

url_join() {
  local base=${1%/}
  local suffix=${2#/}
  printf '%s/%s\n' "$base" "$suffix"
}

lm_model_available() {
  local url=$1 expected=$2
  local payload
  payload=$(curl --silent --fail --max-time 5 "$url") || return 1
  python3 -c '
import json
import sys

expected = sys.argv[1]
try:
    payload = json.load(sys.stdin)
except (json.JSONDecodeError, OSError, TypeError, ValueError):
    raise SystemExit(1)
models = payload.get("data", []) if isinstance(payload, dict) else []
identifiers = {
    str(item.get("id", "")) for item in models if isinstance(item, dict)
}
raise SystemExit(0 if expected in identifiers else 1)
' "$expected" <<<"$payload" 2>/dev/null
}
