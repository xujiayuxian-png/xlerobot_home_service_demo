#!/usr/bin/env bash
set -euo pipefail

service_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
gpd_root="${GPD_ROOT:-${service_root}/../../.xlerobot/vendor/gpd}"
gpd_commit="6327f20eabfcba41a05fdd2e2ba408153dc2e958"

if [[ ! -d "${gpd_root}/.git" ]]; then
  mkdir -p "$(dirname "${gpd_root}")"
  git clone https://github.com/atenpas/gpd.git "${gpd_root}"
fi

actual_remote="$(git -C "${gpd_root}" remote get-url origin)"
if [[ "${actual_remote}" != "https://github.com/atenpas/gpd.git" && \
      "${actual_remote}" != "git@github.com:atenpas/gpd.git" ]]; then
  echo "Refusing to modify unexpected GPD checkout: ${actual_remote}" >&2
  exit 2
fi

git -C "${gpd_root}" fetch origin "${gpd_commit}"
git -C "${gpd_root}" checkout --detach "${gpd_commit}"
install -m 0644 \
  "${service_root}/patches/detect_grasps_json.cpp" \
  "${gpd_root}/src/detect_grasps_json.cpp"

if git -C "${gpd_root}" apply --check "${service_root}/patches/gpd-cmake.patch"; then
  git -C "${gpd_root}" apply "${service_root}/patches/gpd-cmake.patch"
elif ! git -C "${gpd_root}" apply --reverse --check \
  "${service_root}/patches/gpd-cmake.patch"; then
  echo "GPD CMake patch does not match pinned commit" >&2
  exit 3
fi

cmake -S "${gpd_root}" -B "${gpd_root}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${gpd_root}/build" --target gpd_detect_grasps_json --parallel
echo "Built ${gpd_root}/build/gpd_detect_grasps_json"
