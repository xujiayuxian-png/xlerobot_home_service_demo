#!/usr/bin/env bash
# Variables are consumed by the public tools sourcing this file.
# shellcheck disable=SC2034

# Deliberately limited to the reference laptop and the native X1 migration.
select_robot_platform() {
  local os_id=$1 os_version=$2 architecture=$3
  case "$os_id/$os_version/$architecture" in
    ubuntu/24.04/x86_64)
      robot_ros_distro=jazzy
      robot_requirements=robot.txt
      robot_person_requirements=robot-person-agpl.txt
      ;;
    ubuntu/22.04/aarch64)
      robot_ros_distro=humble
      robot_requirements=robot-x1.txt
      robot_person_requirements=robot-person-x1-agpl.txt
      ;;
    *) die "unsupported Robot platform: $os_id $os_version $architecture" ;;
  esac
  robot_ros_setup="/opt/ros/$robot_ros_distro/setup.bash"
}

detect_robot_platform() {
  # shellcheck disable=SC1091
  source /etc/os-release
  select_robot_platform "${ID:-}" "${VERSION_ID:-}" "$(uname -m)"
  if [[ -n ${ROS_DISTRO:-} && $ROS_DISTRO != "$robot_ros_distro" ]]; then
    die "this platform requires $robot_ros_distro; open a shell without ROS $ROS_DISTRO sourced"
  fi
}
