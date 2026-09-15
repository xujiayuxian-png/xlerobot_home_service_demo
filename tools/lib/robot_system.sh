#!/usr/bin/env bash
# Platform and repository variables are supplied by tools/setup.
# shellcheck disable=SC2154

install_robot_system() {
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl gnupg build-essential cmake \
    python3-pip python3-venv python3-yaml rsync libportaudio2
  if [[ $robot_ros_distro == humble && ! -f $robot_ros_setup ]]; then
    local key="$repo_root/.xlerobot/environment/ros-archive-keyring.gpg"
    mkdir -p "$(dirname "$key")"
    curl --fail --location --retry 3 \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$key"
    gpg --show-keys --with-colons "$key" | \
      grep -q 'fpr:::::::::C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654:' || \
      die 'unexpected ROS repository signing key'
    sudo install -m 644 "$key" /usr/share/keyrings/xlerobot-ros-keyring.gpg
    printf '%s\n' 'deb [arch=arm64 signed-by=/usr/share/keyrings/xlerobot-ros-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main' | \
      sudo tee /etc/apt/sources.list.d/xlerobot-ros2.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-ros-base
  fi
  [[ -f $robot_ros_setup ]] || die "install ROS at $robot_ros_setup first"
  sudo apt-get install -y python3-rosdep python3-colcon-common-extensions \
    python3-pytest python3-numpy python3-opencv python3-scipy shellcheck
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update --rosdistro "$robot_ros_distro"
  source_environment "$robot_ros_setup"
  rosdep install --from-paths "$repo_root/ros2_ws/src" --ignore-src \
    --rosdistro "$robot_ros_distro" -y
}

ensure_robot_node() {
  # Jammy's Node 12 cannot build Vite 6. Keep Node local to this checkout.
  local node_version=22.22.0 node_arch=x64
  [[ $(uname -m) != aarch64 ]] || node_arch=arm64
  local name="node-v${node_version}-linux-${node_arch}"
  local root="$repo_root/.xlerobot/vendor"
  if [[ ! -x $root/$name/bin/node ]]; then
    mkdir -p "$root"
    (
      cd "$root" || exit
      curl --fail --location --retry 3 \
        "https://nodejs.org/dist/v${node_version}/$name.tar.xz" -o "$name.tar.xz"
      curl --fail --location --retry 3 \
        "https://nodejs.org/dist/v${node_version}/SHASUMS256.txt" -o "$name.sha256"
      awk -v archive="$name.tar.xz" '$2 == archive' "$name.sha256" > "$name.checked.sha256"
      [[ -s $name.checked.sha256 ]] || die 'Node archive missing from checksum manifest'
      sha256sum --check "$name.checked.sha256"
      tar -xJf "$name.tar.xz"
    )
  fi
  export PATH="$root/$name/bin:$PATH"
}
