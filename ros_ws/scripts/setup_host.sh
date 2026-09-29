#!/usr/bin/env bash
# One-time host setup for the Spine coprocessor workspace (ros_ws/).
#
#   setup_host.sh --native   Ubuntu 26.04 (resolute) PC or WSL: installs ROS 2 Lyrical from
#                            packages.ros.org, build tools, rosdep, and a venv with pyntcore.
#                            Stock Lyrical binaries exist only for 26.04.
#   setup_host.sh --isaac    Jetson Orin (JetPack 7.2, Ubuntu 24.04) or an x86 NVIDIA box:
#                            installs the isaac-ros CLI and builds the Isaac ROS 5.0 container
#                            with the `spine` layer. This is the default.
#   setup_host.sh --plain    Any Linux PC with Docker and no NVIDIA GPU: builds
#                            docker/Dockerfile.spine on ros:lyrical-ros-base and prints the
#                            `docker run` line that mounts this workspace.
#
# Safe to re-run. Uses sudo for system packages (not needed as root); run it as your normal user.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"
PYNTCORE_VERSION="${PYNTCORE_VERSION:-2026.2.2}"
MODE="${1:---isaac}"

# Root (e.g. a CI container) doesn't need sudo, and may not have it.
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo"; fi
export DEBIAN_FRONTEND=noninteractive

log() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

codename() { . /etc/os-release && echo "${VERSION_CODENAME}"; }

install_rosdep_deps() {
  log "Installing package dependencies with rosdep"
  [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || ${SUDO} rosdep init
  rosdep update --rosdistro "${ROS_DISTRO}"
  set +u # ROS setup scripts reference unset variables
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  if ! find "${WS}/src" -name package.xml -print -quit | grep -q .; then
    echo "no packages in src/ yet; skipping"
    return
  fi
  rosdep install --from-paths "${WS}/src" --ignore-src -y --rosdistro "${ROS_DISTRO}"
}

setup_native() {
  local cn; cn="$(codename)"
  [ "${cn}" = "resolute" ] || die "--native needs Ubuntu 26.04 (resolute); this is '${cn}'. Use --plain (Docker) or --isaac."

  log "Adding the ROS 2 apt source"
  ${SUDO} apt-get update
  ${SUDO} apt-get install -y curl ca-certificates
  if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    local ver deb
    ver="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -oE '"tag_name": *"[^"]+"' | grep -oE '[0-9][^"]*')"
    deb="$(mktemp --suffix=.deb)"
    curl -fsSL -o "${deb}" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${cn}_all.deb"
    ${SUDO} apt-get install -y "${deb}"
    rm -f "${deb}"
  fi

  log "Installing ROS 2 ${ROS_DISTRO} (ros-base) and dev tools"
  ${SUDO} apt-get update
  ${SUDO} apt-get install -y "ros-${ROS_DISTRO}-ros-base" ros-dev-tools python3-venv python3-pip

  log "Creating ${WS}/.venv (system site packages + pyntcore ${PYNTCORE_VERSION})"
  if [ ! -x "${WS}/.venv/bin/python" ]; then
    python3 -m venv --system-site-packages "${WS}/.venv"
  fi
  touch "${WS}/.venv/COLCON_IGNORE"
  "${WS}/.venv/bin/python" -m pip install --upgrade "pyntcore==${PYNTCORE_VERSION}"

  install_rosdep_deps
}

setup_isaac() {
  local cn arch suite
  cn="$(codename)"
  arch="$(uname -m)"
  if [ "${arch}" = "aarch64" ]; then
    suite="noble-jetpack"
    if [ -f /etc/nv_tegra_release ]; then
      grep -q "R39 (release), REVISION: 2" /etc/nv_tegra_release \
        || echo "warning: expected JetPack 7.2 (L4T R39.2); found: $(head -1 /etc/nv_tegra_release)"
    fi
  else
    suite="noble"
  fi
  [ "${cn}" = "noble" ] || echo "warning: Isaac ROS 5.0 hosts are Ubuntu 24.04 (noble); this is '${cn}'."

  log "Adding the Isaac ROS 5.0 apt repo (${suite})"
  ${SUDO} apt-get update
  ${SUDO} apt-get install -y curl ca-certificates gnupg
  ${SUDO} install -d -m 0755 /etc/apt/keyrings
  curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
    | ${SUDO} gpg --dearmor --yes -o /etc/apt/keyrings/isaac-ros.gpg
  echo "deb [signed-by=/etc/apt/keyrings/isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 ${suite} main" \
    | ${SUDO} tee /etc/apt/sources.list.d/isaac-ros.list >/dev/null
  ${SUDO} apt-get update
  ${SUDO} apt-get install -y isaac-ros-cli

  log "Pointing the isaac-ros CLI at this workspace"
  export ISAAC_ROS_WS="${WS}"
  if ! grep -q "ISAAC_ROS_WS=" "${HOME}/.bashrc" 2>/dev/null; then
    echo "export ISAAC_ROS_WS=\"${WS}\"" >> "${HOME}/.bashrc"
    echo "added ISAAC_ROS_WS to ~/.bashrc"
  fi

  log "Initialising the Isaac ROS docker environment"
  ${SUDO} isaac-ros init docker
  isaac-ros activate --build-local

  cat <<EOF

Done. Enter the container with:  isaac-ros activate
Then, inside it (the workspace is mounted at /workspaces/isaac_ros-dev):
  cd /workspaces/isaac_ros-dev && scripts/test.sh
EOF
}

setup_plain() {
  command -v docker >/dev/null || die "--plain needs Docker."
  log "Building spine:plain from docker/Dockerfile.spine on ros:${ROS_DISTRO}-ros-base"
  docker build \
    --build-arg "BASE_IMAGE=ros:${ROS_DISTRO}-ros-base" \
    --build-arg "PYNTCORE_VERSION=${PYNTCORE_VERSION}" \
    -t spine:plain -f "${WS}/docker/Dockerfile.spine" "${WS}/docker"
  cat <<EOF

Done. Run the container with host networking and this workspace mounted:
  docker run --rm -it --network host -v "${WS}:/workspaces/isaac_ros-dev" \\
    -w /workspaces/isaac_ros-dev spine:plain bash
Then, inside it:  scripts/test.sh
Under WSL2, host networking only reaches the RIO with networkingMode=mirrored.
EOF
}

case "${MODE}" in
  --native) setup_native ;;
  --isaac)  setup_isaac ;;
  --plain)  setup_plain ;;
  -h|--help) sed -n '2,16p' "$0" ;;
  *) die "unknown mode '${MODE}' (use --native, --isaac or --plain)" ;;
esac
