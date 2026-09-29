# Source this before building, testing or launching: ROS underlay, this workspace's overlay (if
# built), and the pyntcore venv (if present). Safe to source in an interactive shell.
# Works natively (setup_host.sh --native) and inside the Isaac / plain containers.
ROS_DISTRO="${ROS_DISTRO:-lyrical}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "error: /opt/ros/${ROS_DISTRO} not found. Run scripts/setup_host.sh first." >&2
  return 1 2>/dev/null || exit 1
fi

# ROS and colcon setup scripts reference unset variables: relax nounset around them and restore
# the caller's setting afterwards.
case "$-" in *u*) _spine_nounset=1 ;; *) _spine_nounset=0 ;; esac
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
fi
if [ -x "${WS}/.venv/bin/python" ]; then
  if [ -z "${VIRTUAL_ENV:-}" ]; then
    # shellcheck disable=SC1091
    source "${WS}/.venv/bin/activate"
  fi
  # ROS installs Python nodes with a #!/usr/bin/python3 shebang, which ignores the venv. Putting
  # the venv's site-packages on PYTHONPATH lets those nodes import pyntcore anyway (the venv uses
  # the same interpreter version).
  _spine_site="$("${WS}/.venv/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  case ":${PYTHONPATH:-}:" in
    *":${_spine_site}:"*) ;;
    *) export PYTHONPATH="${_spine_site}${PYTHONPATH:+:${PYTHONPATH}}" ;;
  esac
  unset _spine_site
fi
[ "${_spine_nounset}" = 1 ] && set -u
unset _spine_nounset

# WSL2 (mirrored networking) doesn't deliver the UDP multicast DDS uses for discovery, so nodes in
# different processes never see each other. Fast DDS's shared-memory transport works, and is all
# a single-machine fake coprocessor needs. The Spine link itself is plain unicast UDP and is
# unaffected. Override by setting FASTDDS_BUILTIN_TRANSPORTS yourself.
if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null && [ -z "${FASTDDS_BUILTIN_TRANSPORTS:-}" ]; then
  export FASTDDS_BUILTIN_TRANSPORTS=SHM
fi

# Where spine_stats writes its rotating JSONL logs, unless its log_dir parameter says otherwise.
export SPINE_LOG_DIR="${SPINE_LOG_DIR:-${WS}/spine_logs}"
