#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DIR="${REPO_DIR}/ros"
ROS_SETUP="${HOME}/ros2_humble/install/setup.bash"
REMOTE_HOST_IP="${1:-${REMOTE_HOST_IP:-192.168.0.15}}"

if [[ -f "${HOME}/.bashrc" ]]; then
  # Load user shell config so pyenv is available in non-interactive shells.
  # shellcheck disable=SC1090
  source "${HOME}/.bashrc"
fi

if command -v pyenv >/dev/null 2>&1; then
  eval "$(pyenv init -)"
  eval "$(pyenv virtualenv-init -)"
else
  echo "Erreur: pyenv est introuvable dans le PATH." >&2
  exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Erreur: fichier ROS introuvable: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -d "${ROS_DIR}" ]]; then
  echo "Erreur: workspace ROS introuvable: ${ROS_DIR}" >&2
  exit 1
fi

export REMOTE_HOST_IP
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=99
unset ROS_LOCALHOST_ONLY
export ROS_DISCOVERY_SERVER="${REMOTE_HOST_IP}:11811"

pyenv activate ros-humble

set +u
source "${ROS_SETUP}"
set -u

cd "${ROS_DIR}"

set +u
source install/setup.bash
set -u

ros2 daemon stop || true
ros2 daemon start
sleep 2
ros2 daemon status
