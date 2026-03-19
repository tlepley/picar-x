#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DIR="${REPO_DIR}/ros"
ROS_SETUP="${HOME}/ros2_humble/install/setup.bash"

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

pyenv activate ros-humble

set +u
source "${ROS_SETUP}"
set -u

cd "${ROS_DIR}"
PYTHONNOUSERSITE=1 colcon build \
  --packages-select picarx_local_ros2 picarx_remote_ros2

set +u
source install/setup.bash
set -u

echo "Compilation terminee."
