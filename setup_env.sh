#!/usr/bin/env bash
# Onero A1 工作区：安装依赖、构建并加载环境。用法：source setup_env.sh
# 系统基础：Ubuntu 24.04，已安装 ROS 2 Jazzy。

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source setup_env.sh，将环境加载到当前终端。" >&2
  exit 1
fi

_onero_runtime_status() {
  "$1" - "$2" "$3" <<'PY'
from importlib.metadata import PackageNotFoundError, distribution, version
from pathlib import Path
import sys

from packaging.requirements import Requirement
import onero_sdk
import pinocchio
import rclpy

for index, filename in enumerate(sys.argv[1:]):
    for line in Path(filename).read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        requirement = Requirement(line)
        try:
            installed_version = version(requirement.name)
        except PackageNotFoundError:
            if index == 0:
                raise
            continue  # Constraints apply only to installed dependencies.
        if installed_version not in requirement.specifier:
            raise RuntimeError(f'{requirement.name} {installed_version} 不满足 {requirement.specifier}')

installed = distribution('onero-sdk')
if Path(onero_sdk.__file__).resolve() != Path(installed.locate_file('onero_sdk/__init__.py')).resolve():
    raise RuntimeError('onero-sdk 被其他路径覆盖，请在未注入源码 PYTHONPATH 的终端中运行。')
print(f'{installed.version}；Pinocchio {pinocchio.__version__}')
PY
}

_onero_workspace_fingerprint() {
  /usr/bin/python3 - "$1" <<'PY'
import hashlib
import os
from pathlib import Path
import sys

workspace = Path(sys.argv[1])
digest = hashlib.sha256()
digest.update(str(workspace).encode() + b'\0colcon build --symlink-install --base-paths src\0')
ignored = {'__pycache__', '.pytest_cache', '.git', '.eggs'}
paths = []
for directory, names, files in os.walk(workspace / 'src'):
    names[:] = sorted(name for name in names
                      if name not in ignored and not name.endswith('.egg-info'))
    paths.extend(Path(directory) / name for name in files
                 if not name.endswith(('.pyc', '.pyo')))
paths.extend(workspace / name for name in ('colcon.meta', 'colcon.defaults.yaml')
             if (workspace / name).is_file())
for path in sorted(paths):
    digest.update(str(path.relative_to(workspace)).encode() + b'\0')
    if path.is_symlink():
        digest.update(os.readlink(path).encode() + b'\0')
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    digest.update(b'\0')
print(digest.hexdigest())
PY
}

_onero_load_environment() {
  local onero_workspace="$1" onero_load_workspace="${2:-true}" onero_overlay="未使用"
  local onero_sdk_status
  # Restore PATH before reloading ROS, then activate the project environment
  # after loading the workspace so repeated sourcing preserves the new paths.
  if [[ "${onero_load_workspace}" == true && -n "${VIRTUAL_ENV:-}" ]] \
      && declare -F deactivate >/dev/null; then
    deactivate
  fi
  source /opt/ros/jazzy/setup.bash || return 1
  export WS_DIR="${onero_workspace}"
  export ROS_HOME="${onero_workspace}/.ros"
  export ROS_LOG_DIR="${ROS_HOME}/log"
  mkdir -p "${ROS_LOG_DIR}" || return 1
  export ONERO_RUNTIME_PYTHON="${ONERO_RUNTIME_PYTHON:-${onero_workspace}/.venv/bin/python}"

  if [[ -d "${onero_workspace}/overlay/share" ]]; then
    export AMENT_PREFIX_PATH="${onero_workspace}/overlay:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${onero_workspace}/overlay/lib:${LD_LIBRARY_PATH:-}"
    export CMAKE_PREFIX_PATH="${onero_workspace}/overlay:${CMAKE_PREFIX_PATH:-}"
    onero_overlay="已启用"
  fi
  if [[ "${onero_load_workspace}" != true ]]; then
    return 0
  fi

  source "${onero_workspace}/install/setup.bash" || return 1
  source "${onero_workspace}/.venv/bin/activate" || return 1
  if ! ros2 pkg prefix joint_state_topic_hardware_interface >/dev/null 2>&1; then
    echo "话题硬件接口不可用，请检查 ROS 2 软件包或 overlay 配置。" >&2
    return 1
  fi
  onero_sdk_status="$(_onero_runtime_status "${ONERO_RUNTIME_PYTHON}" \
      "${onero_workspace}/tools/requirements-runtime.txt" \
      "${onero_workspace}/tools/constraints-runtime.txt")" || return 1
  echo "Onero A1 工作区已就绪"
  echo "  WS_DIR      = ${onero_workspace}"
  echo "  ROS_HOME    = ${ROS_HOME}"
  echo "  ROS_DISTRO  = ${ROS_DISTRO}"
  echo "  overlay     = ${onero_overlay}"
  echo "  Python      = ${ONERO_RUNTIME_PYTHON}"
  echo "  onero-sdk   = ${onero_sdk_status}"
}

_onero_prepare_environment() {
  local onero_workspace onero_package onero_status
  local onero_requirements onero_constraints onero_stamp onero_fingerprint onero_sdk_status
  local onero_build_stamp onero_source_fingerprint
  local -a onero_missing=() onero_sudo=()
  local -a onero_packages=(
    build-essential cmake python3-colcon-common-extensions python3-packaging python3-scipy python3-tk
    python3-dev ros-jazzy-ament-cmake-python ros-jazzy-pybind11-vendor ros-jazzy-yaml-cpp-vendor
    ros-jazzy-moveit ros-jazzy-moveit-servo ros-jazzy-moveit-planners-chomp ros-jazzy-moveit-planners-stomp
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2controlcli
  )
  onero_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" || return 1
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "请先在 Ubuntu 24.04 中安装 ROS 2 Jazzy。" >&2
    return 1
  fi

  # Install missing system dependencies; a supplied topic-interface overlay is usable.
  if [[ ! -f "${onero_workspace}/overlay/share/ament_index/resource_index/packages/joint_state_topic_hardware_interface" ]]; then
    onero_packages+=(ros-jazzy-joint-state-topic-hardware-interface)
  fi
  if ! /usr/bin/python3 -c 'import venv, ensurepip' >/dev/null 2>&1; then
    onero_missing+=(python3-venv)
  fi
  for onero_package in "${onero_packages[@]}"; do
    onero_status="$(dpkg-query -W -f='${db:Status-Status}' "${onero_package}" 2>/dev/null)"
    if [[ "${onero_status}" != installed ]]; then
      onero_missing+=("${onero_package}")
    fi
  done
  if (( ${#onero_missing[@]} )); then
    if (( EUID != 0 )); then
      onero_sudo=(sudo)
    fi
    echo "安装系统依赖：${onero_missing[*]}"
    "${onero_sudo[@]}" apt-get update || return 1
    "${onero_sudo[@]}" apt-get install -y "${onero_missing[@]}" || return 1
  fi

  _onero_load_environment "${onero_workspace}" false || return 1

  if [[ ! -x "${onero_workspace}/.venv/bin/python" ]]; then
    /usr/bin/python3 -m venv --system-site-packages "${onero_workspace}/.venv" || return 1
  fi
  onero_requirements="${onero_workspace}/tools/requirements-runtime.txt"
  onero_constraints="${onero_workspace}/tools/constraints-runtime.txt"
  onero_stamp="${onero_workspace}/.venv/.onero-runtime.sha256"
  onero_fingerprint="$(sha256sum "${onero_requirements}" "${onero_constraints}")" || return 1

  if [[ ! -f "${onero_stamp}" || "$(cat "${onero_stamp}")" != "${onero_fingerprint}" ]] \
      || ! onero_sdk_status="$(_onero_runtime_status "${onero_workspace}/.venv/bin/python" \
          "${onero_requirements}" "${onero_constraints}" 2>/dev/null)"; then
    echo "准备 .venv 并通过 pip 安装 onero-sdk 运行依赖……"
    "${onero_workspace}/.venv/bin/python" -m pip --cache-dir "${onero_workspace}/.tmp/pip-cache" \
      install --disable-pip-version-check --quiet \
      -r "${onero_requirements}" -c "${onero_constraints}" || return 1
    onero_sdk_status="$(_onero_runtime_status "${onero_workspace}/.venv/bin/python" \
        "${onero_requirements}" "${onero_constraints}")" || return 1
    printf '%s\n' "${onero_fingerprint}" > "${onero_stamp}" || return 1
  fi
  if [[ "${ONERO_RUNTIME_PYTHON}" != "${onero_workspace}/.venv/bin/python" ]]; then
    onero_sdk_status="$(_onero_runtime_status "${ONERO_RUNTIME_PYTHON}" \
        "${onero_requirements}" "${onero_constraints}")" || return 1
  fi

  onero_build_stamp="${onero_workspace}/build/.onero-workspace.sha256"
  onero_source_fingerprint="$(_onero_workspace_fingerprint "${onero_workspace}")" || return 1
  if [[ ! -f "${onero_workspace}/install/setup.bash" || ! -f "${onero_build_stamp}" \
      || "$(cat "${onero_build_stamp}")" != "${onero_source_fingerprint}" ]]; then
    echo "首次配置或源码有变化，正在构建工作区……"
    (
      cd "${onero_workspace}" || exit 1
      colcon build --symlink-install --base-paths src
    ) || return 1
    printf '%s\n' "${onero_source_fingerprint}" > "${onero_build_stamp}" || return 1
  fi
}

_onero_setup_environment() {
  local onero_workspace onero_lock_fd
  onero_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" || return 1
  mkdir -p "${onero_workspace}/.tmp" || return 1
  (
    # The descriptor belongs to this subshell; it never stays in the user's shell.
    # Keep the lock file so concurrent terminals always lock the same inode.
    exec {onero_lock_fd}> "${onero_workspace}/.tmp/setup_env.lock" || exit 1
    if ! flock -n "${onero_lock_fd}"; then
      echo "另一个终端正在配置环境，等待其完成……"
      flock "${onero_lock_fd}" || exit 1
    fi
    # Recheck dependency and build stamps only after acquiring the lock.
    _onero_prepare_environment
  ) || return 1
  _onero_load_environment "${onero_workspace}"
}

if _onero_setup_environment; then
  unset -f _onero_setup_environment _onero_prepare_environment _onero_load_environment \
    _onero_runtime_status _onero_workspace_fingerprint
else
  unset -f _onero_setup_environment _onero_prepare_environment _onero_load_environment \
    _onero_runtime_status _onero_workspace_fingerprint
  return 1
fi
