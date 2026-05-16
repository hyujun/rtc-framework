#!/bin/bash
# install_ros2.sh — ROS2 / Ubuntu prerequisites + workspace setup
#
# 제공 함수:
#   install_ros2 UBUNTU_VER   — ROS2 humble/jazzy 자동 설치 (locale + apt repo)
#   check_prerequisites       — Ubuntu / ROS2 / venv 확인, ROS_PKG_PREFIX 설정
#   setup_workspace           — ament-cmake, eigen, colcon, vcstool, ethtool
#
# Caller scope 의존:
#   ROS_PKG_PREFIX (check_prerequisites가 설정), UBUNTU_VERSION, ROS_DISTRO_DETECTED
#   apt_update_if_stale (install.sh 정의), 로거 (info/warn/success/error)

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This file should be sourced, not executed." >&2
  exit 1
fi

[[ -n "${_INSTALL_ROS2_LOADED:-}" ]] && return 0
_INSTALL_ROS2_LOADED=1

# ── ROS2 Installation (when not found) ────────────────────────────────────────
install_ros2() {
  local ubuntu_ver="$1"
  local ros_distro=""

  case "$ubuntu_ver" in
    22.04) ros_distro="humble" ;;
    24.04) ros_distro="jazzy"  ;;
    *)
      error "Cannot auto-install ROS2: unsupported Ubuntu ${ubuntu_ver}. Supported: 22.04 (Humble), 24.04 (Jazzy)."
      ;;
  esac

  info "Installing ROS2 ${ros_distro} for Ubuntu ${ubuntu_ver}..."

  # ── Locale setup ──────────────────────────────────────────────────────────
  info "Setting up locale (UTF-8)..."
  sudo apt-get update -qq
  sudo apt-get install -y locales > /dev/null
  sudo locale-gen en_US en_US.UTF-8 > /dev/null
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  # ── Add ROS2 apt repository ───────────────────────────────────────────────
  info "Adding ROS2 apt repository..."
  sudo apt-get install -y software-properties-common curl > /dev/null
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  # ── Install ROS2 ─────────────────────────────────────────────────────────
  info "Updating package index and installing ros-${ros_distro}-desktop..."
  sudo apt-get update -qq
  sudo apt-get install -y ros-${ros_distro}-desktop > /dev/null

  # ── Install rosdep ────────────────────────────────────────────────────────
  info "Installing rosdep..."
  sudo apt-get install -y python3-rosdep2 > /dev/null 2>&1 \
      || sudo apt-get install -y python3-rosdep > /dev/null 2>&1 \
      || true
  if command -v rosdep &>/dev/null; then
    sudo rosdep init 2>/dev/null || true
    rosdep update --rosdistro="${ros_distro}" 2>/dev/null || true
  fi

  # ── Source the newly installed ROS2 ──────────────────────────────────────
  # shellcheck disable=SC1090
  # NOTE: set -e 하에서 setup.bash 내부 non-zero 반환 방지
  source "/opt/ros/${ros_distro}/setup.bash" || true

  success "ROS2 ${ros_distro} installed successfully"
}

# ── Common: ROS2 + Ubuntu check ────────────────────────────────────────────────
check_prerequisites() {
  # ── Detect Ubuntu version early (needed for ROS2 auto-install) ───────────
  UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
  local SUPPORTED_UBUNTU=("22.04" "24.04")
  if [[ ! " ${SUPPORTED_UBUNTU[*]} " =~ " ${UBUNTU_VERSION} " ]]; then
    warn "Unsupported Ubuntu (${UBUNTU_VERSION}). Supported: 22.04 (Humble), 24.04 (Jazzy). Continuing..."
  fi

  # If ros2 command is not in PATH, try to find and source ROS2 from /opt/ros/
  # Distro priority matches setup_env.sh / build_deps.sh: jazzy first, humble fallback.
  if ! command -v ros2 &>/dev/null; then
    warn "ros2 command not found in PATH. Searching /opt/ros/ ..."
    local _found_ros2=0
    if [[ -d /opt/ros ]]; then
      for _distro in jazzy humble; do
        if [[ -f "/opt/ros/${_distro}/setup.bash" ]]; then
          info "Found ROS2 ${_distro} at /opt/ros/${_distro} — sourcing setup.bash ..."
          # shellcheck disable=SC1090
          # NOTE: set -e 하에서 setup.bash 내부 non-zero 반환 방지
          source "/opt/ros/${_distro}/setup.bash" || true
          _found_ros2=1
          break
        fi
      done
    fi
    if [[ "$_found_ros2" -eq 0 ]]; then
      warn "ROS2 not found on this system."
      if [[ " ${SUPPORTED_UBUNTU[*]} " =~ " ${UBUNTU_VERSION} " ]]; then
        info "Supported Ubuntu ${UBUNTU_VERSION} detected — attempting automatic ROS2 installation..."
        install_ros2 "$UBUNTU_VERSION"
      else
        error "ROS2 not found and auto-install unavailable for Ubuntu ${UBUNTU_VERSION}. Install ROS2 manually: https://docs.ros.org/en/jazzy/Installation.html"
      fi
    fi
  fi

  # Detect ROS2 distro via three-tier fallback:
  #   1. $ROS_DISTRO env var  — set automatically when setup.bash is sourced (most reliable)
  #   2. /opt/ros/<distro>    — jazzy preferred, humble fallback (matches setup_env.sh)
  #   3. ros2 --version       — parses "(humble)" from "ros2, version X.Y.Z (humble)"
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    ROS_DISTRO_DETECTED="$ROS_DISTRO"
  else
    ROS_DISTRO_DETECTED="unknown"
    for _distro in jazzy humble; do
      if [[ -f "/opt/ros/${_distro}/setup.bash" ]]; then
        ROS_DISTRO_DETECTED="$_distro"
        break
      fi
    done
    if [[ "$ROS_DISTRO_DETECTED" == "unknown" ]]; then
      ROS_DISTRO_DETECTED=$(ros2 --version 2>/dev/null | grep -oP '\(\K[^)]+' || echo "unknown")
    fi
  fi

  success "ROS2 detected: ${ROS_DISTRO_DETECTED}"

  # ── Distro-aware package prefix ─────────────────────────────────────────────
  # ros-humble-* or ros-jazzy-* selected automatically based on detected distro.
  ROS_PKG_PREFIX="ros-${ROS_DISTRO_DETECTED}"

  if [[ "$ROS_DISTRO_DETECTED" != "humble" && "$ROS_DISTRO_DETECTED" != "jazzy" ]]; then
    warn "Unsupported ROS2 distro: ${ROS_DISTRO_DETECTED}. Supported: humble, jazzy. Continuing..."
  fi

  # ── venv compatibility check ─────────────────────────────────────────────────
  if is_venv_active; then
    warn "Active virtual environment: ${VIRTUAL_ENV}"
    warn "System apt packages may not be visible. Recommended:"
    warn "  python3 -m venv .venv --system-site-packages"
    warn "Python deps will also be pip-installed into the venv as fallback."
  fi
}

# ── Common: Workspace + build tools ───────────────────────────────────────────
setup_workspace() {
  info "Installing ROS2 build tools (${ROS_PKG_PREFIX})..."
  apt_update_if_stale
  sudo apt-get install -y \
      ${ROS_PKG_PREFIX}-ament-cmake \
      ${ROS_PKG_PREFIX}-ament-cmake-gtest \
      ${ROS_PKG_PREFIX}-ament-lint-auto \
      ${ROS_PKG_PREFIX}-ament-lint-common \
      libeigen3-dev \
      python3-colcon-common-extensions \
      python3-vcstool \
      ethtool \
      > /dev/null
  success "Build tools installed"
}
