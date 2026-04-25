// ── ros2_resource_provider.cpp
// ──────────────────────────────────────────────── Custom MuJoCo resource
// provider for "package://" ROS 2 URIs.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/ros2_resource_provider.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace rtc {
namespace {

// ── URI Helpers ──────────────────────────────────────────────────────────────

/// Parses "package://<pkg>/<path>" into <pkg> and <path>.
bool ParsePackageUri(const std::string &uri, std::string &pkg_name,
                     std::string &rel_path) {
  const std::string prefix = "package://";
  if (uri.find(prefix) != 0) {
    return false;
  }

  std::size_t start = prefix.length();
  std::size_t slash = uri.find('/', start);
  if (slash == std::string::npos) {
    pkg_name = uri.substr(start);
    rel_path = "";
  } else {
    pkg_name = uri.substr(start, slash - start);
    rel_path = uri.substr(slash + 1);
  }
  return true;
}

/// Resolves a "package://" URI to an absolute filesystem path via ament.
std::string ResolvePackageUri(const std::string &uri) {
  std::string pkg_name, rel_path;
  if (!ParsePackageUri(uri, pkg_name, rel_path)) {
    return "";
  }

  try {
    const std::string share_dir =
        ament_index_cpp::get_package_share_directory(pkg_name);
    if (!rel_path.empty()) {
      return share_dir + "/" + rel_path;
    }
    return share_dir;
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    return "";
  }
}

// ── mjpResourceProvider Callbacks ────────────────────────────────────────────

int Ros2OpenResource(mjResource *resource) {
  if (!resource || !resource->name) {
    return 0;
  }

  std::string resolved_path = ResolvePackageUri(resource->name);
  if (resolved_path.empty()) {
    std::cerr << "[MuJoCoSimulator] VFS failed to resolve URI: "
              << resource->name << "\n";
    return 0; // Failed
  }

  std::ifstream file(resolved_path, std::ios::binary | std::ios::ate);
  if (!file) {
    std::cerr << "[MuJoCoSimulator] VFS failed to open file: " << resolved_path
              << "\n";
    return 0; // Failed
  }

  const std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  auto *buffer = new std::vector<char>(static_cast<std::size_t>(size));
  if (file.read(buffer->data(), size)) {
    resource->data = static_cast<void *>(buffer);
    return 1; // Success
  }

  delete buffer;
  return 0;
}

int Ros2ReadResource(mjResource *resource, const void **buffer) {
  if (!resource || !resource->data || !buffer) {
    return -1;
  }

  const auto *buf_vec = static_cast<const std::vector<char> *>(resource->data);
  *buffer = buf_vec->data();
  return static_cast<int>(buf_vec->size());
}

void Ros2CloseResource(mjResource *resource) {
  if (!resource || !resource->data) {
    return;
  }

  auto *buf_vec = static_cast<std::vector<char> *>(resource->data);
  delete buf_vec;
  resource->data = nullptr;
}

void Ros2GetResourceDir(mjResource *resource, const char **dir, int *ndir) {
  if (!resource || !resource->name || !dir || !ndir) {
    return;
  }

  // Use a thread-local static string to hold the directory path
  // so the pointer returned remains valid for the caller.
  static thread_local std::string dir_str;

  std::string name(resource->name);
  std::size_t last_slash = name.find_last_of('/');
  if (last_slash != std::string::npos) {
    dir_str = name.substr(0, last_slash + 1); // Keep the trailing slash
  } else {
    dir_str = "";
  }

  *dir = dir_str.c_str();
  *ndir = static_cast<int>(dir_str.length());
}

} // namespace

// ── Public API ───────────────────────────────────────────────────────────────

void RegisterRos2ResourceProvider() {
  mjpResourceProvider provider;
  mjp_defaultResourceProvider(&provider);

  provider.prefix = "package";
  provider.open = Ros2OpenResource;
  provider.read = Ros2ReadResource;
  provider.close = Ros2CloseResource;
  provider.getdir = Ros2GetResourceDir;

  mjp_registerResourceProvider(&provider);
}

std::string ResolveModelPath(const std::string &path) {
  const std::string prefix = "package://";
  if (path.find(prefix) != 0) {
    return path;
  }
  return ResolvePackageUri(path);
}

} // namespace rtc
