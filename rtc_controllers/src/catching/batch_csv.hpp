// ── CSV plumbing shared by the offline catching batch tools ─────────────────
// Private to this package (src/, not include/): `catch_pose_ik_batch` and
// `catch_gate_batch` read and write the same dialect, and the rules below each
// close a way a well-formed-looking file was once misread.
#pragma once

#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace rtc::catching::batch_csv {

// %.17g round-trips every finite double exactly, which is the point: the map's
// numbers must be the solver's numbers. Shortest-round-trip would be prettier
// but std::to_chars for doubles is not uniformly available in this toolchain.
[[nodiscard]] inline std::string Num(double v) {
  std::array<char, 32> buf{};
  const int n = std::snprintf(buf.data(), buf.size(), "%.17g", v);
  if (n <= 0) {
    return "nan";
  }
  return std::string(buf.data(), static_cast<std::size_t>(n));
}

/// Split on ',' KEEPING every field, including empty ones at the end.
///
/// `std::getline(ss, cell, ',')` silently drops a trailing empty field, so a
/// row whose last columns are blank — which is exactly what a poseless result
/// writes — parses one column short and the width check then rejects a row
/// that is in fact well formed.
[[nodiscard]] inline std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    const std::string cell =
        line.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    const auto b = cell.find_first_not_of(" \t\r\n");
    const auto e = cell.find_last_not_of(" \t\r\n");
    out.push_back(b == std::string::npos ? std::string() : cell.substr(b, e - b + 1));
    if (comma == std::string::npos) {
      return out;
    }
    start = comma + 1;
  }
}

/// True for a line that carries no data (blank, or a `#` comment).
[[nodiscard]] inline bool IsSkippable(const std::string& line) {
  const auto b = line.find_first_not_of(" \t\r\n");
  return b == std::string::npos || line[b] == '#';
}

[[nodiscard]] inline double ParseFinite(const std::string& cell, std::string_view what,
                                        int line_no) {
  double v = 0.0;
  try {
    std::size_t used = 0;
    v = std::stod(cell, &used);
    if (used != cell.size()) {
      throw std::invalid_argument("trailing characters");
    }
  } catch (const std::exception&) {
    throw std::invalid_argument("line " + std::to_string(line_no) + ": '" + std::string(what) +
                                "' is not a number: '" + cell + "'");
  }
  if (!std::isfinite(v)) {
    // A non-finite candidate would be judged kTargetNonFinite and land in the
    // map as a legitimate rejection, hiding a broken generator as physics.
    throw std::invalid_argument("line " + std::to_string(line_no) + ": '" + std::string(what) +
                                "' is not finite: '" + cell + "'");
  }
  return v;
}

/// Whole-cell integer of the DESTINATION's width: `BatchCandidate::id` is
/// 64-bit, and reading it through an `int` would refuse every id ≥ 2³¹ that the
/// field can in fact hold.
template <typename Int>
[[nodiscard]] Int ParseIntCell(const std::string& cell, std::string_view what, int line_no) {
  Int v = 0;
  const char* first = cell.data();
  const char* last = cell.data() + cell.size();
  const auto res = std::from_chars(first, last, v);
  if (res.ec != std::errc() || res.ptr != last) {
    throw std::invalid_argument("line " + std::to_string(line_no) + ": '" + std::string(what) +
                                "' is not an integer: '" + cell + "'");
  }
  return v;
}

}  // namespace rtc::catching::batch_csv
