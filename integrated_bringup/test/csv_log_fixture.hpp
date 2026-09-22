#ifndef INTEGRATED_BRINGUP_TEST_CSV_LOG_FIXTURE_HPP_
#define INTEGRATED_BRINGUP_TEST_CSV_LOG_FIXTURE_HPP_

// ── Reading a controller-owned CSV back off disk, for tests ─────────────────
//
// Shared by every suite that asserts on a `logs:` channel. Extracted from
// test_compliance_diag_log.cpp when the catching controller's per-tick record
// (S5.4) needed the same three things: a private session directory, a CSV
// split that does not lose empty trailing cells, and a lookup BY COLUMN NAME.
//
// WHY BY NAME AND NOT BY INDEX. These headers carry per-joint and per-tip
// blocks whose width comes from the robot's config, so an index-based
// assertion is a test that passes on one robot and silently reads a
// neighbouring column on another. The width check inside At() is the other
// half of that: a row and a header that disagree is exactly the failure #440
// shipped 138,248 rows through, and it must be caught wherever it is read.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

namespace integrated_bringup::testfx {

namespace fs = std::filesystem;

/// Point `RTC_SESSION_DIR` at a private directory for the life of the object,
/// and put the operator's own value back afterwards. A test that wrote into
/// the ambient session directory would leave rows behind for whoever reads
/// that session next.
class ScopedSessionDir {
 public:
  explicit ScopedSessionDir(const std::string& tag) {
    if (const char* prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_value_ = prev;
    }
    auto base = fs::temp_directory_path() / ("rtc_" + tag + "_test");
    fs::create_directories(base);
    dir_ = base / ("s_" + std::to_string(reinterpret_cast<std::uintptr_t>(this) & 0xFFFFFFFFU));
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    ::setenv("RTC_SESSION_DIR", dir_.c_str(), 1);
  }

  ~ScopedSessionDir() {
    if (had_prev_) {
      ::setenv("RTC_SESSION_DIR", prev_value_.c_str(), 1);
    } else {
      ::unsetenv("RTC_SESSION_DIR");
    }
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }

  ScopedSessionDir(const ScopedSessionDir&) = delete;
  ScopedSessionDir& operator=(const ScopedSessionDir&) = delete;
  ScopedSessionDir(ScopedSessionDir&&) = delete;
  ScopedSessionDir& operator=(ScopedSessionDir&&) = delete;

  [[nodiscard]] const fs::path& path() const noexcept { return dir_; }

 private:
  fs::path dir_;
  bool had_prev_{false};
  std::string prev_value_;
};

inline std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

struct CsvFile {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;

  [[nodiscard]] std::size_t Column(const std::string& name) const {
    for (std::size_t i = 0; i < header.size(); ++i) {
      if (header[i] == name) {
        return i;
      }
    }
    return header.size();
  }

  [[nodiscard]] bool Has(const std::string& name) const { return Column(name) < header.size(); }

  /// Same cell as At(), narrowed to float. A POD that stores these as float
  /// prints float::max_digits10 digits, so the round trip is exact — comparing
  /// at double width would fail on the digits the file never had.
  [[nodiscard]] float AtF(std::size_t row, const std::string& name) const {
    return static_cast<float>(At(row, name));
  }

  [[nodiscard]] double At(std::size_t row, const std::string& name) const {
    const std::size_t c = Column(name);
    EXPECT_LT(c, header.size()) << "missing column " << name;
    if (c >= header.size() || row >= rows.size()) {
      return 0.0;
    }
    EXPECT_EQ(rows[row].size(), header.size())
        << "row " << row << " does not match the header width";
    return std::stod(rows[row][c]);
  }

  /// The raw cell, for the columns a CSV carries as text (an FSM name, a
  /// phase). Numeric At() would turn those into 0 without saying so.
  [[nodiscard]] std::string Text(std::size_t row, const std::string& name) const {
    const std::size_t c = Column(name);
    EXPECT_LT(c, header.size()) << "missing column " << name;
    if (c >= header.size() || row >= rows.size()) {
      return {};
    }
    EXPECT_EQ(rows[row].size(), header.size())
        << "row " << row << " does not match the header width";
    return rows[row][c];
  }
};

inline CsvFile ReadCsv(const fs::path& path) {
  CsvFile out;
  std::ifstream in(path);
  std::string line;
  if (!std::getline(in, line)) {
    return out;
  }
  out.header = SplitCsv(line);
  while (std::getline(in, line)) {
    if (!line.empty()) {
      out.rows.push_back(SplitCsv(line));
    }
  }
  return out;
}

}  // namespace integrated_bringup::testfx

#endif  // INTEGRATED_BRINGUP_TEST_CSV_LOG_FIXTURE_HPP_
