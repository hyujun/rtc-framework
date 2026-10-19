#ifndef INTEGRATED_BRINGUP_TEST_LOG_CSV_ASSERTS_HPP_
#define INTEGRATED_BRINGUP_TEST_LOG_CSV_ASSERTS_HPP_

// ── Asserting a CSV writer's row against its header, column by column ───────
//
// For tests that drive a writer in memory (header string + row string), not
// ones that read a logger's file back (csv_log_fixture.hpp, whose SplitCsv
// this uses). Extracted from test_device_log_pods.cpp (#883) when the planner
// events writer needed the same assertions (#886).
//
// HeaderRowAligned proves the two writers agree on width and names, and
// ColumnValue pins one cell. Neither catches two row statements trading places
// when both cells hold the same value — the default 0, or two bools left
// false. RowMatchesByName in strict mode asserts every column by name with a
// value no other column has, and BoolsAreOneHot walks each bool column on
// alone, so any such swap reads a wrong cell. Value rules: |v| < 10000 (the
// default ostream prints 6 significant digits), steps of 0.25 (exact in
// float), integer and floating columns from disjoint ranges (float 101 and int
// 101 both print "101").
//
// A row passed here is ONE line without its terminator: a writer that ends
// its row with '\n' leaves it on the last cell.

#include "csv_log_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace integrated_bringup::testfx {

// Header ↔ row structural agreement: same token count, no empty header name,
// no duplicate header name. Comma counting alone (the pre-#551 check) passed
// a header whose columns were in a different order from the row's — every CSV
// consumer is name-based, so that reads silently wrong values.
inline ::testing::AssertionResult HeaderRowAligned(const std::string& hdr, const std::string& row) {
  const auto h = SplitCsv(hdr);
  const auto r = SplitCsv(row);
  if (h.size() != r.size())
    return ::testing::AssertionFailure() << "header has " << h.size() << " columns, row has "
                                         << r.size() << "\nheader: " << hdr << "\nrow: " << row;
  std::vector<std::string> sorted = h;
  std::sort(sorted.begin(), sorted.end());
  for (std::size_t i = 0; i < sorted.size(); ++i) {
    if (sorted[i].empty())
      return ::testing::AssertionFailure() << "empty header name\nheader: " << hdr;
    if (i > 0 && sorted[i] == sorted[i - 1])
      return ::testing::AssertionFailure()
             << "duplicate header name '" << sorted[i] << "'\nheader: " << hdr;
  }
  return ::testing::AssertionSuccess();
}

// Row value under the header column named `name` ("" when the name is absent).
inline std::string ColumnValue(const std::string& hdr, const std::string& row,
                               const std::string& name) {
  const auto h = SplitCsv(hdr);
  const auto r = SplitCsv(row);
  for (std::size_t i = 0; i < h.size() && i < r.size(); ++i)
    if (h[i] == name)
      return r[i];
  return {};
}

// Expected cells of one row, as (header name, printed value).
using Cells = std::vector<std::pair<std::string, std::string>>;

template <class T>
std::string Str(const T& v) {
  std::ostringstream os;
  os << v;
  return os.str();
}

// Every name in `expected` is a header column whose row cell prints its value,
// and every header column is in `expected`. `strict` (the writer tests, not
// the Fill round trips) also requires each non-bool cell to be neither "0" nor
// "1" and unlike every other non-bool cell, so swapping any two of them shows.
inline ::testing::AssertionResult RowMatchesByName(const std::string& hdr, const std::string& row,
                                                   const Cells& expected,
                                                   const std::vector<std::string>& bools,
                                                   bool strict) {
  const auto aligned = HeaderRowAligned(hdr, row);
  if (!aligned)
    return aligned;
  const auto h = SplitCsv(hdr);
  const auto r = SplitCsv(row);
  std::ostringstream err;
  std::map<std::string, std::string> want;
  for (const auto& [name, value] : expected)
    if (!want.emplace(name, value).second)
      err << "\n  '" << name << "' expected twice";
  for (const auto& name : bools)
    if (want.count(name) == 0)
      err << "\n  bool '" << name << "' has no expected value";
  for (const auto& [name, value] : expected) {
    const auto it = std::find(h.begin(), h.end(), name);
    if (it == h.end()) {
      err << "\n  '" << name << "' is not a header column";
      continue;
    }
    const auto& got = r[static_cast<std::size_t>(it - h.begin())];
    if (got != value)
      err << "\n  " << name << ": row has '" << got << "', expected '" << value << "'";
  }
  for (const auto& name : h)
    if (want.count(name) == 0)
      err << "\n  header column '" << name << "' is not asserted";
  if (strict) {
    std::map<std::string, std::string> owner;  // printed value → column
    for (const auto& [name, value] : expected) {
      if (std::find(bools.begin(), bools.end(), name) != bools.end())
        continue;
      if (value == "0" || value == "1")
        err << "\n  non-bool '" << name << "' expects '" << value << "' (reads as a bool)";
      const auto [it, fresh] = owner.emplace(value, name);
      if (!fresh)
        err << "\n  '" << name << "' and '" << it->second << "' both expect '" << value
            << "' (a swap between them would not show)";
    }
  }
  if (!err.str().empty())
    return ::testing::AssertionFailure() << err.str() << "\nheader: " << hdr << "\nrow: " << row;
  return ::testing::AssertionSuccess();
}

// One bool walk step: the cells `on` must read "1" after `set` is applied to
// the all-false base pod, every other bool cell "0".
template <class Pod>
struct BoolCase {
  std::vector<std::string> on;
  std::function<void(Pod&)> set;
};

// `write_row(pod)` returns the row. The base must print every bool as "0",
// each case must light exactly its `on` cells, and every bool must be lit by
// some case — two bool statements that trade places then light the wrong cell.
template <class Pod, class WriteRow>
::testing::AssertionResult BoolsAreOneHot(const std::string& hdr, const Pod& base,
                                          const WriteRow& write_row,
                                          const std::vector<std::string>& bools,
                                          const std::vector<BoolCase<Pod>>& cases) {
  std::ostringstream err;
  const auto check = [&](const std::string& label, const std::string& row,
                         const std::vector<std::string>& on) {
    const auto aligned = HeaderRowAligned(hdr, row);
    if (!aligned) {
      err << "\n  " << label << ": " << aligned.message();
      return;
    }
    for (const auto& name : bools) {
      const bool lit = std::find(on.begin(), on.end(), name) != on.end();
      const auto got = ColumnValue(hdr, row, name);
      if (got != (lit ? "1" : "0"))
        err << "\n  " << label << ": '" << name << "' reads '" << got << "', expected "
            << (lit ? "1" : "0");
    }
  };
  check("base", write_row(base), {});
  std::vector<std::string> covered;
  for (const auto& c : cases) {
    Pod pod = base;
    c.set(pod);
    std::string label = "case";
    for (const auto& name : c.on)
      label += " " + name;
    check(label, write_row(pod), c.on);
    covered.insert(covered.end(), c.on.begin(), c.on.end());
  }
  for (const auto& name : bools)
    if (std::find(covered.begin(), covered.end(), name) == covered.end())
      err << "\n  bool '" << name << "' is never lit";
  if (!err.str().empty())
    return ::testing::AssertionFailure() << err.str() << "\nheader: " << hdr;
  return ::testing::AssertionSuccess();
}

// Gives each field a value and records the cell the row must print for it.
// Real() hands out 100.25, 100.75, ... — always x.25 / x.75, so a floating cell
// never prints like an integer one; Int() takes its value from a range the
// caller keeps disjoint (and away from 0 / 1).
struct CellBuilder {
  Cells want;
  double next{100.25};

  template <class T>
  void Real(const std::string& name, T& field) {
    field = static_cast<T>(next);
    next += 0.5;
    want.emplace_back(name, Str(field));
  }

  template <class T>
  void Int(const std::string& name, T& field, long long value) {
    field = static_cast<T>(value);
    want.emplace_back(name, std::to_string(value));
  }

  void Bool(const std::string& name) { want.emplace_back(name, "0"); }
};

// A writer-test pod and the cells its row must print.
template <class Pod>
struct PodCells {
  Pod pod{};
  Cells want;
};

// One BoolCase per (column, bool member).
template <class Pod>
std::vector<BoolCase<Pod>> BoolMemberCases(
    const std::vector<std::pair<std::string, bool Pod::*>>& members) {
  std::vector<BoolCase<Pod>> cases;
  for (const auto& [name, member] : members)
    cases.push_back({{name}, [member = member](Pod& p) { p.*member = true; }});
  return cases;
}

}  // namespace integrated_bringup::testfx

#endif  // INTEGRATED_BRINGUP_TEST_LOG_CSV_ASSERTS_HPP_
