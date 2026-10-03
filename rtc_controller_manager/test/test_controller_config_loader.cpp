// Tier 1 — LoadControllerConfig: one controller YAML plus its `include:`
// fragments, composed into the node the CM hands a controller.
//
// What is pinned here:
//   - a file without `include:` loads exactly as YAML::LoadFile did;
//   - the composed tree of config/test_include/ equals expected_leaves.txt —
//     the same file rtc_tools' Python twin is checked against, which is what
//     keeps the two loaders on one tree;
//   - every way a composition can be wrong throws ControllerConfigIncludeError
//     and says which file and which key. The type matters as much as the
//     throw: the CM treats YAML::BadFile as "no config for this controller",
//     so a missing fragment surfacing as BadFile would silently drop (or
//     default) a controller the operator did configure. The bring-up side of
//     that is test_cm_config_pipeline / test_cm_required_config.

#include "rtc_controller_manager/controller_config_loader.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace rtc {
namespace {

namespace fs = std::filesystem;

constexpr const char* kKey = "ctrl";

// The SOURCE tree's config/, not the installed copy: rtc_tools' Python test
// reads expected_leaves.txt from the source tree too, so both loaders are held
// to one file even when the install is stale.
fs::path FixtureDir(const std::string& variant) {
  return fs::path(RTC_CM_SOURCE_CONFIG_DIR) / variant;
}

std::vector<std::string> ReadLines(const fs::path& path) {
  std::ifstream in(path);
  EXPECT_TRUE(in.is_open()) << path;
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  return lines;
}

// A scratch config directory per test: main.yaml plus whatever fragments the
// case writes.
class ControllerConfigLoaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // mkdtemp: a fixed path would be shared by two runs of this binary.
    std::string pattern = (fs::temp_directory_path() / "rtc_cm_config_loader_XXXXXX").string();
    ASSERT_NE(nullptr, ::mkdtemp(pattern.data())) << pattern;
    dir_ = pattern;
  }

  void TearDown() override { fs::remove_all(dir_); }

  void Write(const std::string& relative, const std::string& text) const {
    const fs::path path = dir_ / relative;
    fs::create_directories(path.parent_path());
    std::ofstream(path) << text;
  }

  [[nodiscard]] std::string Main() const { return (dir_ / "main.yaml").string(); }

  // The load must fail as an include error whose message carries every needle.
  // A bare EXPECT_THROW would also pass on the wrong failure — a typo in the
  // fixture text failing for its own reason.
  void ExpectIncludeError(const std::vector<std::string>& needles) const {
    try {
      const YAML::Node node = LoadControllerConfig(Main(), kKey);
      ADD_FAILURE() << "composed without error:\n" << YAML::Dump(node);
    } catch (const ControllerConfigIncludeError& e) {
      const std::string what = e.what();
      for (const auto& needle : needles) {
        EXPECT_NE(std::string::npos, what.find(needle))
            << "message lacks '" << needle << "': " << what;
      }
    } catch (const YAML::BadFile& e) {
      ADD_FAILURE() << "surfaced as YAML::BadFile — the CM would read this as 'no config': "
                    << e.what();
    }
  }

  fs::path dir_;
};

// ── Files without `include:` ─────────────────────────────────────────────────

TEST_F(ControllerConfigLoaderTest, AFileWithoutIncludeLoadsAsBefore) {
  const fs::path path = FixtureDir("test_fixtures") / "controllers" / "rtc_cm_cfg_test.yaml";
  const YAML::Node composed = LoadControllerConfig(path.string(), "rtc_cm_cfg_test");
  const YAML::Node direct = YAML::LoadFile(path.string())["rtc_cm_cfg_test"];

  const auto lines = ControllerConfigLeafLines(composed);
  EXPECT_FALSE(lines.empty());
  EXPECT_EQ(ControllerConfigLeafLines(direct), lines);
  EXPECT_EQ(YAML::Dump(direct), YAML::Dump(composed));
}

TEST_F(ControllerConfigLoaderTest, AMisspelledKeyStillYieldsAnUndefinedNode) {
  // The CM tells "file absent" from "key misspelled" by this: the second must
  // reach the controller as an undefined node and be refused there.
  Write("main.yaml", "not_the_key:\n  a: 1\n");
  EXPECT_FALSE(LoadControllerConfig(Main(), kKey).IsDefined());
}

TEST_F(ControllerConfigLoaderTest, AMissingMainFileIsBadFile) {
  EXPECT_THROW((void)LoadControllerConfig(Main(), kKey), YAML::BadFile);
}

// ── Composition ──────────────────────────────────────────────────────────────

TEST_F(ControllerConfigLoaderTest, TheComposedTreeMatchesTheExpectedLeaves) {
  const fs::path variant = FixtureDir("test_include");
  const YAML::Node composed = LoadControllerConfig(
      (variant / "controllers" / "rtc_cm_cfg_test.yaml").string(), "rtc_cm_cfg_test");

  const auto expected = ReadLines(variant / "expected_leaves.txt");
  ASSERT_FALSE(expected.empty());
  // Order is part of the contract: main file first, then each fragment's new
  // keys in include order. Readers take "the first topic group" to be the arm.
  EXPECT_EQ(expected, ControllerConfigLeafLines(composed));
}

TEST_F(ControllerConfigLoaderTest, ALeafMapSplitAcrossFilesMergesAtLeafLevel) {
  Write("main.yaml", "include: [p/a.yaml, sub/b.yaml]\nctrl:\n  g:\n    x: 1\n");
  Write("p/a.yaml", "ctrl:\n  g:\n    y: 2\n");
  Write("sub/b.yaml", "ctrl:\n  g:\n    z: 3\n  h: 4\n");

  const YAML::Node node = LoadControllerConfig(Main(), kKey);
  EXPECT_EQ((std::vector<std::string>{"g.x\t1", "g.y\t2", "g.z\t3", "h\t4"}),
            ControllerConfigLeafLines(node));
}

TEST_F(ControllerConfigLoaderTest, AnEmptyIncludeListComposesTheMainFileAlone) {
  Write("main.yaml", "include: []\nctrl:\n  x: 1\n");
  EXPECT_EQ((std::vector<std::string>{"x\t1"}),
            ControllerConfigLeafLines(LoadControllerConfig(Main(), kKey)));
}

// ── Every broken composition is an include error naming the files ────────────

TEST_F(ControllerConfigLoaderTest, AMissingFragmentIsAnIncludeErrorNotBadFile) {
  Write("main.yaml", "include: [p/gone.yaml]\nctrl:\n  x: 1\n");
  ExpectIncludeError({"main.yaml", "p/gone.yaml", "cannot be opened"});
}

TEST_F(ControllerConfigLoaderTest, ALeafSetByTwoFilesIsRejected) {
  Write("main.yaml", "include: [p/a.yaml, p/b.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "ctrl:\n  g:\n    y: 2\n");
  Write("p/b.yaml", "ctrl:\n  g:\n    y: 2\n");
  // Same value on both sides — still an error: which file owns the key is the
  // thing the split exists to make unambiguous.
  ExpectIncludeError({"'g.y'", "p/a.yaml", "p/b.yaml", "set in both"});
}

TEST_F(ControllerConfigLoaderTest, ALeafSetByMainAndFragmentIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "ctrl:\n  x: 1\n");
  ExpectIncludeError({"'x'", "main.yaml", "p/a.yaml", "set in both"});
}

TEST_F(ControllerConfigLoaderTest, ASequenceIsOneLeafAndIsNotConcatenated) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  logs: [one]\n");
  Write("p/a.yaml", "ctrl:\n  logs: [two]\n");
  ExpectIncludeError({"'logs'", "main.yaml", "p/a.yaml", "set in both"});
}

TEST_F(ControllerConfigLoaderTest, AMapInOneFileAndALeafInAnotherIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  g:\n    y: 2\n");
  Write("p/a.yaml", "ctrl:\n  g: 5\n");
  ExpectIncludeError(
      {"'g'", "main.yaml", "p/a.yaml", "a map in one file and a value in the other"});
}

TEST_F(ControllerConfigLoaderTest, ALeafInOneFileAndAMapInAnotherIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  g: ~\n");
  Write("p/a.yaml", "ctrl:\n  g:\n    y: 2\n");
  ExpectIncludeError(
      {"'g'", "main.yaml", "p/a.yaml", "a map in one file and a value in the other"});
}

TEST_F(ControllerConfigLoaderTest, AFragmentWithoutTheConfigKeyIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "other_ctrl:\n  y: 2\n");
  ExpectIncludeError({"p/a.yaml", "'other_ctrl'", "only top-level key is 'ctrl'"});
}

TEST_F(ControllerConfigLoaderTest, AFragmentThatIsNotAMapIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "- just\n- a list\n");
  ExpectIncludeError({"p/a.yaml", "must be a map"});
}

TEST_F(ControllerConfigLoaderTest, AFragmentWhoseConfigKeyIsNotAMapIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "ctrl: 3\n");
  ExpectIncludeError({"p/a.yaml", "no map under 'ctrl'"});
}

TEST_F(ControllerConfigLoaderTest, ANestedIncludeIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "include: [p/b.yaml]\nctrl:\n  y: 2\n");
  Write("p/b.yaml", "ctrl:\n  z: 3\n");
  ExpectIncludeError({"p/a.yaml", "do not nest"});
}

TEST_F(ControllerConfigLoaderTest, AnAbsoluteIncludePathIsRejected) {
  // The target exists and is a valid fragment: only the path form is wrong.
  Write("abs.yaml", "ctrl:\n  y: 2\n");
  Write("main.yaml", "include: ['" + (dir_ / "abs.yaml").string() + "']\nctrl:\n  x: 1\n");
  ExpectIncludeError({"main.yaml", "abs.yaml", "not absolute"});
}

TEST_F(ControllerConfigLoaderTest, AParentDirectoryIncludePathIsRejected) {
  Write("up.yaml", "ctrl:\n  y: 2\n");
  Write("cfg/main.yaml", "include: ['../up.yaml']\nctrl:\n  x: 1\n");
  try {
    (void)LoadControllerConfig((dir_ / "cfg" / "main.yaml").string(), kKey);
    ADD_FAILURE() << "a '..' include path composed";
  } catch (const ControllerConfigIncludeError& e) {
    EXPECT_NE(std::string::npos, std::string(e.what()).find("cannot contain '..'")) << e.what();
  }
}

TEST_F(ControllerConfigLoaderTest, AnIncludeThatIsNotAListIsRejected) {
  Write("main.yaml", "include: p/a.yaml\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "ctrl:\n  y: 2\n");
  ExpectIncludeError({"main.yaml", "must be a list"});
}

TEST_F(ControllerConfigLoaderTest, AnIncludeEntryThatIsNotAStringIsRejected) {
  Write("main.yaml", "include: [{path: p/a.yaml}]\nctrl:\n  x: 1\n");
  ExpectIncludeError({"main.yaml", "must be a path string"});
}

TEST_F(ControllerConfigLoaderTest, AMainFileWithAnotherTopLevelKeyIsRejected) {
  Write("main.yaml", "include: []\nctrl:\n  x: 1\nstray:\n  y: 2\n");
  ExpectIncludeError({"main.yaml", "'stray'"});
}

TEST_F(ControllerConfigLoaderTest, AnIncludingMainFileWithoutItsConfigKeyIsRejected) {
  // Without `include:` a misspelled key is an undefined node (above). With one
  // it cannot be: the fragments would compose into a tree missing every key
  // the main file meant to set.
  Write("main.yaml", "include: [p/a.yaml]\n");
  Write("p/a.yaml", "ctrl:\n  y: 2\n");
  ExpectIncludeError({"main.yaml", "no map under 'ctrl'"});
}

TEST_F(ControllerConfigLoaderTest, AFragmentThatDoesNotParseNamesTheFragment) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "ctrl:\n  y: [1, 2\n");
  ExpectIncludeError({"p/a.yaml", "main.yaml", "does not parse"});
}

TEST_F(ControllerConfigLoaderTest, AFragmentBesideTheMainFileIsRejected) {
  // Readers list a controllers/ directory's *.yaml as its controllers; a
  // fragment there would be counted as one. The file exists and is valid.
  Write("main.yaml", "include: [beside.yaml]\nctrl:\n  x: 1\n");
  Write("beside.yaml", "ctrl:\n  y: 2\n");
  ExpectIncludeError({"main.yaml", "beside.yaml", "subdirectory"});
}

TEST_F(ControllerConfigLoaderTest, AFragmentPathThatIsADirectoryIsAnIncludeError) {
  // A directory opens as a stream and fails on the first read with an error
  // that names no file — it must not reach the caller as that.
  Write("main.yaml", "include: [p/dir.yaml]\nctrl:\n  x: 1\n");
  fs::create_directories(dir_ / "p" / "dir.yaml");
  ExpectIncludeError({"main.yaml", "dir.yaml", "cannot be opened"});
}

TEST_F(ControllerConfigLoaderTest, AKeyWrittenTwiceInAFragmentIsRejected) {
  // yaml-cpp would read the first, PyYAML the last.
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "ctrl:\n  g:\n    y: 1\n    y: 2\n");
  ExpectIncludeError({"'ctrl.g.y'", "p/a.yaml", "appears twice"});
}

TEST_F(ControllerConfigLoaderTest, AKeyWrittenTwiceInAnIncludingMainFileIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n  x: 2\n");
  Write("p/a.yaml", "ctrl:\n  y: 2\n");
  ExpectIncludeError({"'ctrl.x'", "main.yaml", "appears twice"});
}

TEST_F(ControllerConfigLoaderTest, ASecondIncludeListIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\ninclude: [p/b.yaml]\n");
  Write("p/a.yaml", "ctrl:\n  y: 2\n");
  Write("p/b.yaml", "ctrl:\n  z: 3\n");
  ExpectIncludeError({"'include'", "main.yaml", "appears twice"});
}

TEST_F(ControllerConfigLoaderTest, AFragmentWithASecondDocumentIsRejected) {
  // LoadFile would return the first document and drop the second.
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n");
  Write("p/a.yaml", "ctrl:\n  y: 2\n---\nctrl:\n  z: 3\n");
  ExpectIncludeError({"p/a.yaml", "2 YAML documents"});
}

TEST_F(ControllerConfigLoaderTest, AnIncludingMainFileWithASecondDocumentIsRejected) {
  Write("main.yaml", "include: [p/a.yaml]\nctrl:\n  x: 1\n---\nctrl:\n  z: 3\n");
  Write("p/a.yaml", "ctrl:\n  y: 2\n");
  ExpectIncludeError({"main.yaml", "2 YAML documents"});
}

// ── Leaf lines ───────────────────────────────────────────────────────────────

TEST_F(ControllerConfigLoaderTest, LeafLinesKeepScalarTextAndRenderNullsAlike) {
  Write("main.yaml",
        "ctrl:\n  a: 1.0e-4\n  b: \"quoted\"\n  c: ~\n  d:\n  e: \"\"\n  f: {}\n  g: []\n");
  EXPECT_EQ((std::vector<std::string>{"a\t1.0e-4", "b\tquoted", "c\tnull", "d\tnull", "e\tnull",
                                      "f\t{}", "g\t[]"}),
            ControllerConfigLeafLines(LoadControllerConfig(Main(), kKey)));
}

}  // namespace
}  // namespace rtc
