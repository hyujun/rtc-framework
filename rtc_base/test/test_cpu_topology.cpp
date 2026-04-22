// ── test_cpu_topology.cpp ────────────────────────────────────────────────────
// rtc::DetectCpuTopology / GetCpuTopology — Stage A detection layer.
//
// Stage A does NOT change SelectThreadConfigs(); this suite only verifies
// that the new detector classifies common layouts correctly and that the
// SMT-sibling invariant holds on whatever hardware runs the test.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_base/threading/cpu_topology.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

namespace fs = std::filesystem;

namespace {

class ScopedEnv {
public:
  ScopedEnv(const char *name, const std::string &value) : name_(name) {
    if (const char *prev = std::getenv(name)) {
      prev_ = prev;
      had_prev_ = true;
    }
    ::setenv(name, value.c_str(), 1);
  }
  ~ScopedEnv() {
    if (had_prev_)
      ::setenv(name_, prev_.c_str(), 1);
    else
      ::unsetenv(name_);
  }
  ScopedEnv(const ScopedEnv &) = delete;
  ScopedEnv &operator=(const ScopedEnv &) = delete;

private:
  const char *name_;
  std::string prev_;
  bool had_prev_{false};
};

class ScopedUnsetEnv {
public:
  explicit ScopedUnsetEnv(const char *name) : name_(name) {
    if (const char *prev = std::getenv(name)) {
      prev_ = prev;
      had_prev_ = true;
    }
    ::unsetenv(name);
  }
  ~ScopedUnsetEnv() {
    if (had_prev_)
      ::setenv(name_, prev_.c_str(), 1);
  }

private:
  const char *name_;
  std::string prev_;
  bool had_prev_{false};
};

// Mock sysfs tree builder. Writes the minimum files that DetectCpuTopology
// consults: per-cpu topology/{physical_package_id,core_id},
// cpufreq/cpuinfo_max_freq, and types/{intel_core,intel_atom}/cpus.
class MockSysfs {
public:
  explicit MockSysfs(const fs::path &root) : root_(root) {
    fs::create_directories(root_ / "devices/system/cpu");
  }

  // Add a P-core physical core with optional SMT sibling.
  void AddPCore(int core_id, int primary_cpu, int sibling_cpu = -1,
                int max_freq_khz = 5000000) {
    AddCpu(primary_cpu, core_id, max_freq_khz);
    if (sibling_cpu >= 0)
      AddCpu(sibling_cpu, core_id, max_freq_khz);
    p_cpus_.push_back(primary_cpu);
    if (sibling_cpu >= 0)
      p_cpus_.push_back(sibling_cpu);
  }

  // Add an E-core (single logical, Gracemont-style). LP-E is just an E-core
  // with a noticeably lower cpuinfo_max_freq (<70% of the max E-core freq).
  void AddECore(int core_id, int cpu, int max_freq_khz = 3800000) {
    AddCpu(cpu, core_id, max_freq_khz);
    e_cpus_.push_back(cpu);
  }

  // Finalize: write types/intel_core/cpus and types/intel_atom/cpus.
  // Call once after all AddPCore / AddECore.
  void WriteTypes() {
    if (!p_cpus_.empty()) {
      fs::create_directories(root_ / "devices/system/cpu/types/intel_core");
      WriteFile(root_ / "devices/system/cpu/types/intel_core/cpus",
                JoinCpus(p_cpus_));
    }
    if (!e_cpus_.empty()) {
      fs::create_directories(root_ / "devices/system/cpu/types/intel_atom");
      WriteFile(root_ / "devices/system/cpu/types/intel_atom/cpus",
                JoinCpus(e_cpus_));
    }
  }

  // /proc/cpuinfo stub. Controls the `hybrid` fallback flag path.
  fs::path WriteCpuinfo(bool has_hybrid_flag) {
    const fs::path p =
        root_.parent_path() / (root_.filename().string() + "_cpuinfo");
    std::ofstream f(p);
    f << "processor\t: 0\n";
    f << "vendor_id\t: GenuineIntel\n";
    f << "flags\t\t: fpu vme de pse tsc msr pae mce cx8 apic sep mtrr";
    if (has_hybrid_flag)
      f << " hybrid";
    f << " pat pse36\n";
    return p;
  }

private:
  void AddCpu(int cpu, int core_id, int max_freq_khz) {
    const fs::path cpu_dir =
        root_ / "devices/system/cpu" / ("cpu" + std::to_string(cpu));
    fs::create_directories(cpu_dir / "topology");
    WriteFile(cpu_dir / "topology/physical_package_id", "0");
    WriteFile(cpu_dir / "topology/core_id", std::to_string(core_id));
    if (max_freq_khz > 0) {
      fs::create_directories(cpu_dir / "cpufreq");
      WriteFile(cpu_dir / "cpufreq/cpuinfo_max_freq",
                std::to_string(max_freq_khz));
    }
  }

  static void WriteFile(const fs::path &p, const std::string &content) {
    std::ofstream f(p);
    f << content;
  }

  static std::string JoinCpus(const std::vector<int> &cpus) {
    // Produce comma-separated list; DetectCpuTopology can parse ranges
    // too, but per-id is the simplest unambiguous form.
    std::string s;
    for (std::size_t i = 0; i < cpus.size(); ++i) {
      if (i)
        s += ",";
      s += std::to_string(cpus[i]);
    }
    return s;
  }

  fs::path root_;
  std::vector<int> p_cpus_;
  std::vector<int> e_cpus_;
};

class CpuTopologyTest : public ::testing::Test {
protected:
  void SetUp() override {
    tmp_root_ = fs::temp_directory_path() /
                ("rtc_topo_test_" + std::to_string(::getpid()) + "_" +
                 std::to_string(counter_++));
    fs::create_directories(tmp_root_);
  }
  void TearDown() override {
    std::error_code err;
    fs::remove_all(tmp_root_, err);
  }
  fs::path tmp_root_;
  static inline int counter_{0};
};

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// 1. SmtSiblingInvariant — runs against real /sys. Skips on non-hybrid or
//    non-SMT-P-core systems (almost all developer machines today).
// ─────────────────────────────────────────────────────────────────────────────
TEST(CpuTopology, SmtSiblingInvariant) {
  const auto &t = rtc::GetCpuTopology();
  if (!t.is_hybrid || !t.p_core_has_smt) {
    GTEST_SKIP() << "Not an Intel hybrid CPU with SMT — nothing to verify.";
  }
  ASSERT_EQ(t.p_core_physical_ids.size(), t.p_core_sibling_ids.size());
  for (std::size_t i = 0; i < t.p_core_physical_ids.size(); ++i) {
    const int phys = t.p_core_physical_ids[i];
    const int sib = t.p_core_sibling_ids[i];
    const fs::path phys_core_p = fs::path("/sys/devices/system/cpu") /
                                 ("cpu" + std::to_string(phys)) /
                                 "topology/core_id";
    const fs::path sib_core_p = fs::path("/sys/devices/system/cpu") /
                                ("cpu" + std::to_string(sib)) /
                                "topology/core_id";
    const int phys_core = rtc::internal::topology::ReadInt(phys_core_p);
    const int sib_core = rtc::internal::topology::ReadInt(sib_core_p);
    EXPECT_GE(phys_core, 0);
    EXPECT_EQ(phys_core, sib_core)
        << "P-core index " << i << " SMT mismatch: cpu" << phys
        << "(core_id=" << phys_core << ") vs cpu" << sib
        << "(core_id=" << sib_core << ")";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. NUC13_i7_1360P_Mock — 4P + 8E, P-core SMT on, no LP-E.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CpuTopologyTest, NUC13_i7_1360P_Mock) {
  MockSysfs m(tmp_root_);
  // 4 P-cores, each with SMT sibling. core_id 0..3, cpus 0..7.
  m.AddPCore(0, 0, 1);
  m.AddPCore(1, 2, 3);
  m.AddPCore(2, 4, 5);
  m.AddPCore(3, 6, 7);
  // 8 E-cores, no SMT, core_id 4..11, cpus 8..15, uniform freq.
  for (int i = 0; i < 8; ++i)
    m.AddECore(4 + i, 8 + i, /*freq*/ 3800000);
  m.WriteTypes();
  const auto cpuinfo = m.WriteCpuinfo(/*has_hybrid_flag=*/true);

  ScopedUnsetEnv clear{"RTC_FORCE_HYBRID_GENERATION"};
  const auto t =
      rtc::DetectCpuTopology(tmp_root_.string(), cpuinfo.string(), nullptr);

  EXPECT_TRUE(t.is_hybrid);
  EXPECT_TRUE(t.p_core_has_smt);
  EXPECT_FALSE(t.has_lp_e_cores);
  EXPECT_EQ(t.num_p_physical, 4);
  EXPECT_EQ(t.num_p_logical, 8);
  EXPECT_EQ(t.num_e_cores, 8);
  EXPECT_EQ(t.num_lpe_cores, 0);
  EXPECT_EQ(t.num_physical_cores, 12);
  EXPECT_EQ(t.num_logical_cores, 16);
  EXPECT_EQ(t.generation, rtc::NucGeneration::RAPTOR_LAKE_P);
  EXPECT_EQ(t.num_effective_cores(), 4);

  ASSERT_EQ(t.p_core_physical_ids.size(), 4u);
  ASSERT_EQ(t.p_core_sibling_ids.size(), 4u);
  // Sibling invariant: same core_id grouping for each P-core.
  EXPECT_EQ(t.p_core_physical_ids[0], 0);
  EXPECT_EQ(t.p_core_sibling_ids[0], 1);
  EXPECT_EQ(t.p_core_physical_ids[3], 6);
  EXPECT_EQ(t.p_core_sibling_ids[3], 7);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. BIOS_HT_Off_Mock — Intel hybrid with Hyper-Threading disabled in BIOS.
//    IS_HYBRID stays true (E-core side is SMT-free anyway), but
//    p_core_has_smt becomes false → generation RAPTOR_LAKE_P_HT_OFF.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CpuTopologyTest, BIOS_HT_Off_Mock) {
  MockSysfs m(tmp_root_);
  m.AddPCore(0, 0); // no sibling = HT off
  m.AddPCore(1, 1);
  m.AddPCore(2, 2);
  m.AddPCore(3, 3);
  for (int i = 0; i < 8; ++i)
    m.AddECore(4 + i, 4 + i, 3800000);
  m.WriteTypes();
  const auto cpuinfo = m.WriteCpuinfo(/*has_hybrid_flag=*/true);

  ScopedUnsetEnv clear{"RTC_FORCE_HYBRID_GENERATION"};
  const auto t =
      rtc::DetectCpuTopology(tmp_root_.string(), cpuinfo.string(), nullptr);

  EXPECT_TRUE(t.is_hybrid);
  EXPECT_FALSE(t.p_core_has_smt);
  EXPECT_FALSE(t.has_lp_e_cores);
  EXPECT_EQ(t.num_p_physical, 4);
  EXPECT_EQ(t.num_p_logical, 4);
  EXPECT_EQ(t.generation, rtc::NucGeneration::RAPTOR_LAKE_P_HT_OFF);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. ContainerFallback_NoSysfsTypes — /sys/.../types missing, no cpuinfo
//    hybrid flag. Detector must stay conservative (is_hybrid=false).
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CpuTopologyTest, ContainerFallback_NoSysfsTypes) {
  MockSysfs m(tmp_root_);
  // Only topology files, no types/intel_core or types/intel_atom directory.
  for (int cpu = 0; cpu < 8; ++cpu)
    m.AddPCore(cpu, cpu);
  // Intentionally skip m.WriteTypes().
  const auto cpuinfo = m.WriteCpuinfo(/*has_hybrid_flag=*/false);

  ScopedUnsetEnv clear{"RTC_FORCE_HYBRID_GENERATION"};
  const auto t =
      rtc::DetectCpuTopology(tmp_root_.string(), cpuinfo.string(), nullptr);

  EXPECT_FALSE(t.is_hybrid);
  EXPECT_EQ(t.generation, rtc::NucGeneration::NOT_NUC_HYBRID);
  EXPECT_EQ(t.num_physical_cores, 8);
  EXPECT_EQ(t.num_effective_cores(), 8);
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. AMD_Ryzen_Mock — no hybrid flag, no intel_core/intel_atom types.
//    Must regression-preserve num_effective_cores() == num_physical_cores.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CpuTopologyTest, AMD_Ryzen_Mock) {
  MockSysfs m(tmp_root_);
  // 8 physical cores with SMT siblings = 16 logical.
  for (int core = 0; core < 8; ++core) {
    m.AddPCore(core, core * 2, core * 2 + 1, /*freq*/ 5400000);
  }
  const auto cpuinfo = m.WriteCpuinfo(/*has_hybrid_flag=*/false);

  ScopedUnsetEnv clear{"RTC_FORCE_HYBRID_GENERATION"};
  const auto t =
      rtc::DetectCpuTopology(tmp_root_.string(), cpuinfo.string(), nullptr);

  EXPECT_FALSE(t.is_hybrid);
  EXPECT_EQ(t.generation, rtc::NucGeneration::NOT_NUC_HYBRID);
  EXPECT_EQ(t.num_physical_cores, 8);
  EXPECT_EQ(t.num_logical_cores, 16);
  EXPECT_TRUE(t.smt_active);
  EXPECT_EQ(t.num_effective_cores(), 8); // hybrid branch disabled
  // p_core lists should be empty on non-hybrid.
  EXPECT_TRUE(t.p_core_physical_ids.empty());
  EXPECT_TRUE(t.p_core_sibling_ids.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. EnvVarOverridesGenerationOnly — hint forces generation enum but leaves
//    topology id-lists as-detected. Guards against env-var-driven misuse in
//    production images.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CpuTopologyTest, EnvVarOverridesGenerationOnly) {
  // Plain AMD-looking mock: no hybrid evidence whatsoever.
  MockSysfs m(tmp_root_);
  for (int core = 0; core < 8; ++core) {
    m.AddPCore(core, core * 2, core * 2 + 1);
  }
  const auto cpuinfo = m.WriteCpuinfo(/*has_hybrid_flag=*/false);

  const auto t = rtc::DetectCpuTopology(tmp_root_.string(), cpuinfo.string(),
                                        "raptor_lake_p");

  // generation comes from the hint...
  EXPECT_EQ(t.generation, rtc::NucGeneration::RAPTOR_LAKE_P);
  // ...but is_hybrid and id lists remain what sysfs said.
  EXPECT_FALSE(t.is_hybrid);
  EXPECT_TRUE(t.p_core_physical_ids.empty());
  EXPECT_TRUE(t.p_core_sibling_ids.empty());
  // Validators in Stage B rely on this discrepancy to FAIL loudly.
}

// Bonus: classifier-only coverage for the two NUC generations we cannot
// yet test end-to-end via a pure sysfs mock (requires realistic freq
// distributions for LP-E detection). We verify the classifier primitive
// directly so ClassifyGeneration stays correct as Stage B consumes it.
TEST(CpuTopologyClassifier, ClassifyGeneration) {
  using rtc::NucGeneration;
  using rtc::internal::topology::ClassifyGeneration;
  EXPECT_EQ(ClassifyGeneration(false, false, false),
            NucGeneration::NOT_NUC_HYBRID);
  EXPECT_EQ(ClassifyGeneration(true, true, false),
            NucGeneration::RAPTOR_LAKE_P);
  EXPECT_EQ(ClassifyGeneration(true, true, true), NucGeneration::METEOR_LAKE);
  EXPECT_EQ(ClassifyGeneration(true, false, true), NucGeneration::ARROW_LAKE_H);
  EXPECT_EQ(ClassifyGeneration(true, false, false),
            NucGeneration::RAPTOR_LAKE_P_HT_OFF);
}
