// ── Xacro 전처리 테스트 ─────────────────────────────────────────────────────
#include "rtc_urdf_bridge/xacro_processor.hpp"
#include "rtc_urdf_bridge/urdf_analyzer.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace rub = rtc_urdf_bridge;

static std::string TestUrdfPath(const std::string & filename)
{
  std::filesystem::path p(__FILE__);
  return (p.parent_path() / "urdf" / filename).string();
}

// ═══════════════════════════════════════════════════════════════════════════════
// IsXacroFile 단위 테스트
// ═══════════════════════════════════════════════════════════════════════════════

TEST(XacroProcessorTest, IsXacroFilePositive)
{
  EXPECT_TRUE(rub::IsXacroFile("robot.urdf.xacro"));
  EXPECT_TRUE(rub::IsXacroFile("robot.xacro"));
  EXPECT_TRUE(rub::IsXacroFile("/full/path/to/model.urdf.xacro"));
}

TEST(XacroProcessorTest, IsXacroFileNegative)
{
  EXPECT_FALSE(rub::IsXacroFile("robot.urdf"));
  EXPECT_FALSE(rub::IsXacroFile("robot.xml"));
  EXPECT_FALSE(rub::IsXacroFile(""));
  EXPECT_FALSE(rub::IsXacroFile("xacro"));
}

// ═══════════════════════════════════════════════════════════════════════════════
// ProcessXacro 단위 테스트
// ═══════════════════════════════════════════════════════════════════════════════

TEST(XacroProcessorTest, ProcessSimpleXacro)
{
  auto xml = rub::ProcessXacro(TestUrdfPath("simple_robot.urdf.xacro"));

  // 출력 비어있지 않음
  EXPECT_FALSE(xml.empty());

  // URDF 유효성: <robot> 태그 존재
  EXPECT_NE(xml.find("<robot"), std::string::npos);
  EXPECT_NE(xml.find("base_link"), std::string::npos);

  // xacro 매크로 태그가 확장되어 없어야 함
  EXPECT_EQ(xml.find("<xacro:"), std::string::npos);

  // 3개 세그먼트 생성 확인
  EXPECT_NE(xml.find("joint_1"), std::string::npos);
  EXPECT_NE(xml.find("joint_2"), std::string::npos);
  EXPECT_NE(xml.find("joint_3"), std::string::npos);
}

TEST(XacroProcessorTest, ProcessXacroWithArgs)
{
  std::unordered_map<std::string, std::string> args = {{"link_length", "1.0"}};
  auto xml = rub::ProcessXacro(TestUrdfPath("simple_robot.urdf.xacro"), args);

  EXPECT_FALSE(xml.empty());
  EXPECT_NE(xml.find("<robot"), std::string::npos);
}

TEST(XacroProcessorTest, NonexistentFileThrows)
{
  EXPECT_THROW(
    rub::ProcessXacro("/nonexistent/path/robot.xacro"),
    std::runtime_error);
}

// ═══════════════════════════════════════════════════════════════════════════════
// UrdfAnalyzer + xacro 통합 테스트
// ═══════════════════════════════════════════════════════════════════════════════

class UrdfAnalyzerXacroTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    analyzer_ = std::make_unique<rub::UrdfAnalyzer>(
      TestUrdfPath("simple_robot.urdf.xacro"));
  }
  std::unique_ptr<rub::UrdfAnalyzer> analyzer_;
};

TEST_F(UrdfAnalyzerXacroTest, RootLinkDetection)
{
  EXPECT_EQ(analyzer_->GetRootLinkName(), "base_link");
}

TEST_F(UrdfAnalyzerXacroTest, LinkCount)
{
  // base_link + link_1 + link_2 + link_3 = 4
  EXPECT_EQ(analyzer_->GetNumLinks(), 4u);
}

TEST_F(UrdfAnalyzerXacroTest, ActuatedJointCount)
{
  EXPECT_EQ(analyzer_->GetActuatedJointNames().size(), 3u);
}

// ═══════════════════════════════════════════════════════════════════════════════
// PinocchioModelBuilder + xacro 통합 테스트
// ═══════════════════════════════════════════════════════════════════════════════

class ModelBuilderXacroTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rub::ModelConfig cfg;
    cfg.urdf_path = TestUrdfPath("simple_robot.urdf.xacro");
    cfg.root_joint_type = "fixed";
    cfg.sub_models.push_back({"arm", "base_link", "link_3"});
    builder_ = std::make_unique<rub::PinocchioModelBuilder>(cfg);
  }
  std::unique_ptr<rub::PinocchioModelBuilder> builder_;
};

TEST_F(ModelBuilderXacroTest, FullModelDimensions)
{
  auto model = builder_->GetFullModel();
  // 3 revolute joints → nq=3, nv=3
  EXPECT_EQ(model->nq, 3);
  EXPECT_EQ(model->nv, 3);
}

TEST_F(ModelBuilderXacroTest, ReducedModelDimensions)
{
  auto arm = builder_->GetReducedModel("arm");
  // base_link → link_3: 3개 관절 모두 포함
  EXPECT_EQ(arm->nq, 3);
  EXPECT_EQ(arm->nv, 3);
}

TEST_F(ModelBuilderXacroTest, XacroWithArgs)
{
  rub::ModelConfig cfg;
  cfg.urdf_path = TestUrdfPath("simple_robot.urdf.xacro");
  cfg.root_joint_type = "fixed";
  cfg.xacro_args = {{"link_length", "1.0"}};
  cfg.sub_models.push_back({"arm", "base_link", "link_3"});

  rub::PinocchioModelBuilder builder(cfg);
  auto model = builder.GetFullModel();

  // 토폴로지 동일: nq=3
  EXPECT_EQ(model->nq, 3);
}
