// ── 관성 실현가능성 게이트 (V5 / V6) 단위 테스트 ─────────────────────────────
//
// 합성 URDF 한 쌍(base_link + probe_link)에 관성 블록만 갈아 끼워, 각 결함이
// **자기 사유로만** 발현하는지 확인한다. 사유가 서로 새면 게이트가 "무언가
// 틀렸다"까지만 말하고 무엇이 틀렸는지는 못 말하게 되는데, 그러면 사용자는
// 로드 실패를 디버깅할 수 없다.
//
// tolerance 축이 이 파일의 두 번째 주제다. 판정은 주모멘트 크기로 정규화되며
// (`kInertialRelTol`), 그 이유는 저장소 실모델의 주모멘트가 6.8 decade 에
// 걸쳐 있어 절대 tol 로는 양 끝을 동시에 만족시킬 수 없기 때문이다.
#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "rtc_urdf_bridge/inertial_validation.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

// ── 합성 URDF 조립 ──────────────────────────────────────────────────────────

/// 배정도 수치를 **무손실**로 문자열화. `std::to_string` 은 소수점 6자리 고정이라
/// 3e-12 를 "0.000000" 으로 만든다 — 그러면 스케일 스윕이 조용히 영텐서를
/// 검사하게 되어 통과 이유가 뒤바뀐다.
std::string Num(double value) {
  std::ostringstream oss;
  oss << std::setprecision(17) << value;
  return oss.str();
}

/// 관성 블록 하나를 문자열로. 6 성분을 모두 노출해 위반 수치가 assertion 바로
/// 옆에 보이게 한다.
std::string InertialBlock(const std::string& mass, const std::string& ixx, const std::string& iyy,
                          const std::string& izz, const std::string& ixy = "0",
                          const std::string& ixz = "0", const std::string& iyz = "0") {
  return "<inertial><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/><mass value=\"" + mass +
         "\"/><inertia ixx=\"" + ixx + "\" ixy=\"" + ixy + "\" ixz=\"" + ixz + "\" iyy=\"" + iyy +
         "\" iyz=\"" + iyz + "\" izz=\"" + izz + "\"/></inertial>";
}

/// base_link(고정 베이스 → universe 로 흡수) + probe_link(revolute) 2링크 URDF.
/// 검증 대상은 probe_link 하나뿐이므로 사유 교차 오염을 명확히 볼 수 있다.
std::string ProbeUrdf(const std::string& probe_inertial) {
  return R"(<?xml version="1.0"?>
<robot name="inertial_probe">
  <link name="base_link">)" +
         InertialBlock("1.0", "0.1", "0.1", "0.1") + R"(</link>
  <link name="probe_link">)" +
         probe_inertial + R"(</link>
  <joint name="probe_joint" type="revolute">
    <parent link="base_link"/>
    <child link="probe_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>
</robot>)";
}

/// URDF 문자열 → Pinocchio 모델 (빌더를 거치지 않는 직접 경로 — 게이트가
/// throw 하는 경우에도 판정 내용을 들여다볼 수 있어야 한다).
pinocchio::Model BuildRaw(const std::string& xml) {
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(xml, model);
  return model;
}

rub::InertialValidationReport ValidateProbe(const std::string& probe_inertial,
                                            double rel_tol = rub::kInertialRelTol) {
  const pinocchio::Model model = BuildRaw(ProbeUrdf(probe_inertial));
  return rub::ValidateInertias(model, rel_tol);
}

/// 정확히 1건의 fatal 이 있고 그 종류가 expected 인지. degenerate 는 비어야 한다
/// (사유 교차 오염 검출).
void ExpectSingleFatal(const rub::InertialValidationReport& report, rub::InertialDefect expected) {
  ASSERT_EQ(report.fatal.size(), 1u) << "기대: fatal 1건. 실제:\n"
                                     << rub::DescribeInertialViolations(report.fatal);
  EXPECT_EQ(report.fatal[0].defect, expected)
      << "사유가 다르다 — 기대 '" << rub::ToString(expected) << "', 실제 '"
      << rub::ToString(report.fatal[0].defect) << "'";
  EXPECT_EQ(report.fatal[0].body_name, "probe_link");
  EXPECT_EQ(report.fatal[0].joint_name, "probe_joint");
  EXPECT_TRUE(report.degenerate.empty()) << "fatal 인 body 가 degenerate 레인에도 샜다:\n"
                                         << rub::DescribeInertialViolations(report.degenerate);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 정상 통과 — 게이트가 멀쩡한 모델을 막지 않는가
// ═══════════════════════════════════════════════════════════════════════════

TEST(InertialValidation, PhysicallyValidBodyPasses) {
  const auto report = ValidateProbe(InertialBlock("2.0", "0.03", "0.04", "0.05"));
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
  EXPECT_TRUE(report.degenerate.empty()) << rub::DescribeInertialViolations(report.degenerate);
}

// 얇은 판·막대는 I1+I2 = I3 이 **정확히** 성립한다. 등호를 위반으로 읽으면
// 정당한 강체를 거부하므로, 판정은 반드시 `>= -tol` 이어야 한다.
TEST(InertialValidation, TriangleEqualityIsAccepted) {
  // 균질 얇은 판: Ixx=Iyy=I, Izz=2I.
  const auto report = ValidateProbe(InertialBlock("1.0", "0.25", "0.25", "0.5"));
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
}

// 대각이 아닌 텐서도 주모멘트(고유값)로 판정된다 — 관성 프레임의 회전은
// 물리적 실현가능성과 무관하다.
TEST(InertialValidation, OffDiagonalTensorJudgedByPrincipalMoments) {
  // 유효 텐서를 회전시킨 형태 (off-diagonal 존재, 고유값은 여전히 유효).
  const auto report = ValidateProbe(InertialBlock("1.0", "0.04", "0.04", "0.05", "0.005"));
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
}

// 고정 베이스의 root body 는 어떤 DoF 도 지지 않아 M(q) 에 들어가지 않는다.
// universe(jid 0) 로 흡수되므로 판정 대상이 아니다 — 이후 "왜 base_link 는
// 검사 안 하나" 로 게이트를 넓히지 않도록 계약으로 고정한다.
TEST(InertialValidation, FixedBaseRootBodyIsNotValidated) {
  const std::string xml = R"(<?xml version="1.0"?>
<robot name="bad_root">
  <link name="base_link">)" +
                          InertialBlock("1.0", "9e-7", "2e-6", "3e-6") + R"(</link>
  <link name="probe_link">)" +
                          InertialBlock("1.0", "0.03", "0.04", "0.05") + R"(</link>
  <joint name="probe_joint" type="revolute">
    <parent link="base_link"/><child link="probe_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>
</robot>)";
  const auto report = rub::ValidateInertias(BuildRaw(xml));
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
}

// ═══════════════════════════════════════════════════════════════════════════
// V6 — 물리적으로 불가능 (각 사유가 자기 것만 발현)
// ═══════════════════════════════════════════════════════════════════════════

// **비유한 값은 URDF 텍스트로는 게이트에 도달하지 못한다** — urdfdom 이 파싱
// 단계에서 "Could not parse inertial element" 로 먼저 거부한다 (2026-08-17 실측).
// 그래서 이 세 테스트는 모델을 **프로그램적으로** 오염시킨다.
//
// 이 가드는 URDF 텍스트 경로에 대한 **심층 방어**이지 현재 살아 있는 production
// 경로를 막는 것이 아니다 (2026-08-17 전수 grep: `model.inertias` 를 변형하는
// production 코드 0건 — `appendBodyToJoint` 는 `test_payload_regressor.cpp` 에만
// 있다). 그래도 지우면 안 되는 이유는 두 가지다: ① `pinocchio::Model` 을 직접
// 조립·변형하는 경로가 API 로 열려 있고 (payload 를 프레임에 얹는 형태가 그
// 후보다) 그 값이 게이트를 통과하면 `crba` 가 통째로 NaN 인 M(q) 를 낸다,
// ② 고유값 분해 실패 분기는 유한한 입력에서도 발현할 수 있다. 단 게이트는
// `BuildFullModel()` 안에서 **1회만** 돌므로 로드 후 오염은 잡지 못한다.
TEST(InertialValidation, NonFiniteMassIsFatal) {
  pinocchio::Model model = BuildRaw(ProbeUrdf(InertialBlock("1.0", "0.03", "0.04", "0.05")));
  model.inertias[1] =
      pinocchio::Inertia(std::numeric_limits<double>::quiet_NaN(), Eigen::Vector3d::Zero(),
                         pinocchio::Symmetric3::Identity());
  ExpectSingleFatal(rub::ValidateInertias(model), rub::InertialDefect::kNonFinite);
}

TEST(InertialValidation, NonFiniteInertiaIsFatal) {
  pinocchio::Model model = BuildRaw(ProbeUrdf(InertialBlock("1.0", "0.03", "0.04", "0.05")));
  Eigen::Matrix3d corrupted = Eigen::Matrix3d::Identity() * 0.04;
  corrupted(0, 0) = std::numeric_limits<double>::infinity();
  model.inertias[1] =
      pinocchio::Inertia(1.0, Eigen::Vector3d::Zero(), pinocchio::Symmetric3(corrupted));
  ExpectSingleFatal(rub::ValidateInertias(model), rub::InertialDefect::kNonFinite);
}

// mass 와 CoM 기준 관성이 모두 멀쩡해도 lever(CoM 위치)가 비유한이면 평행축
// 이동에서 M(q) 가 통째로 NaN 이 된다. Inertia 가 lever 를 회전관성과 분리해
// 들고 있어 inertia().allFinite() 만으로는 잡히지 않는 경로다.
TEST(InertialValidation, NonFiniteComOffsetIsFatal) {
  pinocchio::Model model = BuildRaw(ProbeUrdf(InertialBlock("1.0", "0.03", "0.04", "0.05")));
  const Eigen::Vector3d bad_com(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  model.inertias[1] = pinocchio::Inertia(1.0, bad_com, pinocchio::Symmetric3::Identity());
  ASSERT_TRUE(model.inertias[1].inertia().matrix().allFinite());  // 회전관성은 멀쩡하다
  ExpectSingleFatal(rub::ValidateInertias(model), rub::InertialDefect::kNonFinite);
}

TEST(InertialValidation, NegativeMassIsFatal) {
  ExpectSingleFatal(ValidateProbe(InertialBlock("-1.0", "0.03", "0.04", "0.05")),
                    rub::InertialDefect::kNegativeMass);
}

TEST(InertialValidation, NegativeEigenvalueIsFatal) {
  ExpectSingleFatal(ValidateProbe(InertialBlock("1.0", "-0.03", "0.04", "0.05")),
                    rub::InertialDefect::kNotPositiveSemidefinite);
}

// 실제 저장소 자산에서 관측된 위반 그대로다 (schunk_svh_hand_* 의 *_hand_p):
// 9e-7 + 2e-6 = 2.9e-6 < 3e-6 — 상대 위반 -3.33e-2.
TEST(InertialValidation, TriangleInequalityViolationIsFatal) {
  const auto report = ValidateProbe(InertialBlock("0.024", "9e-7", "2e-6", "3e-6"));
  // ASSERT_* 는 헬퍼 **안에서만** 반환하므로, 감싸지 않으면 헬퍼가 실패해도 호출부가
  // 계속 진행해 빈 vector 를 인덱싱한다 (tolerance mutation 때 실제로 SEGV 가 나
  // 타깃 전체가 결과 없이 죽었다 — mutation 신호가 통째로 안 읽히는 형태다).
  ASSERT_NO_FATAL_FAILURE(ExpectSingleFatal(report, rub::InertialDefect::kTriangleInequality));
  EXPECT_NEAR(report.fatal[0].margin, -1.0 / 30.0, 1e-9);
}

// ═══════════════════════════════════════════════════════════════════════════
// V5 — 물리적으로 불완전 (경고 레인, 로드는 성공)
// ═══════════════════════════════════════════════════════════════════════════

TEST(InertialValidation, MasslessMovableBodyIsDegenerateNotFatal) {
  const auto report = ValidateProbe(InertialBlock("0.0", "0", "0", "0"));
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
  ASSERT_EQ(report.degenerate.size(), 1u);
  EXPECT_EQ(report.degenerate[0].defect, rub::InertialDefect::kMasslessMovableBody);
  EXPECT_EQ(report.degenerate[0].body_name, "probe_link");
}

// <inertial> 태그가 아예 없는 movable link 도 같은 레인이다 (Pinocchio 가 영관성
// body 로 채운다) — 실모델에서 이 형태가 압도적으로 흔하다.
TEST(InertialValidation, MissingInertialTagOnMovableBodyIsDegenerate) {
  const auto report = ValidateProbe("");
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
  ASSERT_EQ(report.degenerate.size(), 1u);
  EXPECT_EQ(report.degenerate[0].defect, rub::InertialDefect::kMasslessMovableBody);
}

// mass 와 관성이 **어긋나면** V5 가 아니라 V6 다 — 질량 0 강체는 회전관성도 0
// 이어야 하므로 어떤 질량분포로도 실현할 수 없다. V5 는 "둘 다 0" 일 때만이다.
TEST(InertialValidation, MasslessBodyWithNonZeroInertiaIsFatal) {
  const auto report = ValidateProbe(InertialBlock("0.0", "0.03", "0.04", "0.05"));
  ASSERT_NO_FATAL_FAILURE(ExpectSingleFatal(report, rub::InertialDefect::kMasslessWithInertia));
  EXPECT_EQ(report.fatal[0].mass, 0.0);
}

// 반대 방향 — 텐서가 0 이면 같은 mass 0 이라도 V5 로 남아야 한다 (레인 교차 오염).
TEST(InertialValidation, MasslessBodyWithZeroInertiaStaysDegenerate) {
  const auto report = ValidateProbe(InertialBlock("0.0", "0", "0", "0"));
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
  ASSERT_EQ(report.degenerate.size(), 1u);
  EXPECT_EQ(report.degenerate[0].defect, rub::InertialDefect::kMasslessMovableBody);
}

// V5 는 로드를 막지 않는다 — kinematics 전용 소비자를 위한 결정이다.
TEST(InertialValidation, BuilderLoadsModelWithMasslessBodyButFlagsIt) {
  rub::ModelConfig cfg;
  cfg.urdf_xml_string = ProbeUrdf(InertialBlock("0.0", "0", "0", "0"));
  std::unique_ptr<rub::PinocchioModelBuilder> builder;
  ASSERT_NO_THROW(builder = std::make_unique<rub::PinocchioModelBuilder>(cfg));
  EXPECT_EQ(builder->GetFullModel()->nv, 1);  // 모델은 정상 사용 가능
  EXPECT_FALSE(builder->IsFullModelDynamicsCapable());
  EXPECT_EQ(builder->GetInertialReport().degenerate.size(), 1u);
}

// V6 는 로드를 막는다.
TEST(InertialValidation, BuilderThrowsOnPhysicallyImpossibleInertia) {
  rub::ModelConfig cfg;
  cfg.urdf_xml_string = ProbeUrdf(InertialBlock("0.024", "9e-7", "2e-6", "3e-6"));
  EXPECT_THROW(rub::PinocchioModelBuilder{cfg}, std::runtime_error);
}

TEST(InertialValidation, BuilderReportsCleanModelAsDynamicsCapable) {
  rub::ModelConfig cfg;
  cfg.urdf_xml_string = ProbeUrdf(InertialBlock("2.0", "0.03", "0.04", "0.05"));
  const rub::PinocchioModelBuilder builder(cfg);
  EXPECT_TRUE(builder.IsFullModelDynamicsCapable());
  EXPECT_TRUE(builder.GetInertialReport().fatal.empty());
  EXPECT_TRUE(builder.GetInertialReport().degenerate.empty());
}

// ═══════════════════════════════════════════════════════════════════════════
// tolerance 축 — scale-aware 여야 하는 이유를 테스트로 고정
// ═══════════════════════════════════════════════════════════════════════════

// 같은 **상대** 위반량을 6 decade 떨어진 두 스케일에 두면 둘 다 잡혀야 한다.
// 절대 tol 구현은 작은 쪽을 놓친다 — 위반의 절대 크기(~3e-14)가 어떤 실용적
// 절대 임계보다도 작기 때문이다. 이 테스트가 그 대체 구현을 죽인다.
TEST(InertialValidation, ScaleAwareToleranceCatchesViolationAtAnyScale) {
  // 큰 스케일: 0.9 + 2.0 = 2.9 < 3.0
  ExpectSingleFatal(ValidateProbe(InertialBlock("1.0", "0.9", "2.0", "3.0")),
                    rub::InertialDefect::kTriangleInequality);
  // 작은 스케일: 같은 비율, 절대 결손은 1e-13 수준.
  ExpectSingleFatal(ValidateProbe(InertialBlock("1e-6", "9e-13", "2e-12", "3e-12")),
                    rub::InertialDefect::kTriangleInequality);
}

// 반대 방향 — 정당한 강체는 어느 스케일에서도 통과해야 한다. 저장소 실모델의
// 주모멘트 범위(1.03e-7 ~ 7.07e-1)를 양쪽으로 넉넉히 넘겨 훑는다.
TEST(InertialValidation, ValidBodyPassesAtAnyScale) {
  for (const double scale : {1e-12, 1e-7, 1e-3, 1e0, 1e2}) {
    const auto report =
        ValidateProbe(InertialBlock("1.0", Num(3.0 * scale), Num(4.0 * scale), Num(5.0 * scale)));
    EXPECT_TRUE(report.fatal.empty()) << "scale=" << scale << "\n"
                                      << rub::DescribeInertialViolations(report.fatal);
  }
}

// 저장소 실모델의 **가장 빡빡한 정당 사례** (panda_link2, 상대 여유 +5.53e-5)
// 를 재현한다. 임계를 이 값 근처까지 느슨하게 올리면 이 테스트가 죽는다.
TEST(InertialValidation, TightestRealWorldValidMarginStillPasses) {
  // I1+I2-I3 = 5.53e-5 * I3 이 되도록 구성.
  const double i3 = 1.0;
  const double i2 = 0.6;
  const double i1 = i3 - i2 + 5.5260e-5 * i3;
  const auto report = ValidateProbe(InertialBlock("1.0", Num(i1), Num(i2), Num(i3)));
  EXPECT_TRUE(report.fatal.empty()) << rub::DescribeInertialViolations(report.fatal);
}

// rel_tol **주입 경로**. 이 인자를 명시로 넘기는 테스트가 하나도 없어 (2026-08-17
// 리뷰 지적) 임계가 인자로 흐르는지 자체가 미행사였다 — mutation 은 상수를 고쳐서
// 하므로 이 축을 대신하지 못한다.
TEST(InertialValidation, ExplicitRelToleranceWidensAcceptance) {
  // 상대 결손 -1e-3 인 텐서: 기본 임계(1e-9)에서는 거부.
  const double i3 = 1.0;
  const double i2 = 0.6;
  const double i1 = i3 - i2 - 1e-3 * i3;
  const std::string block = InertialBlock("1.0", Num(i1), Num(i2), Num(i3));
  ASSERT_NO_FATAL_FAILURE(
      ExpectSingleFatal(ValidateProbe(block), rub::InertialDefect::kTriangleInequality));

  // 같은 텐서를 1e-2 로 부르면 통과한다 — 인자가 실제로 판정을 움직인다.
  const auto loose = ValidateProbe(block, 1e-2);
  EXPECT_TRUE(loose.fatal.empty()) << rub::DescribeInertialViolations(loose.fatal);
}

// ═══════════════════════════════════════════════════════════════════════════
// 진입점 계약 — URDF → Model 문은 모두 게이트를 지난다
// ═══════════════════════════════════════════════════════════════════════════

// `PinocchioModelBuilder` 만이 로드 진입점이 아니다. 폐쇄 체인 진입점은 pinocchio
// 를 직접 불러 한동안 게이트 밖에 있었고 (#316 D-5), 그 사실이 builder 헤더의
// "유일한 관문" 주장을 거짓으로 만들고 있었다. 이 테스트가 그 문을 닫아 둔다.
TEST(InertialValidation, ClosedChainEntryPointIsGatedBeforeSidecarParse) {
  const std::filesystem::path probe =
      std::filesystem::temp_directory_path() / "rtc_inertial_gate_probe.urdf";
  {
    std::ofstream out(probe);
    ASSERT_TRUE(out) << probe.string();
    out << ProbeUrdf(InertialBlock("0.024", "9e-7", "2e-6", "3e-6"));
  }

  // sidecar 경로는 **일부러 존재하지 않는 것**을 준다. 게이트가 (1) 모델 빌드 직후에
  // 발사되면 관성 사유로 죽고, 파싱 뒤로 밀리면 sidecar 오류로 죽는다 — 메시지로
  // 둘을 가르므로 "다른 이유로 throw 해서 green" 이 되지 않는다.
  try {
    rub::BuildClosedChainModelFromExtendedUrdf(probe.string(), "/nonexistent.closure.yaml");
    ADD_FAILURE() << "폐쇄 체인 진입점이 V6 모델을 그대로 열었다";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("BuildClosedChainModelFromExtendedUrdf"), std::string::npos) << msg;
    EXPECT_NE(msg.find("V6:triangle-inequality"), std::string::npos) << msg;
  }
  std::filesystem::remove(probe);
}

// ═══════════════════════════════════════════════════════════════════════════
// 보고 계약 — 결정적 순서
// ═══════════════════════════════════════════════════════════════════════════

TEST(InertialValidation, ViolationsReportedInJointIndexOrder) {
  const std::string xml = R"(<?xml version="1.0"?>
<robot name="two_bad">
  <link name="base_link">)" +
                          InertialBlock("1.0", "0.1", "0.1", "0.1") + R"(</link>
  <link name="link_a">)" + InertialBlock("0.024", "9e-7", "2e-6", "3e-6") +
                          R"(</link>
  <link name="link_b">)" + InertialBlock("1.0", "-0.03", "0.04", "0.05") +
                          R"(</link>
  <joint name="joint_a" type="revolute">
    <parent link="base_link"/><child link="link_a"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <joint name="joint_b" type="revolute">
    <parent link="link_a"/><child link="link_b"/>
    <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>)";
  const auto report = rub::ValidateInertias(BuildRaw(xml));
  ASSERT_EQ(report.fatal.size(), 2u) << rub::DescribeInertialViolations(report.fatal);
  EXPECT_EQ(report.fatal[0].joint_name, "joint_a");
  EXPECT_EQ(report.fatal[0].defect, rub::InertialDefect::kTriangleInequality);
  EXPECT_EQ(report.fatal[1].joint_name, "joint_b");
  EXPECT_EQ(report.fatal[1].defect, rub::InertialDefect::kNotPositiveSemidefinite);
}

TEST(InertialValidation, DescriptionNamesDefectLinkAndNumbers) {
  const auto report = ValidateProbe(InertialBlock("0.024", "9e-7", "2e-6", "3e-6"));
  const std::string text = rub::DescribeInertialViolations(report.fatal);
  EXPECT_NE(text.find("V6:triangle-inequality"), std::string::npos) << text;
  EXPECT_NE(text.find("probe_link"), std::string::npos) << text;
  EXPECT_NE(text.find("probe_joint"), std::string::npos) << text;
}
