// ── Layer 2B payload 관성 회귀자 (#455) ─────────────────────────────────────
//
// 이 파일이 박는 계약은 넷이다.
//
//  1. Y_L 이 실제로 payload 토크를 재현한다 — oracle 은 "모델에 그 payload 를 실제로
//     붙였을 때의 RNEA 차분" 이다. 이 oracle 은 q̈ 를 자유롭게 고를 수 있으므로 런타임에서
//     결코 여기되지 않는 I 6열까지 전부 exercise 한다. 그것이 #455 [C2] 의 답이다:
//     I 는 API 계층에서 해석적으로 검증되고, 런타임에서는 추정되지 않는다.
//  2. 열 순서와 I 의 기준점. 둘 다 틀려도 유한하고 매끄러운 답이 나오므로 원소별로 박는다.
//  3. quasi-static 에서 I 6열이 정확히 0 이라는 사실 — 런타임이 4-param 만 추정하는 근거.
//  4. data.bodyRegressor 를 덮어쓰는 pinocchio side effect 가 우리 결과를 오염시키지 않는다.
#include "alloc_counter.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;
using rtc::urdf::test::AllocCounter;

namespace {

// 대각이 아닌 관성 텐서 — I_xy/I_xz/I_yz 가 0 이면 열 순서를 잘못 잡아도 통과한다.
// (CoM 기준으로 준다. toDynamicParameters() 는 ORIGIN 기준으로 돌려주므로 두 값이 다르고,
//  그 차이가 아래 InertiaIsAboutFrameOrigin 이 박는 것이다.)
constexpr double kPayloadMass = 1.7;

pinocchio::Inertia MakePayload() {
  const Eigen::Vector3d com(0.03, -0.05, 0.11);
  Eigen::Matrix3d inertia_com;
  inertia_com << 0.021, 0.004, -0.002, 0.004, 0.017, 0.003, -0.002, 0.003, 0.029;
  return {kPayloadMass, com, inertia_com};
}

// 자명하지 않은 (q, v, a) — 0 이나 neutral 이면 대부분의 열이 죽는다.
struct Sample {
  Eigen::VectorXd q, v, a;
};

Sample MakeSample(int nv, double seed) {
  Sample s;
  s.q = Eigen::VectorXd::Zero(nv);
  s.v = Eigen::VectorXd::Zero(nv);
  s.a = Eigen::VectorXd::Zero(nv);
  for (int i = 0; i < nv; ++i) {
    const double t = seed + 0.37 * static_cast<double>(i);
    s.q[i] = 0.6 * std::sin(t);
    s.v[i] = 0.9 * std::cos(1.3 * t);
    s.a[i] = 1.1 * std::sin(2.1 * t + 0.4);
  }
  return s;
}

std::vector<double> ToStd(const Eigen::VectorXd& v) {
  return {v.data(), v.data() + v.size()};
}

// serial_7dof 를 쓰는 이유는 축이 섞여 있기 때문이다 (Z 4개 + Y 3개). 이웃한
// serial_6dof.urdf 는 **여섯 축이 전부 Z 이고 링크도 Z 로만 뻗은 순수 Z-스택**이라 중력이
// 모든 관절축과 평행하다 — quasi-static 회귀자가 I 열뿐 아니라 **전체가 정확히 0** 이 되고,
// 관절값을 뒤집어도 결과가 안 바뀐다. 즉 이 파일의 거의 모든 단언이 vacuously 통과한다.
// 픽스처를 바꾸려면 먼저 아래 두 positive control (|left4| > 0, 뒤집은 입력이 다른 답) 이
// 살아 있는지 확인할 것.
class PayloadRegressorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rub::ModelConfig cfg;
    cfg.urdf_path = TestUrdfPath("serial_7dof.urdf");
    cfg.root_joint_type = "fixed";
    builder_ = std::make_unique<rub::PinocchioModelBuilder>(cfg);
    model_ = builder_->GetFullModel();
    handle_ = std::make_unique<rub::RtModelHandle>(model_);

    frame_id_ = model_->getFrameId(kFrame);
    ASSERT_LT(frame_id_, model_->frames.size()) << "fixture lost frame " << kFrame;

    // Oracle 모델: 같은 프레임에 payload 를 실제 강체로 붙인다.
    const auto& frame = model_->frames[frame_id_];
    payload_ = MakePayload();
    loaded_ = *model_;
    loaded_.appendBodyToJoint(frame.parentJoint, payload_, frame.placement);
    loaded_data_ = pinocchio::Data(loaded_);
    bare_data_ = pinocchio::Data(*model_);
  }

  static constexpr const char* kFrame = "tool_link";

  // τ_payload = rnea(payload 붙은 모델) − rnea(맨 모델). Y_L·φ 가 재현해야 할 대상.
  Eigen::VectorXd OracleTorque(const Sample& s) {
    const Eigen::VectorXd loaded = pinocchio::rnea(loaded_, loaded_data_, s.q, s.v, s.a);
    const Eigen::VectorXd bare = pinocchio::rnea(*model_, bare_data_, s.q, s.v, s.a);
    return loaded - bare;
  }

  std::unique_ptr<rub::PinocchioModelBuilder> builder_;
  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<rub::RtModelHandle> handle_;
  pinocchio::FrameIndex frame_id_{0};
  pinocchio::Inertia payload_{};
  pinocchio::Model loaded_;
  pinocchio::Data loaded_data_, bare_data_;
};

// ═══════════════════════════════════════════════════════════════════════════════
// AC 9 — oracle. I 6열을 포함해 전 10열이 여기서 검증된다.
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(PayloadRegressorTest, ReproducesRneaDifferenceUnderExcitation) {
  const Eigen::Matrix<double, 10, 1> phi = payload_.toDynamicParameters();

  double worst = 0.0;
  for (int k = 0; k < 6; ++k) {
    const Sample s = MakeSample(handle_->nv(), 0.11 * static_cast<double>(k) + 0.05);
    handle_->ComputePayloadRegressor(ToStd(s.q), ToStd(s.v), ToStd(s.a), frame_id_);

    const Eigen::VectorXd predicted = handle_->GetPayloadRegressor() * phi;
    const Eigen::VectorXd expected = OracleTorque(s);

    ASSERT_EQ(predicted.size(), expected.size());
    worst = std::max(worst, (predicted - expected).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(worst, 1e-10) << "Y_L phi does not reproduce the payload torque";
}

// 관성만 남긴 φ 로도 재현되는가 — I 열이 실제로 옳은지 격리해서 본다. 위 테스트는 m 항이
// 지배적이라 I 열이 통째로 틀려도 통과할 여지가 있다.
TEST_F(PayloadRegressorTest, InertiaColumnsCarryTheirOwnTorque) {
  const Sample s = MakeSample(handle_->nv(), 0.29);
  handle_->ComputePayloadRegressor(ToStd(s.q), ToStd(s.v), ToStd(s.a), frame_id_);
  const Eigen::MatrixXd Y = handle_->GetPayloadRegressor();

  // 질량 0, 1차 모멘트 0, 관성만 있는 (물리적으로는 비실현이지만 선형성 검증용) φ.
  Eigen::Matrix<double, 10, 1> phi_inertia = payload_.toDynamicParameters();
  phi_inertia.head<4>().setZero();

  // 같은 φ 를 모델에 실제로 붙여 RNEA 차분으로 대조한다.
  Eigen::Matrix3d inertia_origin;
  inertia_origin << phi_inertia[4], phi_inertia[5], phi_inertia[7], phi_inertia[5], phi_inertia[6],
      phi_inertia[8], phi_inertia[7], phi_inertia[8], phi_inertia[9];
  const pinocchio::Inertia pure(0.0, Eigen::Vector3d::Zero(), inertia_origin);

  const auto& frame = model_->frames[frame_id_];
  pinocchio::Model m2 = *model_;
  m2.appendBodyToJoint(frame.parentJoint, pure, frame.placement);
  pinocchio::Data d2(m2);
  pinocchio::Data d0(*model_);

  const Eigen::VectorXd expected =
      pinocchio::rnea(m2, d2, s.q, s.v, s.a) - pinocchio::rnea(*model_, d0, s.q, s.v, s.a);
  const Eigen::VectorXd predicted = Y * phi_inertia;

  EXPECT_LT((predicted - expected).cwiseAbs().maxCoeff(), 1e-10);
  // 그리고 그 토크가 실제로 0 이 아니어야 이 테스트가 의미를 갖는다 (vacuous 방지).
  EXPECT_GT(expected.cwiseAbs().maxCoeff(), 1e-6);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 열 순서와 I 의 기준점 — 둘 다 "그럴듯한 틀린 답" 을 내는 축이다
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(PayloadRegressorTest, ColumnOrderIsLowerTriangularColumnMajor) {
  const Eigen::Matrix<double, 10, 1> phi = payload_.toDynamicParameters();
  const Eigen::Vector3d com(0.03, -0.05, 0.11);

  EXPECT_DOUBLE_EQ(phi[0], kPayloadMass);
  EXPECT_DOUBLE_EQ(phi[1], kPayloadMass * com.x());
  EXPECT_DOUBLE_EQ(phi[2], kPayloadMass * com.y());
  EXPECT_DOUBLE_EQ(phi[3], kPayloadMass * com.z());

  // I 는 ORIGIN 기준이다. payload_.inertia() 는 CoM 기준(생성자에 넣은 값)을 그대로
  // 돌려주므로 여기서 평행축 항을 직접 더해 기준점을 명시한다.
  const Eigen::Matrix3d com_inertia = payload_.inertia().matrix();
  const Eigen::Matrix3d origin =
      com_inertia +
      kPayloadMass * (com.squaredNorm() * Eigen::Matrix3d::Identity() - com * com.transpose());

  // 순서는 [I_xx, I_xy, I_yy, I_xz, I_yz, I_zz] 다. I_xz 가 I_yy **뒤**에 온다는 것이
  // 이 단언의 전부다 — #455 본문 [MASS-B0] 은 [.., I_xz, I_yy, ..] 로 적었고 그것은 틀렸다.
  EXPECT_DOUBLE_EQ(phi[4], origin(0, 0));  // I_xx
  EXPECT_DOUBLE_EQ(phi[5], origin(0, 1));  // I_xy
  EXPECT_DOUBLE_EQ(phi[6], origin(1, 1));  // I_yy
  EXPECT_DOUBLE_EQ(phi[7], origin(0, 2));  // I_xz
  EXPECT_DOUBLE_EQ(phi[8], origin(1, 2));  // I_yz
  EXPECT_DOUBLE_EQ(phi[9], origin(2, 2));  // I_zz
}

TEST_F(PayloadRegressorTest, InertiaIsAboutFrameOriginNotCom) {
  const Eigen::Matrix<double, 10, 1> phi = payload_.toDynamicParameters();
  const Eigen::Vector3d com(0.03, -0.05, 0.11);

  // 접근자는 CoM 기준을 그대로 돌려준다 — 이 둘이 다르다는 것이 함정의 실체다.
  const Eigen::Matrix3d com_inertia = payload_.inertia().matrix();
  EXPECT_DOUBLE_EQ(com_inertia(0, 0), 0.021);

  // 그리고 φ 의 I_xx 는 그것과 다르며, 차이는 정확히 평행축 항 m·(c_y² + c_z²) 이다.
  // CoM 기준 값을 그대로 φ 에 넣으면 완전히 그럴듯한 틀린 답이 된다.
  EXPECT_GT(std::abs(phi[4] - com_inertia(0, 0)), 1e-6);
  EXPECT_NEAR(phi[4], com_inertia(0, 0) + kPayloadMass * (com.y() * com.y() + com.z() * com.z()),
              1e-12);
}

// ═══════════════════════════════════════════════════════════════════════════════
// [C2] — quasi-static 에서 I 는 방정식에 없다
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(PayloadRegressorTest, QuasiStaticInertiaColumnsAreExactlyZero) {
  const std::vector<double> zero(static_cast<std::size_t>(handle_->nv()), 0.0);

  for (int k = 0; k < 4; ++k) {
    const Sample s = MakeSample(handle_->nv(), 0.23 * static_cast<double>(k) + 0.07);
    handle_->ComputePayloadRegressor(ToStd(s.q), zero, zero, frame_id_);
    const Eigen::MatrixXd Y = handle_->GetPayloadRegressor();

    // 정확히 0 이다 — "작다" 가 아니라 0. 관측성이 나쁜 게 아니라 항이 없다.
    EXPECT_EQ(Y.rightCols<6>().cwiseAbs().maxCoeff(), 0.0)
        << "pose " << k << ": inertia columns must vanish without angular excitation";

    // 반면 m / m·c 4열은 살아 있어야 한다 (vacuous 방지).
    EXPECT_GT(Y.leftCols<4>().cwiseAbs().maxCoeff(), 1e-3);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// pinocchio side effect — data.bodyRegressor 는 공유 스크래치다
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(PayloadRegressorTest, SecondFrameDoesNotCorruptFirstResult) {
  const pinocchio::FrameIndex other = model_->getFrameId("link_3");
  ASSERT_NE(other, frame_id_);

  const Sample s = MakeSample(handle_->nv(), 0.41);
  handle_->ComputePayloadRegressor(ToStd(s.q), ToStd(s.v), ToStd(s.a), frame_id_);
  const Eigen::MatrixXd first = handle_->GetPayloadRegressor();

  // frameBodyRegressor 는 data.bodyRegressor 에 쓰고 그 참조를 반환한다. 핸들이 그 참조를
  // 붙들고 있었다면 이 호출이 first 를 뒤에서 바꿔놓는다.
  handle_->ComputePayloadRegressor(ToStd(s.q), ToStd(s.v), ToStd(s.a), other);
  const Eigen::MatrixXd second = handle_->GetPayloadRegressor();

  ASSERT_FALSE(first.isApprox(second)) << "두 프레임이 같은 회귀자를 내면 이 테스트는 무의미";

  // 첫 프레임을 다시 계산하면 원래 값으로 정확히 돌아와야 한다.
  handle_->ComputePayloadRegressor(ToStd(s.q), ToStd(s.v), ToStd(s.a), frame_id_);
  EXPECT_TRUE(handle_->GetPayloadRegressor().isApprox(first, 0.0));
}

// ═══════════════════════════════════════════════════════════════════════════════
// 좌표 순서 계약 — PR #452 가 이 축에서 "유한하고 매끄러운 틀린 답" 을 실증했다
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(PayloadRegressorTest, InputsAreDeviceOrderAndRowsStayPinocchioOrder) {
  // 외부(device) 순서를 pinocchio 순서의 역순으로 잡는다 — non-identity 보장.
  std::vector<std::string> pin_names = handle_->GetPinocchioJointNames();
  ASSERT_EQ(pin_names.size(), 7u);
  std::vector<std::string> device_order(pin_names.rbegin(), pin_names.rend());
  ASSERT_NE(device_order, pin_names);

  rub::RtModelHandle reordered(model_);
  ASSERT_TRUE(reordered.SetJointOrder(device_order));
  ASSERT_TRUE(reordered.HasJointReorder());

  const Sample s = MakeSample(handle_->nv(), 0.53);

  // 같은 물리 상태를 두 순서로 표현한다.
  const int nv = handle_->nv();
  Eigen::VectorXd q_dev(nv), v_dev(nv), a_dev(nv);
  for (int i = 0; i < nv; ++i) {
    q_dev[i] = s.q[nv - 1 - i];
    v_dev[i] = s.v[nv - 1 - i];
    a_dev[i] = s.a[nv - 1 - i];
  }

  handle_->ComputePayloadRegressor(ToStd(s.q), ToStd(s.v), ToStd(s.a), frame_id_);
  const Eigen::MatrixXd identity_result = handle_->GetPayloadRegressor();

  reordered.ComputePayloadRegressor(ToStd(q_dev), ToStd(v_dev), ToStd(a_dev), frame_id_);
  const Eigen::MatrixXd reordered_result = reordered.GetPayloadRegressor();

  // 입력은 재배열되고 행은 pinocchio 순서로 남는다 → 두 결과가 행까지 동일해야 한다.
  // 행이 device 순서로 나온다면 이 단언이 뒤집힌 행에서 깨진다.
  EXPECT_TRUE(reordered_result.isApprox(identity_result, 1e-12))
      << "rows must stay in PINOCCHIO order (GetTau() 와 같은 계약)";

  // 재배열이 실제로 무언가를 했는지 — device 순서 입력을 그대로 identity 핸들에 넣으면
  // 다른 답이 나와야 한다 (positive control).
  handle_->ComputePayloadRegressor(ToStd(q_dev), ToStd(v_dev), ToStd(a_dev), frame_id_);
  EXPECT_FALSE(handle_->GetPayloadRegressor().isApprox(identity_result, 1e-12));
}

// ═══════════════════════════════════════════════════════════════════════════════
// RT 계약 — 무할당
// ═══════════════════════════════════════════════════════════════════════════════

TEST_F(PayloadRegressorTest, ComputeIsAllocationFree) {
  const Sample s = MakeSample(handle_->nv(), 0.61);
  const std::vector<double> q = ToStd(s.q), v = ToStd(s.v), a = ToStd(s.a);

  handle_->ComputePayloadRegressor(q, v, a, frame_id_);  // warm-up

  AllocCounter::Arm();
  for (int i = 0; i < 100; ++i) {
    handle_->ComputePayloadRegressor(q, v, a, frame_id_);
  }
  AllocCounter::Disarm();

  EXPECT_EQ(AllocCounter::alloc_count.load(), 0)
      << "ComputePayloadRegressor allocated on the RT path";
}

}  // namespace
