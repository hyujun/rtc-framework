// ── closure_state_publisher 구현 ────────────────────────────────────────────────
#include "rtc_urdf_bridge/closure_state_publisher.hpp"

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace rtc_urdf_bridge {

namespace {
constexpr double kSingularWarnPeriodMs = 2000.0;
}  // namespace

ClosureStatePublisher::ClosureStatePublisher(const rclcpp::NodeOptions& options)
    : rclcpp::Node("closure_state_publisher", options) {
  // ── 파라미터 선언 ──────────────────────────────────────────────────────────
  const auto urdf_path = declare_parameter<std::string>("urdf_path", "");
  const auto robot_description = declare_parameter<std::string>("robot_description", "");
  const auto closure_path = declare_parameter<std::string>("closure_path", "");
  const auto root_joint_type = declare_parameter<std::string>("root_joint_type", "fixed");
  const auto input_topic =
      declare_parameter<std::string>("input_topic", "/digital_twin/actuated_joint_states");
  const auto output_topic =
      declare_parameter<std::string>("output_topic", "/digital_twin/joint_states");
  const bool warn_on_singular = declare_parameter<bool>("warn_on_singular", true);
  projection_opts_.max_iterations =
      static_cast<int>(declare_parameter<int>("max_iterations", projection_opts_.max_iterations));
  projection_opts_.tolerance = declare_parameter<double>("tolerance", projection_opts_.tolerance);

  if (closure_path.empty()) {
    throw std::runtime_error(
        "closure_state_publisher: 'closure_path' 파라미터가 필수입니다 (Extended-URDF sidecar).");
  }
  if (urdf_path.empty() && robot_description.empty()) {
    throw std::runtime_error(
        "closure_state_publisher: 'urdf_path' 또는 'robot_description' 중 하나는 지정해야 합니다.");
  }

  // ── closure model 빌드 (xacro 전처리 + closure 파이프라인 재사용) ─────────────
  ModelConfig config;
  config.urdf_path = urdf_path;
  config.urdf_xml_string = robot_description;
  config.root_joint_type = root_joint_type;
  config.closure_yaml_path = closure_path;

  PinocchioModelBuilder builder(config);
  model_ = *builder.GetFullModel();
  data_ = std::make_unique<pinocchio::Data>(model_);
  constraints_ = builder.GetConstraintModels();
  actuated_joint_ids_ = builder.GetClosureActuatedJointIds();
  q_full_ = builder.GetClosureReferenceConfig();

  if (q_full_.size() != model_.nq) {
    // closure sidecar 미소비(빈 q_ref) 방어 — neutral 로 fallback.
    q_full_ = pinocchio::neutral(model_);
  }
  if (constraints_.empty()) {
    RCLCPP_WARN(
        get_logger(),
        "closure 구속이 비어 있습니다 — passive 사영이 no-op 이 됩니다. closure_path 확인: %s",
        closure_path.c_str());
  }
  if (actuated_joint_ids_.empty()) {
    RCLCPP_WARN(
        get_logger(),
        "actuated joint 목록이 비어 있습니다 — 모든 DoF 가 사영됩니다 (actuated 고정 안 됨).");
  }
  if (warn_on_singular && builder.IsClosureReferenceSingular()) {
    RCLCPP_WARN(get_logger(),
                "closure 기준 형상(q_ref)이 특이(rank 결손)합니다 — neutral 근방에서 미수렴 시 "
                "직전 해를 hold 합니다 (NaN 방지).");
  }

  // ── 출력 관절 캐시: 전체 model 의 단일-DoF 관절 (universe(0) 제외) ──────────────
  //    q_full_ 전체를 JointState 로 publish 하므로 passive loop 관절도 포함한다.
  //    revolute(nq=1)·prismatic(nq=1)·continuous(nq=2, nv=1) 모두 스칼라 각/변위로
  //    표현 가능 — 판정은 nv==1 로 한다 (nq==1 만 보면 continuous 관절이 누락된다).
  output_names_.reserve(static_cast<std::size_t>(model_.njoints - 1));
  output_slots_.reserve(static_cast<std::size_t>(model_.njoints - 1));
  for (int jid = 1; jid < model_.njoints; ++jid) {
    const auto jidx = static_cast<std::size_t>(jid);
    if (model_.nvs[jidx] != 1) {
      continue;  // floating/planar/multi-DoF 등은 스칼라 JointState 로 표현 불가 → skip
    }
    output_names_.push_back(model_.names[jidx]);
    output_slots_.push_back({model_.idx_qs[jidx], model_.nqs[jidx] == 2});
  }

  // ── 입력 seed 맵: actuated 관절만 (measured actuated q 로 덮어쓸 슬롯) ───────────
  //    passive loop 관절은 warm-start seed(직전 loop-consistent 해)를 보존해야 하므로
  //    입력 맵에서 제외한다 — 입력 스트림이 passive 이름을 실어 보내도(예: 초기
  //    _publish_display 가 전체 관절을 0 으로 발행) seed 를 파괴하지 않는다.
  name_to_slot_.reserve(actuated_joint_ids_.size());
  for (const auto jid : actuated_joint_ids_) {
    if (jid == 0 || jid >= static_cast<pinocchio::JointIndex>(model_.njoints)) {
      continue;  // universe(0)/invalid — skip
    }
    const auto jidx = static_cast<std::size_t>(jid);
    if (model_.nvs[jidx] != 1) {
      continue;  // 스칼라 JointState 로 표현 가능한 단일-DoF actuated 관절만
    }
    name_to_slot_.emplace(model_.names[jidx],
                          JointSlot{model_.idx_qs[jidx], model_.nqs[jidx] == 2});
  }

  // ── pub/sub ─────────────────────────────────────────────────────────────────
  publisher_ = create_publisher<sensor_msgs::msg::JointState>(output_topic, rclcpp::QoS(10));
  subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState& msg) { OnJointState(msg); });

  RCLCPP_INFO(get_logger(),
              "closure_state_publisher 준비 완료: nq=%d, constraints=%zu, actuated=%zu, "
              "in='%s' out='%s'",
              model_.nq, constraints_.size(), actuated_joint_ids_.size(), input_topic.c_str(),
              output_topic.c_str());
}

void ClosureStatePublisher::OnJointState(const sensor_msgs::msg::JointState& msg) {
  // 직전 loop-consistent 해를 warm-start seed 로 복사하고 actuated 슬롯만 측정값으로 덮어쓴다.
  // 미수렴 시 q_full_ (직전 해) 을 그대로 hold 하기 위해 seed 는 별도 벡터에 둔다.
  Eigen::VectorXd q_seed = q_full_;
  const std::size_t n = std::min(msg.name.size(), msg.position.size());
  for (std::size_t i = 0; i < n; ++i) {
    const auto it = name_to_slot_.find(msg.name[i]);
    if (it == name_to_slot_.end()) {
      continue;
    }
    const JointSlot& slot = it->second;
    if (slot.is_continuous) {
      // continuous 관절: 스칼라 각 → (cos θ, sin θ) 로 seed.
      q_seed[slot.q_idx] = std::cos(msg.position[i]);
      q_seed[slot.q_idx + 1] = std::sin(msg.position[i]);
    } else {
      q_seed[slot.q_idx] = msg.position[i];
    }
  }

  const ProjectionResult res = ProjectPassiveToConstraint(model_, *data_, constraints_, q_seed,
                                                          actuated_joint_ids_, projection_opts_);

  if (res.converged && res.q.allFinite()) {
    q_full_ = res.q;
  } else {
    // 직전 해 hold (q_full_ 유지) — actuated 갱신도 버려 loop 를 닫힌 상태로 유지.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kSingularWarnPeriodMs,
                         "passive 사영 미수렴 (‖φ‖=%.3e, iters=%d) — 직전 해 hold.",
                         res.final_error, res.iterations);
  }

  // 입력 stamp 를 그대로 전달해 MuJoCo/actuated 스트림과 위상 정렬 (Q3: 콜백 추종).
  const rclcpp::Time stamp =
      (rclcpp::Time(msg.header.stamp).nanoseconds() != 0) ? rclcpp::Time(msg.header.stamp) : now();
  PublishState(stamp);
}

void ClosureStatePublisher::PublishState(const rclcpp::Time& stamp) {
  sensor_msgs::msg::JointState out;
  out.header.stamp = stamp;
  out.name = output_names_;
  out.position.resize(output_slots_.size());
  for (std::size_t i = 0; i < output_slots_.size(); ++i) {
    const JointSlot& slot = output_slots_[i];
    out.position[i] = slot.is_continuous ? std::atan2(q_full_[slot.q_idx + 1], q_full_[slot.q_idx])
                                         : q_full_[slot.q_idx];
  }
  publisher_->publish(std::move(out));
}

}  // namespace rtc_urdf_bridge
