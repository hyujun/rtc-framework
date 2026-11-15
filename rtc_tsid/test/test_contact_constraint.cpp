#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include "panda_fixture.hpp"
#include "rtc_tsid/constraints/contact_constraint.hpp"

#include <cstring>
#include <string>
#include <utility>

namespace rtc::tsid {
namespace {

class ContactConstraintTest : public test::PandaTest {
 protected:
  void SetUp() override {
    test::PandaTest::SetUp();
    // Find a valid frame for contact
    for (size_t i = 0; i < model_->frames.size(); ++i) {
      if (model_->frames[i].name.find("link7") != std::string::npos) {
        frame_name_ = model_->frames[i].name;
        break;
      }
    }
    ASSERT_FALSE(frame_name_.empty());

    // 1 point contact
    contact_cfg_.contacts.resize(1);
    contact_cfg_.contacts[0].name = "ee";
    contact_cfg_.contacts[0].frame_name = frame_name_;
    contact_cfg_.contacts[0].frame_id = static_cast<int>(model_->getFrameId(frame_name_));
    contact_cfg_.contacts[0].contact_dim = 3;
    contact_cfg_.contacts[0].friction_coeff = 0.7;
    contact_cfg_.contacts[0].friction_faces = 4;
    contact_cfg_.max_contacts = 1;
    contact_cfg_.max_contact_vars = 3;

    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg_));

    contacts_.Init(1);
    contacts_.contacts[0].active = true;
    contacts_.RecomputeActive(contact_cfg_);
  }

  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  std::string frame_name_;
};

// Stage A-5a: ContactConstraint::EqDim returns max_contact_vars (fixed)
// regardless of activation. Inactive contacts contribute zero rows so the
// QP sees a stable equality block size across contact on/off transitions.
TEST_F(ContactConstraintTest, Dimensions) {
  ContactConstraint cc;
  YAML::Node cfg;
  cc.Init(*model_, robot_info_, cache_, cfg);
  cc.SetContactManager(&contact_cfg_);

  EXPECT_EQ(cc.EqDim(contacts_), 3);  // max_contact_vars = 3 (1 point contact)
  EXPECT_EQ(cc.IneqDim(contacts_), 0);

  // Deactivate → eq_dim stays at max_contact_vars; inactive rows must be
  // zero so the equality is trivially satisfied (0·a = 0).
  contacts_.contacts[0].active = false;
  contacts_.RecomputeActive(contact_cfg_);
  EXPECT_EQ(cc.EqDim(contacts_), 3);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);
  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  Eigen::MatrixXd A(3, n_vars);
  Eigen::VectorXd b(3);
  A.setZero();
  b.setZero();
  cc.ComputeEquality(cache_, contacts_, robot_info_, n_vars, A, b);
  EXPECT_LT(A.norm(), 1e-15);
  EXPECT_LT(b.norm(), 1e-15);
}

TEST_F(ContactConstraintTest, MatrixShape) {
  ContactConstraint cc;
  YAML::Node cfg;
  cc.Init(*model_, robot_info_, cache_, cfg);
  cc.SetContactManager(&contact_cfg_);

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  // Stage A-5a: n_vars uses max_contact_vars (fixed) rather than
  // active_contact_vars — the QP variable dimension is constant.
  const int n_vars = robot_info_.nv + contact_cfg_.max_contact_vars;
  const int n_eq = cc.EqDim(contacts_);

  Eigen::MatrixXd A(n_eq, n_vars);
  Eigen::VectorXd b(n_eq);
  A.setZero();
  b.setZero();

  cc.ComputeEquality(cache_, contacts_, robot_info_, n_vars, A, b);

  // A[:, 0:nv] = Jc[:3, :] (should be non-zero on the active contact's row block)
  EXPECT_GT(A.leftCols(robot_info_.nv).norm(), 0.0);
  // A[:, nv:] = 0 (no coupling with λ in contact constraint)
  EXPECT_NEAR(A.rightCols(contact_cfg_.max_contact_vars).norm(), 0.0, 1e-15);
}

// ── inactive_lambda ──────────────────────────────────────────────────────────
//
// Three contacts of mixed dimension (point, surface, point) on the Panda, so a
// slot's λ columns are offset by the dimensions before it.
class InactiveLambdaTest : public test::PandaTest {
 protected:
  void SetUp() override {
    test::PandaTest::SetUp();
    const char* links[3] = {"link5", "link6", "link7"};
    const int dims[3] = {3, 6, 3};
    cfg_.contacts.resize(3);
    for (std::size_t i = 0; i < 3; ++i) {
      std::string frame;
      for (const auto& f : model_->frames) {
        if (f.name.find(links[i]) != std::string::npos) {
          frame = f.name;
          break;
        }
      }
      ASSERT_FALSE(frame.empty()) << links[i];
      auto& c = cfg_.contacts[i];
      c.name = links[i];
      c.frame_name = frame;
      c.frame_id = static_cast<int>(model_->getFrameId(frame));
      c.contact_dim = dims[i];
    }
    cfg_.max_contacts = 3;
    cfg_.max_contact_vars = 12;
    cache_.Init(model_, rtc::tsid::ContactFrameIds(cfg_));
    contacts_.Init(3);
    for (auto& c : contacts_.contacts) {
      c.active = true;
    }
    contacts_.RecomputeActive(cfg_);
    Eigen::VectorXd q = pinocchio::neutral(*model_);
    q(1) = 0.4;
    q(3) = -1.2;
    Eigen::VectorXd v = Eigen::VectorXd::Constant(robot_info_.nv, 0.3);
    cache_.Update(q, v);
  }

  int NVars() const { return robot_info_.nv + cfg_.max_contact_vars; }

  // The rows `mode` writes on this tick.
  void Rows(const char* mode, Eigen::MatrixXd& A, Eigen::VectorXd& b) {
    ContactConstraint cc;
    YAML::Node yaml;
    if (mode != nullptr) {
      yaml["inactive_lambda"] = mode;
    }
    cc.Init(*model_, robot_info_, cache_, yaml);
    cc.SetContactManager(&cfg_);
    A.setZero(cc.EqDim(contacts_), NVars());
    b.setZero(cc.EqDim(contacts_));
    cc.ComputeEquality(cache_, contacts_, robot_info_, NVars(), A, b);
  }

  void Release(std::size_t i) {
    contacts_.contacts[i].active = false;
    contacts_.contacts[i].activation = 0.0;
    contacts_.RecomputeActive(cfg_);
  }

  ContactManagerConfig cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
};

bool SameBytes(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  return a.rows() == b.rows() && a.cols() == b.cols() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

TEST_F(InactiveLambdaTest, TheDefaultIsRegularizeAndBothModesParse) {
  YAML::Node none;
  ContactConstraint a;
  a.Init(*model_, robot_info_, cache_, none);
  EXPECT_EQ(a.InactiveLambdaMode(), ContactConstraint::InactiveLambda::kRegularize);
  for (const auto& [text, mode] :
       {std::pair{"regularize", ContactConstraint::InactiveLambda::kRegularize},
        std::pair{"zero", ContactConstraint::InactiveLambda::kZero}}) {
    YAML::Node y;
    y["inactive_lambda"] = text;
    ContactConstraint c;
    c.Init(*model_, robot_info_, cache_, y);
    EXPECT_EQ(c.InactiveLambdaMode(), mode) << text;
  }
}

// An unknown mode is refused. An unknown KEY is not the constraint's to
// refuse: a factory entry may carry its caller's own keys (the locomotion
// builder refuses its own schema's).
TEST_F(InactiveLambdaTest, AnUnknownModeIsRefusedAndACallersKeyIsNot) {
  for (const char* bad : {"Zero", "cached", "", "0"}) {
    YAML::Node y;
    y["inactive_lambda"] = bad;
    ContactConstraint c;
    EXPECT_THROW(c.Init(*model_, robot_info_, cache_, y), std::invalid_argument) << bad;
  }
  ContactConstraint c;
  EXPECT_NO_THROW(c.Init(*model_, robot_info_, cache_,
                         YAML::Load("{type: contact_constraint, terrain_height: 0.0, "
                                    "inactive_lambda: zero, baumgarte: {kp: 1.0, kd: 2.0}, "
                                    "caller_tag: 3}")));
  EXPECT_EQ(c.InactiveLambdaMode(), ContactConstraint::InactiveLambda::kZero);
}

// Every contact active: both modes write the same bytes.
TEST_F(InactiveLambdaTest, WithEveryContactActiveBothModesWriteTheSameRows) {
  Eigen::MatrixXd Ar, Az;
  Eigen::VectorXd br, bz;
  Rows("regularize", Ar, br);
  Rows("zero", Az, bz);
  EXPECT_TRUE(SameBytes(Ar, Az));
  EXPECT_TRUE(SameBytes(br, bz));
  EXPECT_EQ(Ar.rightCols(cfg_.max_contact_vars).cwiseAbs().maxCoeff(), 0.0);
}

// A released slot's rows become I on its own λ columns (point slot 0 and
// surface slot 1 at their offsets), b stays 0, and the active slot's rows are
// the regularize mode's bytes; regularize leaves the released rows all zero.
TEST_F(InactiveLambdaTest, AReleasedSlotsRowsPinItsOwnForcesToZero) {
  const int nv = robot_info_.nv;
  for (const std::size_t released : {std::size_t{0}, std::size_t{1}}) {
    SetUp();
    Release(released);
    const int off = released == 0 ? 0 : 3;
    const int dim = released == 0 ? 3 : 6;
    Eigen::MatrixXd Ar, Az;
    Eigen::VectorXd br, bz;
    Rows(nullptr, Ar, br);
    Rows("zero", Az, bz);
    EXPECT_EQ(Ar.middleRows(off, dim).cwiseAbs().maxCoeff(), 0.0);
    EXPECT_EQ(br.segment(off, dim).cwiseAbs().maxCoeff(), 0.0);
    Eigen::MatrixXd expect = Eigen::MatrixXd::Zero(dim, NVars());
    expect.block(0, nv + off, dim, dim).setIdentity();
    EXPECT_TRUE(SameBytes(Az.middleRows(off, dim), expect)) << "slot " << released;
    EXPECT_EQ(bz.segment(off, dim).cwiseAbs().maxCoeff(), 0.0);
    for (int r = 0; r < Ar.rows(); ++r) {
      if (r >= off && r < off + dim) {
        continue;
      }
      EXPECT_TRUE(SameBytes(Ar.row(r), Az.row(r))) << "row " << r;
      EXPECT_EQ(br(r), bz(r)) << "row " << r;
    }
  }
}

// Two of three released (slots 1 and 2): each gets I on its own columns.
TEST_F(InactiveLambdaTest, EveryReleasedSlotGetsItsOwnBlock) {
  Release(1);
  Release(2);
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Rows("zero", A, b);
  const int nv = robot_info_.nv;
  Eigen::MatrixXd lam = Eigen::MatrixXd::Zero(12, 12);
  lam.block(3, 3, 9, 9).setIdentity();
  EXPECT_TRUE(SameBytes(A.rightCols(12), lam));
  EXPECT_EQ(A.block(3, 0, 9, nv).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_GT(A.block(0, 0, 3, nv).cwiseAbs().maxCoeff(), 0.0) << "slot 0 keeps its Jc rows";
}

// The contact state decides: a slot marked active keeps 0 = 0 rather than the
// zero rows even when the cache has no frame for it.
TEST_F(InactiveLambdaTest, ASlotMarkedActiveWithoutACacheFrameIsNotPinned) {
  ContactManagerConfig two = cfg_;
  two.contacts.resize(2);  // the cache below holds frames for slots 0 and 1 only
  PinocchioCache cache;
  cache.Init(model_, rtc::tsid::ContactFrameIds(two));
  cache.Update(cache_.q, cache_.v);
  ContactConstraint cc;
  cc.Init(*model_, robot_info_, cache, YAML::Load("{inactive_lambda: zero}"));
  cc.SetContactManager(&cfg_);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, NVars());
  Eigen::VectorXd b = Eigen::VectorXd::Zero(12);
  cc.ComputeEquality(cache, contacts_, robot_info_, NVars(), A, b);
  EXPECT_EQ(A.middleRows(9, 3).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_GT(A.middleRows(0, 3).cwiseAbs().maxCoeff(), 0.0) << "premise: slots with frames solve";
}

// A contact ramping in (active, activation < 1) is active: it keeps its Jc
// rows, not the zero rows.
TEST_F(InactiveLambdaTest, ARampingContactIsActive) {
  contacts_.contacts[1].activation = 0.4;
  contacts_.contacts[1].activation_target = 1.0;
  Eigen::MatrixXd Ar, Az;
  Eigen::VectorXd br, bz;
  Rows("regularize", Ar, br);
  Rows("zero", Az, bz);
  EXPECT_TRUE(SameBytes(Ar, Az));
  EXPECT_EQ(Az.rightCols(cfg_.max_contact_vars).cwiseAbs().maxCoeff(), 0.0);
}

// One constraint over a release and a touchdown: the rows switch on the tick
// the flag does, both ways.
TEST_F(InactiveLambdaTest, TheRowsSwitchOnTheTickTheContactDoes) {
  ContactConstraint cc;
  cc.Init(*model_, robot_info_, cache_, YAML::Load("{inactive_lambda: zero}"));
  cc.SetContactManager(&cfg_);
  const int nv = robot_info_.nv;
  Eigen::MatrixXd A(12, NVars());
  Eigen::VectorXd b(12);
  const auto tick = [&] {
    A.setZero();
    b.setZero();
    cc.ComputeEquality(cache_, contacts_, robot_info_, NVars(), A, b);
  };
  tick();
  const Eigen::MatrixXd stance = A.block(3, 0, 6, nv);
  EXPECT_EQ(A.block(3, nv + 3, 6, 6).cwiseAbs().maxCoeff(), 0.0);
  Release(1);
  tick();
  EXPECT_TRUE(SameBytes(A.block(3, nv + 3, 6, 6), Eigen::MatrixXd::Identity(6, 6)));
  EXPECT_EQ(A.block(3, 0, 6, nv).cwiseAbs().maxCoeff(), 0.0);
  contacts_.contacts[1].active = true;
  contacts_.contacts[1].activation = 1.0;
  contacts_.RecomputeActive(cfg_);
  tick();
  EXPECT_EQ(A.block(3, nv + 3, 6, 6).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_TRUE(SameBytes(A.block(3, 0, 6, nv), stance));
}

}  // namespace
}  // namespace rtc::tsid
