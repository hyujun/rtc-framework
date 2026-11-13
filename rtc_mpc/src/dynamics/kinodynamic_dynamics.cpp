/// @file kinodynamic_dynamics.cpp
/// @brief Configure-time checks and in-place mode switching for the
///        kinodynamic ODE (see the header for the dynamics and conventions).

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"

#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "rtc_urdf_bridge/free_flyer_dims.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/modelling/dynamics/integrator-explicit.hpp>
#include <aligator/modelling/dynamics/integrator-rk2.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#pragma GCC diagnostic pop

#include <cmath>
#include <cstddef>
#include <utility>

namespace rtc::mpc {

// A mode is a 32-bit mask; every contact a layout can hold must have a bit,
// with `2^n_contacts` still representable.
static_assert(static_cast<std::size_t>(kMaxContacts) < kMaxContactBits,
              "a contact mode must name every contact of a layout");

KinodynamicError ParseKinodynamicIntegrator(std::string_view name,
                                            KinodynamicIntegrator& out) noexcept {
  if (name == "semi_implicit_euler") {
    out = KinodynamicIntegrator::kSemiImplicitEuler;
    return KinodynamicError::kNoError;
  }
  if (name == "rk2") {
    out = KinodynamicIntegrator::kRk2;
    return KinodynamicError::kNoError;
  }
  return KinodynamicError::kUnknownIntegrator;
}

KinodynamicError KinodynamicDynamics::Init(const RobotModelHandler& handler) noexcept {
  model_ = nullptr;
  space_.reset();
  contact_ids_.clear();
  force_size_ = 0;
  nu_ = 0;
  n_modes_ = 0;

  if (!handler.Initialised()) {
    return KinodynamicError::kModelNotInitialised;
  }
  if (!handler.floating_base()) {
    return KinodynamicError::kFixedBase;
  }
  const std::vector<ContactFrameInfo>& contacts = handler.contact_frames();
  if (contacts.empty()) {
    return KinodynamicError::kNoContacts;
  }
  if (static_cast<int>(contacts.size()) > kMaxContacts) {
    return KinodynamicError::kLayoutMismatch;
  }
  const int dim = contacts.front().dim;
  for (const ContactFrameInfo& c : contacts) {
    if (c.dim != dim) {
      return KinodynamicError::kMixedContactDim;
    }
  }
  if (dim != 3 && dim != 6) {
    return KinodynamicError::kUnsupportedContactDim;
  }

  const pinocchio::Model& model = handler.model();
  const int nc = static_cast<int>(contacts.size());
  const int nu = nc * dim + model.nv - handler.n_base_v();
  const WbLayout& layout = handler.layout();
  if (handler.n_base_v() != rtc_urdf_bridge::kFreeFlyerNv || handler.nu() != nu ||
      layout.n_contacts != nc || layout.n_wrench_u != nc * dim || layout.acc_u_offset != nc * dim) {
    return KinodynamicError::kLayoutMismatch;
  }
  for (int k = 0; k < nc; ++k) {
    if (layout.contact_u_offset[static_cast<std::size_t>(k)] != k * dim ||
        layout.contact_dim[static_cast<std::size_t>(k)] != dim) {
      return KinodynamicError::kLayoutMismatch;
    }
  }

  try {
    space_.emplace(model);
    contact_ids_.reserve(contacts.size());
    for (const ContactFrameInfo& c : contacts) {
      contact_ids_.push_back(static_cast<pinocchio::FrameIndex>(c.frame_id));
    }
  } catch (...) {
    space_.reset();
    contact_ids_.clear();
    return KinodynamicError::kAligatorFailure;
  }
  force_size_ = dim;
  nu_ = nu;
  n_modes_ = std::uint32_t{1} << static_cast<unsigned>(nc);
  model_ = &model;
  return KinodynamicError::kNoError;
}

void KinodynamicDynamics::FillMask(std::uint32_t mode, std::vector<bool>& states) const noexcept {
  for (std::size_t k = 0; k < states.size(); ++k) {
    states[k] = ((mode >> k) & 1U) != 0U;
  }
}

std::optional<KinodynamicDynamics::Ode> KinodynamicDynamics::MakeOde(
    std::uint32_t mode) const noexcept {
  if (!Initialised() || mode >= n_modes_) {
    return std::nullopt;
  }
  try {
    std::vector<bool> states(contact_ids_.size(), false);
    FillMask(mode, states);
    return Ode(*space_, *model_, model_->gravity.linear(), states, contact_ids_, force_size_);
  } catch (...) {
    return std::nullopt;
  }
}

KinodynamicError KinodynamicDynamics::MakeStageDynamics(
    std::uint32_t mode, double dt, KinodynamicIntegrator integrator,
    std::optional<StageDynamics>& out) const noexcept {
  using SemiImplicitEuler = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
  using Rk2 = aligator::dynamics::IntegratorRK2Tpl<double>;
  using PolyOde = xyz::polymorphic<aligator::dynamics::ODEAbstractTpl<double>>;
  out.reset();
  if (!Initialised()) {
    return KinodynamicError::kNotInitialised;
  }
  if (mode >= n_modes_) {
    return KinodynamicError::kInvalidMode;
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    return KinodynamicError::kInvalidDt;
  }
  if (integrator != KinodynamicIntegrator::kSemiImplicitEuler &&
      integrator != KinodynamicIntegrator::kRk2) {
    return KinodynamicError::kUnknownIntegrator;
  }
  std::optional<Ode> ode = MakeOde(mode);
  if (!ode.has_value()) {
    return KinodynamicError::kAligatorFailure;
  }
  try {
    const PolyOde poly(std::move(*ode));
    if (integrator == KinodynamicIntegrator::kSemiImplicitEuler) {
      out.emplace(SemiImplicitEuler(poly, dt));
    } else {
      out.emplace(Rk2(poly, dt));
    }
  } catch (...) {
    out.reset();
    return KinodynamicError::kAligatorFailure;
  }
  return KinodynamicError::kNoError;
}

KinodynamicDynamics::Ode* KinodynamicDynamics::OdeOf(
    aligator::StageModelTpl<double>& stage) noexcept {
  using Integrator = aligator::dynamics::ExplicitIntegratorAbstractTpl<double>;
  if (stage.dynamics_.valueless_after_move()) {
    return nullptr;
  }
  auto* integrator = stage.getDynamics<Integrator>();
  if (integrator == nullptr || integrator->ode_.valueless_after_move()) {
    return nullptr;
  }
  return integrator->getDynamics<Ode>();
}

bool KinodynamicDynamics::SetMode(Ode& ode, std::uint32_t mode) const noexcept {
  if (!Initialised() || mode >= n_modes_ || ode.contact_states_.size() != contact_ids_.size() ||
      ode.contact_ids_ != contact_ids_ || ode.force_size_ != force_size_) {
    return false;
  }
  FillMask(mode, ode.contact_states_);
  return true;
}

}  // namespace rtc::mpc
