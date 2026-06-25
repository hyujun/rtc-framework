// ── SE(3) pose/twist-error comparison experiment (S1–S5) ─────────────────────
// Pose-level CLIK simulation (dt = 1e-3): integrates the EE pose directly under
// each error definition's velocity law (no robot Jacobian) to expose the
// geometric differences between the definitions. NOT a ROS node — a plain
// executable that writes CSVs + prints summary metrics, mirroring the package's
// other examples/.
//
// Usage: se3_error_compare [output_dir]   (default ".")
// CSV schema: t,type,px,py,pz,theta_err,pos_err_norm,e1,e2,e3,e4,e5,e6
#include "rtc_math/se3/pose_error.hpp"
#include "rtc_math/se3/velocity_error.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace se3 = rtc::math::se3;
using se3::ErrorType;
using se3::Iso3;
using se3::Mat3;
using se3::Mat6;
using se3::Vec3;
using se3::Vec6;

namespace {

constexpr double kDt = 1e-3;
constexpr double kPi = M_PI;

const char* TypeName(ErrorType t) {
  switch (t) {
    case ErrorType::SplitWorld:
      return "SplitWorld";
    case ErrorType::BodyLog6:
      return "BodyLog6";
    case ErrorType::SpatialLog6:
      return "SpatialLog6";
    case ErrorType::SplitLee:
      return "SplitLee";
    case ErrorType::SplitQuat:
      return "SplitQuat";
    case ErrorType::SplitBodyRot:
      return "SplitBodyRot";
  }
  return "?";
}

Iso3 MakePose(const Vec3& axis, double angle, const Vec3& p) {
  Iso3 T = Iso3::Identity();
  T.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  T.translation() = p;
  return T;
}

double AngleOf(const Mat3& R) {
  return se3::log3(R).norm();
}

// Advance a pose by a twist for one step, per the twist's frame semantics.
Iso3 StepBody(const Iso3& T, const Vec6& nu_body) {
  return T * se3::exp6(nu_body * kDt);
}

Iso3 StepSpatial(const Iso3& T, const Vec6& nu_spatial) {
  return se3::exp6(nu_spatial * kDt) * T;
}

Iso3 StepLwa(const Iso3& T, const Vec6& nu_lwa) {
  // ṗ = v_world, Ṙ = [ω_world]× R.
  Iso3 Tn = T;
  Tn.translation() = T.translation() + kDt * nu_lwa.head<3>();
  Tn.linear() = se3::exp3(kDt * nu_lwa.tail<3>()) * T.linear();
  return Tn;
}

void WriteRow(std::ofstream& f, double t, ErrorType type, const Iso3& T, const Vec6& e) {
  f << t << ',' << TypeName(type) << ',' << T.translation().x() << ',' << T.translation().y() << ','
    << T.translation().z() << ',' << e.tail<3>().norm() << ',' << e.head<3>().norm();
  for (int i = 0; i < 6; ++i)
    f << ',' << e(i);
  f << '\n';
}

// Max perpendicular distance of a 3D path from the straight segment p0→p1.
double MaxLineDeviation(const std::vector<Vec3>& path, const Vec3& p0, const Vec3& p1) {
  const Vec3 d = p1 - p0;
  const double dn = d.norm();
  if (dn < 1e-12)
    return 0.0;
  const Vec3 u = d / dn;
  double mx = 0.0;
  for (const Vec3& p : path) {
    const Vec3 w = p - p0;
    mx = std::max(mx, (w - u * w.dot(u)).norm());
  }
  return mx;
}

// ── S1: large rotation + translation regulation ──────────────────────────────
void RunS1(const std::string& dir) {
  const Vec3 p_d(0.4, -0.2, 0.3);
  const Iso3 T_d = MakePose(Vec3(0.3, 0.5, 0.8), 150.0 * kPi / 180.0, p_d);
  const Iso3 T0 = Iso3::Identity();
  const double k = 2.0;
  std::ofstream f(dir + "/s1_regulation.csv");
  f << "t,type,px,py,pz,theta_err,pos_err_norm,e1,e2,e3,e4,e5,e6\n";
  std::printf("\n[S1] large rot(150°)+trans regulation — straight-line deviation:\n");
  for (ErrorType type : {ErrorType::SplitWorld, ErrorType::BodyLog6, ErrorType::SpatialLog6}) {
    Iso3 T = T0;
    std::vector<Vec3> path;
    for (int s = 0; s < 4000; ++s) {
      const double t = s * kDt;
      const Vec6 e = se3::computePoseError(T, T_d, type);
      WriteRow(f, t, type, T, e);
      path.push_back(T.translation());
      switch (type) {
        case ErrorType::SplitWorld:
          T = StepLwa(T, k * e);
          break;
        case ErrorType::BodyLog6:
          T = StepBody(T, k * e);
          break;
        case ErrorType::SpatialLog6:
          T = StepSpatial(T, k * e);
          break;
        default:
          break;
      }
    }
    std::printf("  %-12s max ⟂ deviation from p0→p_d line = %.5f m\n", TypeName(type),
                MaxLineDeviation(path, T0.translation(), p_d));
  }
}

// ── S2: pure-rotation regulation, scale comparison + Lee stall ───────────────
void RunS2(const std::string& dir) {
  std::ofstream f(dir + "/s2_rotation.csv");
  f << "t,type,px,py,pz,theta_err,pos_err_norm,e1,e2,e3,e4,e5,e6\n";
  const double k = 2.0;
  std::printf("\n[S2] pure-rotation regulation — time to θ < 1°:\n");
  std::printf("  %-12s | %8s %8s %8s %8s\n", "type", "30°", "90°", "150°", "179°");
  for (ErrorType type : {ErrorType::SplitWorld, ErrorType::SplitLee, ErrorType::SplitQuat}) {
    std::printf("  %-12s |", TypeName(type));
    for (double deg : {30.0, 90.0, 150.0, 179.0}) {
      const Iso3 T_d = MakePose(Vec3(0.2, 0.3, 0.9), deg * kPi / 180.0, Vec3::Zero());
      Iso3 T = Iso3::Identity();
      double t_reach = -1.0;
      for (int s = 0; s < 8000; ++s) {
        const double t = s * kDt;
        const Vec6 e = se3::computePoseError(T, T_d, type);
        if (deg == 150.0)
          WriteRow(f, t, type, T, e);  // dump one curve per type
        if (t_reach < 0.0 && AngleOf(T.linear().transpose() * T_d.linear()) < kPi / 180.0)
          t_reach = t;
        T = StepLwa(T, k * e);
      }
      std::printf(" %8.3f", t_reach);
    }
    std::printf("  (s)\n");
  }
  std::printf("  → SplitLee stalls as θ→π (rotation scale sinθ→0).\n");
}

// ── S3: θ ≈ π robustness (no NaN / divergence) ───────────────────────────────
void RunS3() {
  const Iso3 T_d = MakePose(Vec3(0.1, 0.2, 0.97), 179.999 * kPi / 180.0, Vec3(0.1, 0.0, 0.0));
  const Iso3 T = Iso3::Identity();
  std::printf("\n[S3] θ₀ = 179.999° — all definitions finite:\n");
  for (ErrorType type : {ErrorType::SplitWorld, ErrorType::BodyLog6, ErrorType::SpatialLog6,
                         ErrorType::SplitLee, ErrorType::SplitQuat, ErrorType::SplitBodyRot}) {
    const Vec6 e = se3::computePoseError(T, T_d, type);
    std::printf("  %-12s finite=%d  ‖e‖=%.5f\n", TypeName(type), static_cast<int>(e.allFinite()),
                e.norm());
  }
}

// ── S4: gain isotropy × Jlog compensation (BodyLog6 regulation) ──────────────
void RunS4(const std::string& dir) {
  const Iso3 T_d = MakePose(Vec3(0.3, 0.5, 0.8), 120.0 * kPi / 180.0, Vec3(0.2, 0.1, -0.15));
  const double k = 2.0;
  Mat6 K = Mat6::Identity();
  K.diagonal() << 1, 1, 1, 8, 8, 1;  // anisotropic (heavy on two rotation axes)
  K *= k;
  std::ofstream f(dir + "/s4_gain.csv");
  f << "t,law,e1,e2,e3,e4,e5,e6\n";
  // law: scalar | aniso_naive | aniso_jlog
  const char* laws[] = {"scalar", "aniso_naive", "aniso_jlog"};
  std::printf("\n[S4] BodyLog6 anisotropic gain — max deviation from ideal per-component\n");
  std::printf("     decay e_i(t)=e_i(0)·exp(−K_ii·t)  (K_rot=8k on axes 4,5):\n");
  const Vec6 e0 = se3::log6(Iso3::Identity().inverse() * T_d);
  for (int law = 0; law < 3; ++law) {
    Iso3 T = Iso3::Identity();
    double max_dev = 0.0;
    for (int s = 0; s < 2000; ++s) {
      const double t = s * kDt;
      const Iso3 F = T.inverse() * T_d;
      const Vec6 e = se3::log6(F);
      f << t << ',' << laws[law];
      for (int i = 0; i < 6; ++i)
        f << ',' << e(i);
      f << '\n';
      // Deviation of each component from THIS law's target exponential
      // (scalar targets k on every axis; aniso laws target K_ii).
      for (int i = 0; i < 6; ++i) {
        const double gain = (law == 0) ? k : K(i, i);
        const double ideal = e0(i) * std::exp(-gain * t);
        max_dev = std::max(max_dev, std::abs(e(i) - ideal));
      }
      Vec6 nu;
      if (law == 0) {
        nu = k * e;  // scalar: ė = −k e exactly (isotropic — matches ideal trivially)
      } else if (law == 1) {
        nu = se3::adjoint(F) * (K * e);  // naive aniso: Jlog coupling left in
      } else {
        nu = se3::adjoint(F) * (se3::Jlog6(F).inverse() * (K * e));  // Jlog-compensated
      }
      T = StepBody(T, nu);
    }
    std::printf("  %-12s max |e_i − ideal_i| = %.3e\n", laws[law], max_dev);
  }
  std::printf("  → aniso_jlog ≈ scalar (clean decay); aniso_naive deviates (axes couple).\n");
}

// ── S5: transport map presence (BodyLog6 tracking) ★ ─────────────────────────
//
// Faithful framing: for BodyLog6 with EXACT feedforward, the correct transport
// ν = Ad_{T⁻¹T_d}·ν_d + k·e makes ė = −k·e hold EXACTLY even against a moving
// target (Modern Robotics §11.3) ⇒ ‖e(t)‖ = ‖e₀‖·e^{−kt} to integration error.
// Omitting the transport (ν = ν_d + k·e — a very common bug) injects a
// rotation-coupled feedforward mismatch (I − Ad_{T_d⁻¹T})·ν_d, breaking that
// guarantee: the error no longer follows the clean exponential. We start from a
// non-zero offset against a spinning target and measure the deviation of the
// error VECTOR e(t) from the guaranteed e₀·e^{−kt} (the omission mostly ROTATES
// e — it barely changes ‖e‖, so a norm-only metric would miss the bug).
void RunS5(const std::string& dir) {
  const double radius = 0.1;
  const double freq = 0.5;  // Hz
  const double k = 3.0;
  const double T_total = 3.0;
  const int steps = static_cast<int>(T_total / kDt);
  auto Desired = [&](double t) {
    const double ang = 2.0 * kPi * freq * t;
    const Vec3 p(radius * std::cos(ang), radius * std::sin(ang), 0.2);
    return MakePose(Vec3(0, 0, 1), ang, p);  // continuous spin (angle = ang)
  };
  // Sizeable initial pose offset from the target at t=0.
  Vec6 off;
  off << 0.15, -0.10, 0.12, 0.5, -0.3, 0.4;
  const Iso3 T0 = Desired(0.0) * se3::exp6(off);
  const Vec6 e_init = se3::log6(T0.inverse() * Desired(0.0));
  std::ofstream f(dir + "/s5_tracking.csv");
  f << "t,law,pos_err_norm,theta_err,err_norm,vec_dev\n";
  std::printf("\n[S5] BodyLog6 spinning-target tracking from offset — deviation of error\n");
  std::printf("     VECTOR e(t) from the guaranteed e₀·e^{−kt}  (k=%.1f, ‖e₀‖=%.3f):\n", k,
              e_init.norm());
  for (int law = 0; law < 2; ++law) {  // 0: with transport, 1: omitted (common bug)
    Iso3 T = T0;
    double max_dev = 0.0;
    for (int s = 0; s < steps; ++s) {
      const double t = s * kDt;
      const Iso3 T_d = Desired(t);
      const Iso3 F = T.inverse() * T_d;
      const Vec6 e = se3::log6(F);
      const Vec6 nu_d = se3::log6(T_d.inverse() * Desired(t + kDt)) / kDt;  // body twist
      const Vec6 nu = (law == 0) ? Vec6(se3::adjoint(F) * nu_d + k * e)     // correct transport
                                 : Vec6(nu_d + k * e);                      // transport omitted
      const Vec6 ideal_vec = e_init * std::exp(-k * t);
      max_dev = std::max(max_dev, (e - ideal_vec).norm());
      f << t << ',' << (law == 0 ? "transport" : "no_transport") << ',' << e.head<3>().norm() << ','
        << e.tail<3>().norm() << ',' << e.norm() << ',' << (e - ideal_vec).norm() << '\n';
      T = StepBody(T, nu);
    }
    std::printf("  %-13s max ‖e(t) − e₀·e^{−kt}‖ = %.3e\n", law == 0 ? "transport" : "no_transport",
                max_dev);
  }
  std::printf("  → transport preserves exact exponential convergence; omission breaks it.\n");
}

}  // namespace

int main(int argc, char** argv) {
  const std::string dir = (argc > 1) ? argv[1] : ".";
  std::printf("SE(3) error comparison experiment (output dir: %s)\n", dir.c_str());
  RunS1(dir);
  RunS2(dir);
  RunS3();
  RunS4(dir);
  RunS5(dir);
  std::printf("\nCSVs written to %s/ (plot with scripts/plot_se3_compare.py)\n", dir.c_str());
  return 0;
}
