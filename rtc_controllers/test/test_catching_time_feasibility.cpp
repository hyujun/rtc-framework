// L3 reachability gates — G3-A, G3-B (docs/dynamic_catching/L3_planner.md §9)
// plus the S1.5 fixes (invalid-limit flag, directional-speed projection and
// zero guards) and the error-budget / stopping-distance formulas.
//
// Ported from docs/dynamic_catching/test_l3.cpp with thresholds unchanged.
// The reference also wrote cases.txt for verify_l3.py; that round trip only
// compared the C++ closed form with its own output. G3-A's "fixed table vs the
// script" is now a literal table (below) produced by verify_l3.py's
// independently written Python closed form `t_min`, whose agreement with the
// velocity/acceleration LP + bisection is ≤ 1.2e-5 s on these rows (LP grid
// resolution) — S1 sub-plan F-5. Generator: numpy default_rng(20260919),
// q0, q1 ~ U(−2, 2), a ~ U(5, 20), w0 ~ U(−π, π), w_max = π, 40 rows, plus two
// |w0| > w_max rows; values printed with %.17g.
//
// Include order: the Eigen allocation tripwire must precede every Eigen header.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/soft_catch.hpp"
#include "rtc_controllers/catching/time_feasibility.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/catching_ball_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {

using rtc::catching::BallTime;
using rtc::catching::CatchErrorSigma;
using rtc::catching::ComputeGammaWindow;
using rtc::catching::DirectionalSpeed;
using rtc::catching::DirectionalSpeedMax;
using rtc::catching::ErrorBudgetOk;
using rtc::catching::GammaProfile;
using rtc::catching::GammaWindow;
using rtc::catching::MakeNowLead;
using rtc::catching::MaxCatchableSpeed;
using rtc::catching::MaxJointTMin;
using rtc::catching::NowReal;
using rtc::catching::PlanningTcpSpeed;
using rtc::catching::ReachTimeFeasible;
using rtc::catching::SoftCatchTranslation;
using rtc::catching::StoppingPoint;
using rtc::catching::StoppingReservation;
using rtc::catching::TargetState;
using rtc::catching::TMin;
using rtc::catching::TMinChecked;
using rtc::catching::TMinResult;
using rtc::catching::TRest;
namespace fx = rtc::catching::fixture;

constexpr double kPi = 3.14159265358979323846;

// {q0, w0, q1, w_max, a_max, t_min from verify_l3.py}
constexpr std::array<std::array<double, 6>, 42> kTMinTable{{
    {-1.1714685612925249, 1.7783477058907851, 1.4605408346209012, 3.1415926535897931,
     16.31626680993239, 0.9521942928466931},
    {1.9453924172830348, 2.3649671402030474, 0.30808751474680918, 3.1415926535897931,
     5.9120463715707094, 1.6031517800721449},
    {-0.37684342359416156, 1.897707861412167, -0.41666210344058641, 3.1415926535897931,
     6.6791214672703205, 0.71459152410608306},
    {-0.30239086863756581, -0.38978231559464938, 1.7383779565045323, 3.1415926535897931,
     5.5291359456320164, 1.2926549498120272},
    {0.67950981082800377, -0.27614671947647507, 0.20555550085176399, 3.1415926535897931,
     18.990233532033642, 0.30208773685595719},
    {1.049961934912599, 2.330247546240769, -0.9825842235533373, 3.1415926535897931,
     10.39673348280953, 1.2564074962387231},
    {-1.5077648099003049, 2.3366192364956033, -1.5070534271477274, 3.1415926535897931,
     8.4085145317452472, 0.67044844861401742},
    {-0.66480064577272646, 2.2887065550384502, 0.78026196483533461, 3.1415926535897931,
     19.260557569889073, 0.54754361061524415},
    {0.59074517726733999, 0.1664760768346083, 0.5187178838902895, 3.1415926535897931,
     7.8426566983513091, 0.21523045802997581},
    {0.31831084129993892, 0.81365916713989295, 0.97866660861439048, 3.1415926535897931,
     16.995857071165908, 0.35336799068812824},
    {-0.44365744319494116, 1.6693925494297011, 1.5157237593789579, 3.1415926535897931,
     6.5160397590976675, 0.9176948255169961},
    {1.0815812637491717, -1.3715159524686884, -0.62781944137764567, 3.1415926535897931,
     15.539816346997302, 0.67729034854384762},
    {0.94561824907188319, 2.727173985755627, -1.5343062404458578, 3.1415926535897931,
     8.3591150093964917, 1.6330716439896187},
    {0.49754745398182232, -0.98382484381725455, 0.9751976929749393, 3.1415926535897931,
     5.2703902878685955, 0.84409515331582852},
    {-0.96264419449097982, 2.6175768995298503, 0.69988528131929506, 3.1415926535897931,
     12.846755258708315, 0.65487325388576001},
    {0.40804018434035072, 0.17542936160289058, 1.3625598247780593, 3.1415926535897931,
     16.849209678250244, 0.48016542493507341},
    {-1.4398288988195858, 1.6293079454179695, 1.9532069593200623, 3.1415926535897931,
     19.181110144676538, 1.1809061228939952},
    {1.0607320836529412, 1.4164126578604908, -0.93190185656931934, 3.1415926535897931,
     16.2129839102532, 0.93510224291779775},
    {1.4061733380866546, -1.2394724328071109, -1.8166739647698948, 3.1415926535897931,
     17.130105171088765, 1.1511773801817244},
    {-0.51549053842490267, -1.5071697594226432, 0.59371240774912248, 3.1415926535897931,
     13.502720431373364, 0.72412826996007607},
    {1.3521125700873262, 2.8433703656461198, -1.7903566075957982, 3.1415926535897931,
     8.0175781608100074, 1.9072476196349646},
    {-0.16720316493098775, -1.2961637540756863, -1.3049497482218424, 3.1415926535897931,
     19.076978226916154, 0.4729080983178594},
    {-0.75332435023762656, -3.0327954808921223, 0.092646603095241087, 3.1415926535897931,
     10.548146832124335, 0.99341486284828395},
    {0.89964423539185479, 3.0478801380479688, 1.9795609208002367, 3.1415926535897931,
     16.612847750429651, 0.43838539550629985},
    {-1.3345292188782212, 2.7992953454353868, 1.2479785321298968, 3.1415926535897931,
     19.927653643649563, 0.90179847344345065},
    {-0.042752612219354891, 2.4077532963395312, 0.91867071315598192, 3.1415926535897931,
     14.254584674875073, 0.42223908966500545},
    {-1.0405867313046255, -1.6294161714322015, 1.0873818763967114, 3.1415926535897931,
     8.1937953911718431, 1.3111945998804426},
    {0.074357605642976665, 0.70099676113141784, 0.33812244943497438, 3.1415926535897931,
     7.6323758060278024, 0.30198956841056934},
    {1.1824550312555684, -2.6733463280382459, -1.9661132003659927, 3.1415926535897931,
     15.728154785400521, 1.1043106791154196},
    {1.9066181720512203, 0.10518877442655628, 0.83965585417478295, 3.1415926535897931,
     5.7042059817977977, 0.88381534356951164},
    {0.72896336930858707, -1.6263774512229523, -1.7295519088365783, 3.1415926535897931,
     16.825912624713098, 0.89764202146842753},
    {-0.30905780236648983, -2.407242657762215, -0.9398256707130308, 3.1415926535897931,
     5.395408472504359, 0.4842969764050471},
    {-1.8342067506947712, 0.76585387909781844, -0.28620043295854192, 3.1415926535897931,
     10.550432671870167, 0.72677291961677271},
    {-1.1506795831575571, 0.25443299061676639, 0.34849773181252086, 3.1415926535897931,
     10.633394730387117, 0.74969004615403501},
    {1.1617817086922009, -2.6314430119283019, -0.21211536464044434, 3.1415926535897931,
     18.411782183033893, 0.52488942993741183},
    {1.1967618180992976, -2.191722898695696, 0.6799122376329314, 3.1415926535897931,
     17.293874284011775, 0.26365136818878587},
    {-0.7765980188492434, 1.3604476834145798, 1.7740182400746449, 3.1415926535897931,
     11.753291332560293, 0.98849322347908863},
    {-1.4383401890775787, -0.52669309544257636, 1.3756495286935486, 3.1415926535897931,
     17.423704761863746, 1.1087888786744922},
    {-1.0628053785289762, -0.060070187856366974, 1.6424596584571893, 3.1415926535897931,
     9.5188721093954349, 1.1975219099494503},
    {0.60455453585914576, -0.44607336351309268, 1.7280707321082591, 3.1415926535897931,
     5.6416734419635892, 0.97856019768066338},
    {0, 6, 2, 3.1415926535897931, 10, 0.79369940504707104},
    {0.29999999999999999, -4.5, -1, 3.1415926535897931, 12, 0.54470254593850265},
}};

// ── G3-A ────────────────────────────────────────────────────────────────────

TEST(CatchingTimeFeasibility, G3ATMinMatchesScriptTable) {
  double worst = 0.0;
  for (const auto& r : kTMinTable) {
    const TMinResult c = TMinChecked(r[0], r[1], r[2], r[3], r[4]);
    ASSERT_FALSE(c.limits_invalid || c.input_invalid);
    EXPECT_EQ(c.w0_clamped, std::abs(r[1]) > r[3]);
    worst = std::max(worst, std::abs(c.t - r[5]));
  }
  EXPECT_LT(worst, 1e-9);
}

// Every branch against a hand-checkable closed form. w0 = 0 alone would leave
// the reverse / overshoot branches and TRest() unasserted.
TEST(CatchingTimeFeasibility, G3ATMinClosedFormAllBranches) {
  const double a = 8.0;
  EXPECT_LT(std::abs(TMin(0.0, 0.0, 1.0, 100.0, a) - 2.0 * std::sqrt(1.0 / a)), 1e-12);  // triangle
  EXPECT_LT(std::abs(TMin(0.0, 0.0, 4.0, 2.0, a) - (4.0 / 2.0 + 2.0 / a)), 1e-12);  // trapezoid
  const double w0 = -4.0;
  const double D3 = 1.0;  // reverse: brake |w0|/a, then rest-to-rest D + w0²/2a
  EXPECT_LT(std::abs(TMin(0.0, w0, D3, 100.0, a) -
                     (4.0 / a + 2.0 * std::sqrt((D3 + 16.0 / (2 * a)) / a))),
            1e-12);
  const double w4 = 5.0;
  const double D4 = 0.5;
  const double ds4 = w4 * w4 / (2 * a);  // overshoot: 1.5625 > 0.5
  EXPECT_LT(std::abs(TMin(0.0, w4, D4, 100.0, a) - (w4 / a + 2.0 * std::sqrt((ds4 - D4) / a))),
            1e-12);
  EXPECT_LT(std::abs(TRest(4.0, 2.0, a) - (4.0 / 2.0 + 2.0 / a)), 1e-12);  // TRest trapezoid
}

// Regression C9: |w0| > w_max is clamped and flagged, never a negative segment.
TEST(CatchingTimeFeasibility, G3AOverspeedInitialStateFlagged) {
  const TMinResult r = TMinChecked(0.0, 6.0, 2.0, kPi, 10.0);
  const TMinResult ok = TMinChecked(0.0, 3.0, 2.0, kPi, 10.0);
  EXPECT_TRUE(r.w0_clamped);
  EXPECT_FALSE(r.Usable());
  EXPECT_GT(r.t, 0.0);
  EXPECT_FALSE(ok.w0_clamped);
  EXPECT_TRUE(ok.Usable());
}

// S1.5 fix: invalid limits are a distinguishable flag with t = +∞ — the
// reference returned an unflagged 0, a reach-time gate that always passed.
TEST(CatchingTimeFeasibility, G3AInvalidLimitsFlaggedAndNeverPass) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto& [w_max, a_max] : std::array<std::array<double, 2>, 7>{{{kPi, 0.0},
                                                                          {kPi, -1.0},
                                                                          {0.0, 10.0},
                                                                          {-kPi, 10.0},
                                                                          {nan, 10.0},
                                                                          {kPi, nan},
                                                                          {inf, 10.0}}}) {
    const TMinResult r = TMinChecked(0.0, 0.0, 1.0, w_max, a_max);
    EXPECT_TRUE(r.limits_invalid) << w_max << " " << a_max;
    EXPECT_FALSE(r.Usable());
    EXPECT_EQ(r.t, inf);
    EXPECT_EQ(TMin(0.0, 0.0, 1.0, w_max, a_max), inf);
  }
  const TMinResult bad_q = TMinChecked(nan, 0.0, 1.0, kPi, 10.0);
  EXPECT_TRUE(bad_q.input_invalid);
  EXPECT_EQ(bad_q.t, inf);
  // A flagged candidate fails the reach-time gate however far away t_k is.
  const BallTime far{1'000'000'000'000};
  EXPECT_FALSE(
      ReachTimeFeasible(far, MakeNowLead(NowReal{0}, 0), 0, TMinChecked(0.0, 0.0, 1.0, kPi, 0.0)));
}

TEST(CatchingTimeFeasibility, MaxJointTMinTakesWorstJointAndOrsFlags) {
  const std::array<double, 3> q0{0.0, 0.0, 0.0};
  const std::array<double, 3> w0{0.0, 0.0, 0.0};
  const std::array<double, 3> q1{0.5, 2.0, -1.0};
  const std::array<double, 3> wm{kPi, kPi, kPi};
  const std::array<double, 3> am{10.0, 10.0, 10.0};
  const TMinResult r = MaxJointTMin(q0, w0, q1, wm, am);
  EXPECT_DOUBLE_EQ(r.t, TMin(0.0, 0.0, 2.0, kPi, 10.0));
  EXPECT_TRUE(r.Usable());
  const std::array<double, 3> am_bad{10.0, 0.0, 10.0};
  EXPECT_TRUE(MaxJointTMin(q0, w0, q1, wm, am_bad).limits_invalid);
  const std::array<double, 2> short_span{0.0, 0.0};
  EXPECT_TRUE(MaxJointTMin(short_span, w0, q1, wm, am).input_invalid);
}

// Reach-time gate is on the lead axis (L3 §4.3, plan §3): with T_arm ≠ 0 the
// same t_min that passes on the real axis fails once T_arm is subtracted.
TEST(CatchingTimeFeasibility, ReachTimeGateOnLeadAxis) {
  constexpr std::int64_t kMs = 1'000'000;
  const TMinResult joints = TMinChecked(0.0, 0.0, 1.0, kPi, 10.0);  // ≈ 0.632 s
  const BallTime t_k{2'000 * kMs};
  const NowReal now{t_k.ns - 700 * kMs};
  EXPECT_TRUE(ReachTimeFeasible(t_k, MakeNowLead(now, 0), 20 * kMs, joints));
  EXPECT_FALSE(ReachTimeFeasible(t_k, MakeNowLead(now, 50 * kMs), 20 * kMs, joints));
  EXPECT_FALSE(ReachTimeFeasible(t_k, MakeNowLead(now, 0), -1, joints));  // bad margin
}

// ── G3-B ────────────────────────────────────────────────────────────────────

// γ_min value pinned — a boolean alone passes even if 1 − d/(vT) became
// 0.9 − … (the clamp hides negatives).
TEST(CatchingTimeFeasibility, G3BGammaMinValueFixed) {
  const GammaWindow w1 = ComputeGammaWindow(5.0, 10.0, 10.0, 0.04, 0.02);  // 1 − 0.04/0.1 = 0.6
  const GammaWindow w2 = ComputeGammaWindow(4.0, 10.0, 10.0, 0.05, 0.05);  // 1 − 0.05/0.2 = 0.75
  EXPECT_LT(std::abs(w1.g_min - 0.6), 1e-12);
  EXPECT_LT(std::abs(w2.g_min - 0.75), 1e-12);
}

// [R1] sanity: stationary catch, 3 cm pocket, 6 m/s → T_close ≤ 5 ms. 5 ms
// exactly is the g_min = g_max = 0 knife edge, so both sides are probed.
TEST(CatchingTimeFeasibility, G3BR1PocketSanity) {
  const double v = 6.0;
  const double d = 0.03;
  EXPECT_TRUE(ComputeGammaWindow(v, 0.0, 10.0, d, 0.0049).Feasible());
  EXPECT_FALSE(ComputeGammaWindow(v, 0.0, 10.0, d, 0.0051).Feasible());
  EXPECT_LT(std::abs(d / v - 0.005), 1e-12);
}

// Non-physical input must not be clamped into a verdict: negative v_dir,max
// would be clamped UP to 0 and flip it; a zero ball speed has no window.
TEST(CatchingTimeFeasibility, G3BRejectsNonPhysicalInput) {
  const GammaWindow neg = ComputeGammaWindow(1.0, -1.0, 5.0, 0.04, 0.06);
  const GammaWindow pos = ComputeGammaWindow(1.0, 1.0, 5.0, 0.04, 0.06);
  EXPECT_TRUE(neg.input_invalid);
  EXPECT_FALSE(neg.Feasible());
  EXPECT_FALSE(pos.input_invalid);
  EXPECT_TRUE(pos.Feasible());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double v : {0.0, -1.0, nan}) {
    const GammaWindow w = ComputeGammaWindow(v, 1.0, 5.0, 0.04, 0.06);
    EXPECT_TRUE(w.input_invalid) << v;  // the reference floored 0 at 1e-6 → [0, 1] feasible
    EXPECT_FALSE(w.Feasible()) << v;
  }
  EXPECT_FALSE(ComputeGammaWindow(1.0, 1.0, 5.0, 0.04, 0.0).Feasible());    // T_close = 0
  EXPECT_FALSE(ComputeGammaWindow(1.0, 1.0, 5.0, -0.01, 0.06).Feasible());  // d_eff < 0
}

// Regression C6 + D-9: the TCP speed bound really binds g_max, and the value the
// planner passes is η_v · reference.v_max.
TEST(CatchingTimeFeasibility, G3BTcpSpeedBindsAndIsEtaVTimesVmax) {
  const GammaWindow a = ComputeGammaWindow(4.0, 3.0, 3.0, 0.04, 0.06);
  const GammaWindow b = ComputeGammaWindow(4.0, 3.0, 1.0, 0.04, 0.06);
  EXPECT_LT(std::abs(a.g_max - 0.75), 1e-12);
  EXPECT_LT(std::abs(b.g_max - 0.25), 1e-12);

  const double v_tcp = PlanningTcpSpeed(0.8, 2.5);  // η_v = 0.8, v_max = 2.5
  EXPECT_DOUBLE_EQ(v_tcp, 2.0);
  EXPECT_LT(std::abs(ComputeGammaWindow(4.0, 3.0, v_tcp, 0.04, 0.06).g_max - 0.5), 1e-12);
  EXPECT_EQ(PlanningTcpSpeed(0.0, 2.5), 0.0);   // η_v must be > 0
  EXPECT_EQ(PlanningTcpSpeed(1.01, 2.5), 0.0);  // …and ≤ 1
  EXPECT_EQ(PlanningTcpSpeed(0.8, -1.0), 0.0);
  EXPECT_DOUBLE_EQ(PlanningTcpSpeed(1.0, 2.5), 2.5);
}

TEST(CatchingTimeFeasibility, G3BMaxCatchableSpeed) {
  const double v_lim = MaxCatchableSpeed(1.5, 2.0, 0.04, 0.06);
  EXPECT_LT(std::abs(v_lim - (1.5 + 0.04 / 0.06)), 1e-12);
  EXPECT_FALSE(ComputeGammaWindow(6.0, 1.5, 2.0, 0.04, 0.06).Feasible());
  EXPECT_TRUE(ComputeGammaWindow(2.0, 1.5, 2.0, 0.04, 0.06).Feasible());
  EXPECT_EQ(MaxCatchableSpeed(1.5, 2.0, 0.04, 0.0), 0.0);  // invalid → nothing catchable
}

// §4.8 γ rollout: the reference table value (T_w = 0.30, γ_f = 0.4) → 33.0 m/s²
// within ±5 %, peak unsaturated demand in the window [t_c − T_w, t_c].
TEST(CatchingTimeFeasibility, G3BGammaRolloutTableReproducible) {
  constexpr double kDt = 2e-3;
  constexpr double kTc = 0.8;
  const auto ball_at = [](double t) {
    fx::BallState x;
    x << -2.5, 0.2, 0.8, 4.0, -0.2, 3.5, 0.0229;
    if (t > 0.0)
      (void)fx::Propagate(fx::BallModel{}, x, t, 1e-4, 1 << 20);
    return x;
  };
  SoftCatchTranslation::Params prm;
  prm.omega = 10.0;
  prm.zeta = 1.0;
  prm.a_max = 1e9;
  prm.v_max = 1e9;
  SoftCatchTranslation ds(prm);
  ASSERT_TRUE(ds.Reset({0.3, 0.0, 0.6}, Eigen::Vector3d::Zero()));
  const double Tw = 0.30;
  ASSERT_TRUE(ds.SetIntercept(ball_at(kTc).head<3>(), GammaProfile{0.0, 0.4, kTc - Tw, kTc}));
  double peak = 0.0;
  const int N = static_cast<int>(kTc / kDt + 0.5);
  for (int k = 0; k < N; ++k) {
    const double t = k * kDt;
    const fx::BallState x = ball_at(t);
    const TargetState o{x.head<3>(), x.segment<3>(3), fx::F(fx::BallModel{}, x).segment<3>(3)};
    const auto out = ds.Step(o, t, kDt);
    ASSERT_TRUE(out.valid);
    if (t >= kTc - Tw)
      peak = std::max(peak, out.u_des.norm());
  }
  EXPECT_LT(std::abs(peak - 33.0), 0.05 * 33.0) << peak;
}

// Directional speed (L3 §4.5 v0.5): the numerator is the projection on v̂, not
// the norm — leakage off v̂ must not count as speed — and a reverse
// projection is 0.
TEST(CatchingTimeFeasibility, G3BDirectionalSpeedUsesProjection) {
  const Eigen::Vector3d v_hat(1.0, 0.0, 0.0);
  const std::array<double, 2> qdot_u{0.5, -0.25};
  const std::array<double, 2> qdot_max{1.0, 1.0};  // denom = max(0.5, 0.25) = 0.5
  // DLS achieved 0.9 along v̂ plus 0.4 sideways leakage.
  const DirectionalSpeed r = DirectionalSpeedMax(v_hat, {0.9, 0.4, 0.0}, qdot_u, qdot_max);
  EXPECT_FALSE(r.limits_invalid || r.input_invalid || r.undetermined);
  EXPECT_DOUBLE_EQ(r.v_dir_max, 0.9 / 0.5);  // norm would give √0.97/0.5
  EXPECT_EQ(DirectionalSpeedMax(v_hat, {-0.3, 0.1, 0.0}, qdot_u, qdot_max).v_dir_max, 0.0);
}

// Zero guards: a non-positive / NaN joint limit is a flag with v = 0, a zero
// q̇ᵘ is "undetermined" with v = 0, malformed inputs are invalid.
TEST(CatchingTimeFeasibility, G3BDirectionalSpeedGuards) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector3d v_hat(0.0, 1.0, 0.0);
  const Eigen::Vector3d jp(0.0, 0.8, 0.0);
  const std::array<double, 2> qdot_u{0.5, 0.2};
  for (const std::array<double, 2> lim :
       {std::array<double, 2>{1.0, 0.0}, std::array<double, 2>{1.0, -2.0},
        std::array<double, 2>{nan, 1.0}}) {
    const DirectionalSpeed r = DirectionalSpeedMax(v_hat, jp, qdot_u, lim);
    EXPECT_TRUE(r.limits_invalid);
    EXPECT_EQ(r.v_dir_max, 0.0);
  }
  const std::array<double, 2> zeros{0.0, 0.0};
  const std::array<double, 2> lim{1.0, 1.0};
  const DirectionalSpeed z = DirectionalSpeedMax(v_hat, jp, zeros, lim);
  EXPECT_TRUE(z.undetermined);
  EXPECT_EQ(z.v_dir_max, 0.0);
  EXPECT_TRUE(DirectionalSpeedMax({0.0, 2.0, 0.0}, jp, qdot_u, lim).input_invalid);  // not unit
  EXPECT_TRUE(DirectionalSpeedMax(v_hat, {nan, 0, 0}, qdot_u, lim).input_invalid);
  const std::array<double, 2> bad_qdu{nan, 0.1};
  EXPECT_TRUE(DirectionalSpeedMax(v_hat, jp, bad_qdu, lim).input_invalid);
  const std::array<double, 3> long_lim{1.0, 1.0, 1.0};
  EXPECT_TRUE(DirectionalSpeedMax(v_hat, jp, qdot_u, long_lim).input_invalid);
}

// ── stopping distance and error budget (L3 §4.9, §4.6) ─────────────────────

TEST(CatchingTimeFeasibility, StoppingPointAlongBallDirection) {
  const Eigen::Vector3d p_c(0.5, 0.0, 0.8);
  const Eigen::Vector3d v_c(-6.0, 0.0, -8.0);  // ‖v‖ = 10
  const StoppingReservation r = StoppingPoint(p_c, v_c, 0.4, 20.0);
  ASSERT_TRUE(r.valid);
  EXPECT_DOUBLE_EQ(r.distance, (0.4 * 10.0) * (0.4 * 10.0) / (2.0 * 20.0));  // 0.4 m
  EXPECT_LT((r.p_stop - (p_c + 0.4 * Eigen::Vector3d(-0.6, 0.0, -0.8))).norm(), 1e-15);
  const StoppingReservation rest = StoppingPoint(p_c, Eigen::Vector3d::Zero(), 0.4, 20.0);
  EXPECT_TRUE(rest.valid);
  EXPECT_EQ(rest.distance, 0.0);
  EXPECT_EQ(rest.p_stop, p_c);
  EXPECT_FALSE(StoppingPoint(p_c, v_c, 0.4, 0.0).valid);
  EXPECT_FALSE(StoppingPoint(p_c, v_c, 1.2, 20.0).valid);
}

// σ_gap² = (1−γ)²σ_c² + (2γ−γ²)σ_ℓ² + σ_trk² + (‖v‖δ)²: γ = 0 → σ_c, γ = 1 →
// σ_ℓ, σ_ℓ = σ_c → σ_c for every γ (the planning-time upper bound), and the
// v0.1 γ² coefficient under-estimates (15.5 % at σ_ℓ/σ_c = 0.5, γ = 0.5).
TEST(CatchingTimeFeasibility, CatchErrorSigmaProperties) {
  EXPECT_DOUBLE_EQ(CatchErrorSigma(0.0, 0.02, 0.01, 0.0, 0.0, 0.0), 0.02);
  EXPECT_DOUBLE_EQ(CatchErrorSigma(1.0, 0.02, 0.01, 0.0, 0.0, 0.0), 0.01);
  for (const double g : {0.1, 0.4, 0.7})
    EXPECT_NEAR(CatchErrorSigma(g, 0.02, 0.02, 0.0, 0.0, 0.0), 0.02, 1e-15);
  const double ours = CatchErrorSigma(0.5, 1.0, 0.5, 0.0, 0.0, 0.0);
  const double v01 = std::sqrt(0.25 * 1.0 + 0.25 * 0.25);  // γ² coefficient
  EXPECT_NEAR(1.0 - v01 / ours, 0.155, 1e-3);
  EXPECT_NEAR(CatchErrorSigma(0.0, 0.0, 0.0, 0.003, 8.0, 0.0005), 0.005, 1e-15);  // 3-4-5
  EXPECT_TRUE(std::isnan(CatchErrorSigma(1.5, 0.02, 0.01, 0.0, 0.0, 0.0)));
  EXPECT_TRUE(ErrorBudgetOk(3.0, 0.01, 0.03));
  EXPECT_FALSE(ErrorBudgetOk(3.0, 0.011, 0.03));
  EXPECT_FALSE(ErrorBudgetOk(3.0, std::numeric_limits<double>::quiet_NaN(), 0.03));
}

// RT: the gate functions allocate nothing (the planner may run SCHED_FIFO).
TEST(CatchingTimeFeasibility, GatesAllocationFree) {
  const std::array<double, 6> q0{0.1, -0.2, 0.3, 0.0, 0.5, -0.4};
  const std::array<double, 6> w0{0.0, 0.5, -0.3, 0.1, 0.0, 0.2};
  const std::array<double, 6> q1{1.0, 0.2, -0.3, 0.4, 0.0, 0.1};
  const std::array<double, 6> wm{kPi, kPi, kPi, kPi, kPi, kPi};
  const std::array<double, 6> am{10, 10, 10, 10, 10, 10};
  double sink = 0.0;
  std::size_t heap = 0;
  std::uint64_t eigen = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    const TMinResult j = MaxJointTMin(q0, w0, q1, wm, am);
    sink +=
        j.t +
        (ReachTimeFeasible(BallTime{2'000'000'000}, MakeNowLead(NowReal{0}, 5), 0, j) ? 1.0 : 0.0);
    sink += ComputeGammaWindow(5.0, 2.0, PlanningTcpSpeed(0.8, 2.0), 0.04, 0.06).g_max;
    sink += MaxCatchableSpeed(2.0, 1.6, 0.04, 0.06);
    sink += DirectionalSpeedMax({1, 0, 0}, {0.9, 0.1, 0}, q0, wm).v_dir_max;
    sink += StoppingPoint({0.5, 0, 0.8}, {-6, 0, -8}, 0.4, 20.0).distance;
    sink += CatchErrorSigma(0.4, 0.02, 0.015, 0.003, 6.0, 0.0005);
    heap = heap_gate.count();
    eigen = eigen_gate.violations();
  }
  EXPECT_EQ(heap, 0u);
  EXPECT_EQ(eigen, 0u);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
