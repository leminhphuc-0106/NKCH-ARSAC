// Offline torque-normalization test tool (PC side, evaluation only).
//
// Uses the REAL trajectory generators (DATAA::RRplanar_in / planLine /
// planCircle) and the REAL inverse dynamics (DATAA::inverseDynamics) to:
//   * measure the maximum +/- / |tau| per joint over POINT/LINE/CIRCLE samples,
//   * derive the normalization scales tauScale1Nm / tauScale2Nm,
//   * write a separate test CSV (no existing trajectory data is touched), and
//   * validate the mapping tau -> pwmNorm with representative test vectors.
//
// The PWMNorm columns reproduce the EXACT mapping implemented in
// mega2560/src/main.cpp (normalizeTorquePwm).  This tool does NOT drive motors
// and does NOT change any production code.

#include "Calculate_Lib.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// ---- Must mirror mega2560/src/main.cpp::normalizeTorquePwm exactly ----
static int16_t normalizeTorquePwm(float tauNm, float scaleNm) {
  // Reject non-finite, zero, near-zero AND negative scales (a negative scale is
  // invalid, not a sign inversion).  The scale must be strictly positive.
  if (!isfinite(scaleNm) || !(scaleNm > 1e-6f))
    return 0;
  float v = (tauNm / scaleNm) * 255.0f;
  if (!isfinite(v))
    return 0;
  if (v > 255.0f)
    v = 255.0f;
  else if (v < -255.0f)
    v = -255.0f;
  return static_cast<int16_t>(lroundf(v));
}

struct Row {
  string type;
  int idx;
  double t, q1, q2, dq1, dq2, ddq1, ddq2, tau1, tau2;
};

// Reproduce the HMI stream (10 ms grid, u = t/T, quintic easing of the sample
// index) but compute reference derivatives analytically through the chain rule,
// so a near-singular IK sample does not produce a finite-difference spike:
//   idx = eased(u)*(n-1);  dIdx/dt = (n-1)eased'(u)/T;  d2Idx/dt2 = (n-1)eased''(u)/T^2
//   q'  = dq/dIdx (central difference on the counts), q'' = d2q/dIdx2
//   qdot = q'*dIdx/dt;  qddot = q''*(dIdx/dt)^2 + q'*d2Idx/dt2
static vector<Row> simulate(DATAA &calc, const string &type,
                            const vector<int16_t> &counts, int n, double T) {
  vector<Row> rows;
  const double step = 0.010;
  auto qAt = [&](int j) -> pair<double, double> {
    if (j < 0) j = 0;
    if (j >= n) j = n - 1;
    return {counts[2 * j] * 2.0 * PI / COUNTS_PER_REV_1,
            counts[2 * j + 1] * 2.0 * PI / COUNTS_PER_REV_2};
  };
  for (int k = 0;; ++k) {
    double t = k * step;
    if (t > T + 1e-9)
      break;
    double u = (T <= 0.0) ? 1.0 : t / T;
    if (u > 1.0) u = 1.0;
    double eased = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    double easedD = 30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u;
    double easedDD = 60.0 * u - 180.0 * u * u + 120.0 * u * u * u;
    int idx = static_cast<int>(lround(eased * (n - 1)));
    if (idx < 0) idx = 0;
    if (idx >= n) idx = n - 1;

    double q1 = qAt(idx).first, q2 = qAt(idx).second;
    double q1p = (qAt(idx + 1).first - qAt(idx - 1).first) / 2.0;
    double q2p = (qAt(idx + 1).second - qAt(idx - 1).second) / 2.0;
    double q1pp = qAt(idx + 1).first - 2.0 * q1 + qAt(idx - 1).first;
    double q2pp = qAt(idx + 1).second - 2.0 * q2 + qAt(idx - 1).second;

    const double didt = (n - 1) * easedD / max(T, 1e-9);
    const double d2idt2 = (n - 1) * easedDD / max(T * T, 1e-9);
    double dq1 = q1p * didt, dq2 = q2p * didt;
    double ddq1 = q1pp * didt * didt + q1p * d2idt2;
    double ddq2 = q2pp * didt * didt + q2p * d2idt2;

    joint q = {(float)q1, (float)q2};
    Torque dqv = {(float)dq1, (float)dq2};
    Torque ddqv = {(float)ddq1, (float)ddq2};
    Torque tau = calc.inverseDynamics(q, dqv, ddqv);
    rows.push_back({type, idx, t, q1, q2, dq1, dq2, ddq1, ddq2, tau.t1, tau.t2});
  }
  return rows;
}

// POINT: linear joint-count grid from start counts to target counts (matches
// MainForm.BuildJointReference), duration from the TIME*_PER_90 model.
static bool planPoint(DATAA &calc, short s1, short s2, int x, int y,
                      vector<int16_t> &counts, int &n, double &T, string &label) {
  point target = {x, y, 0};
  joint result = {0.0f, 0.0f};
  if (!calc.RRplanar_in(target, result))
    return false;
  int16_t t1, t2;
  calc.jointToCounts(result, t1, t2);
  double dq1 = fabs(result.theta1 -
                    (s1 * 2.0 * PI / COUNTS_PER_REV_1));
  double dq2 = fabs(result.theta2 -
                    (s2 * 2.0 * PI / COUNTS_PER_REV_2));
  T = max(0.1, max(dq1 * TIME1_PER_90 / (PI / 2.0),
                   dq2 * TIME2_PER_90 / (PI / 2.0)));
  n = max(2, (int)ceil(T / 0.010) + 1);
  counts.assign(n * 2, 0);
  for (int i = 0; i < n; ++i) {
    double f = (double)i / (n - 1);
    counts[2 * i] = (int16_t)lround(s1 + (t1 - s1) * f);
    counts[2 * i + 1] = (int16_t)lround(s2 + (t2 - s2) * f);
  }
  ostringstream os; os << "POINT(" << x << "," << y << ")"; label = os.str();
  return true;
}

int main() {
  DATAA calc;
  const short startCount1 = 0;
  const short startCount2 = COUNTS_PER_REV_2 / 2; // 990 (q2 = PI)

  vector<Row> all;
  auto addRows = [&](vector<Row> r) {
    for (auto &row : r) all.push_back(row);
  };

  // POINT set
  int skipPoint = 0, skipLine = 0, skipCircle = 0;
  const int pointTargets[][2] = {{10, 10}, {-5, 15}, {15, 5}};
  for (auto &pt : pointTargets) {
    vector<int16_t> counts; int n = 0; double T = 0; string label;
    if (planPoint(calc, startCount1, startCount2, pt[0], pt[1], counts, n, T, label))
      addRows(simulate(calc, label, counts, n, T));
    else {
      ++skipPoint;
      cout << "[SKIP] " << pt[0] << "," << pt[1] << " unreachable\n";
    }
  }

  // LINE set
  const int lines[][4] = {{10, 0, 15, 10}, {10, 3, 15, 10}, {12, 0, 8, 12}};
  for (auto &l : lines) {
    vector<int16_t> counts(4096 * 2);
    int n = 0; float T = 0;
    point a = {l[0], l[1], 0}, b = {l[2], l[3], 0};
    if (calc.planLine(a, b, counts.data(), 4096, n, T)) {
      counts.resize(n * 2);
      ostringstream os; os << "LINE(" << l[0] << "," << l[1] << "->" << l[2] << "," << l[3] << ")";
      addRows(simulate(calc, os.str(), counts, n, max((double)T, 0.2)));
    } else {
      ++skipLine;
      cout << "[SKIP] line\n";
    }
  }

  // CIRCLE set
  const int circles[][3] = {{10, 5, 3}, {12, 0, 2}, {8, 6, 2}};
  for (auto &c : circles) {
    vector<int16_t> counts(4096 * 2);
    int n = 0; float T = 0;
    circle path; path.p = {c[0], c[1], 0}; path.radius = c[2];
    if (calc.planCircle(path, counts.data(), 4096, n, T)) {
      counts.resize(n * 2);
      ostringstream os; os << "CIRCLE(" << c[0] << "," << c[1] << ",r" << c[2] << ")";
      addRows(simulate(calc, os.str(), counts, n, max((double)T, 0.2)));
    } else {
      ++skipCircle;
      cout << "[SKIP] circle\n";
    }
  }

  // ---- Maxima ----
  // A sample is used only if it respects the joint-2 range (|q2| <= 140 deg,
  // the project constraint) and a physically plausible acceleration bound
  // (|ddq| <= 300 rad/s^2).  Samples outside are near-singularity / limit
  // artifacts of the reference generators, not representative dynamics.  The
  // exclusion reasons are counted separately below.
  const double q2Limit = 140.0 * PI / 180.0;
  const double accelLimit = 300.0;
  auto nonFinite = [](const Row &r) {
    return !isfinite(r.q1) || !isfinite(r.q2) || !isfinite(r.dq1) ||
           !isfinite(r.dq2) || !isfinite(r.ddq1) || !isfinite(r.ddq2) ||
           !isfinite(r.tau1) || !isfinite(r.tau2);
  };
  auto jointLimit = [&](const Row &r) { return fabs(r.q2) > q2Limit; };
  auto highAccel = [&](const Row &r) {
    return fabs(r.ddq1) > accelLimit || fabs(r.ddq2) > accelLimit;
  };
  double maxP1 = 0, minP1 = 0, maxA1 = 0, maxP2 = 0, minP2 = 0, maxA2 = 0;
  const Row *rA1 = nullptr, *rA2 = nullptr;
  int kept = 0, exNonFinite = 0, exLimitOnly = 0, exAccelOnly = 0, exBoth = 0;
  for (const auto &r : all) {
    const bool nf = nonFinite(r), jl = jointLimit(r), ha = highAccel(r);
    if (nf) { ++exNonFinite; continue; }
    if (jl || ha) {
      if (jl && ha) ++exBoth; else if (jl) ++exLimitOnly; else ++exAccelOnly;
      continue;
    }
    ++kept;
    if (r.tau1 > maxP1) maxP1 = r.tau1;
    if (r.tau1 < minP1) minP1 = r.tau1;
    if (fabs(r.tau1) > maxA1) { maxA1 = fabs(r.tau1); rA1 = &r; }
    if (r.tau2 > maxP2) maxP2 = r.tau2;
    if (r.tau2 < minP2) minP2 = r.tau2;
    if (fabs(r.tau2) > maxA2) { maxA2 = fabs(r.tau2); rA2 = &r; }
  }

  cout << fixed << setprecision(6);
  cout << "=== MAX TORQUE (N*m), samples=" << all.size() << " (valid=" << kept
       << " excluded=" << (exNonFinite + exLimitOnly + exAccelOnly + exBoth)
       << ") ===\n";
  cout << "Excluded reasons: nonFinite=" << exNonFinite
       << " joint2Limit(|q2|>140deg)=" << exLimitOnly
       << " highAccel(|ddq|>300)=" << exAccelOnly
       << " joint2Limit+highAccel=" << exBoth
       << " skippedPlans(point/line/circle)=" << skipPoint << "/" << skipLine
       << "/" << skipCircle << "\n";
  cout << "Joint1: max+=" << maxP1 << " max-=" << minP1 << " |max|=" << maxA1 << "\n";
  cout << "Joint2: max+=" << maxP2 << " max-=" << minP2 << " |max|=" << maxA2 << "\n";
  if (rA1) cout << "  |tau1| at " << rA1->type << " idx" << rA1->idx << " t=" << rA1->t
                << " q=(" << rA1->q1 << "," << rA1->q2 << ") dq=(" << rA1->dq1 << "," << rA1->dq2
                << ") ddq=(" << rA1->ddq1 << "," << rA1->ddq2 << ") tau1=" << rA1->tau1 << "\n";
  if (rA2) cout << "  |tau2| at " << rA2->type << " idx" << rA2->idx << " t=" << rA2->t
                << " q=(" << rA2->q1 << "," << rA2->q2 << ") dq=(" << rA2->dq1 << "," << rA2->dq2
                << ") ddq=(" << rA2->ddq1 << "," << rA2->ddq2 << ") tau2=" << rA2->tau2 << "\n";

  // Normalization scales derived from the measured maximum absolute torque,
  // rounded up to 3 decimal places.
  double scale1 = ceil(maxA1 * 1000.0) / 1000.0;
  double scale2 = ceil(maxA2 * 1000.0) / 1000.0;
  if (scale1 < 1e-3) scale1 = 1.0;
  if (scale2 < 1e-3) scale2 = 1.0;
  cout << "DERIVED SCALES: TAU_SCALE_1_NM=" << scale1 << " TAU_SCALE_2_NM=" << scale2 << "\n";

  // ---- CSV (separate folder; never overwrites trajectory data) ----
  filesystem::path dir("E:\\CODE\\Arduino\\codecode\\Data\\TorqueMap");
  error_code ec;
  filesystem::create_directories(dir, ec);
  time_t now = time(nullptr); tm lt = {}; localtime_s(&lt, &now);
  ostringstream fn; fn << "TorqueMap_" << put_time(&lt, "%Y%m%d_%H%M%S") << ".csv";
  filesystem::path out = dir / fn.str();
  ofstream csv(out);
  csv << "TrajectoryType,SampleIndex,Time_s,Q1_rad,Q2_rad,DQ1_rad_s,DQ2_rad_s,"
         "DDQ1_rad_s2,DDQ2_rad_s2,TauDyn1_Nm,TauDyn2_Nm,PWMNorm1,PWMNorm2\n";
  csv << setprecision(9);
  for (const auto &r : all) {
    int16_t p1 = normalizeTorquePwm((float)r.tau1, (float)scale1);
    int16_t p2 = normalizeTorquePwm((float)r.tau2, (float)scale2);
    csv << r.type << ',' << r.idx << ',' << r.t << ',' << r.q1 << ',' << r.q2 << ','
        << r.dq1 << ',' << r.dq2 << ',' << r.ddq1 << ',' << r.ddq2 << ','
        << r.tau1 << ',' << r.tau2 << ',' << p1 << ',' << p2 << '\n';
  }
  csv.close();
  cout << "CSV: " << out.string() << "\n";

  // ---- Mapping validation vectors ----
  cout << "=== MAPPING VALIDATION (scale1=" << scale1 << ", scale2=" << scale2 << ") ===\n";
  auto vec = [&](const char *name, float tau, float scale) {
    cout << "  " << name << ": tau=" << tau << " scale=" << scale
         << " -> pwmNorm=" << (int)normalizeTorquePwm(tau, scale) << "\n";
  };
  vec("zero", 0.0f, (float)scale1);
  vec("+scale", (float)scale1, (float)scale1);
  vec("-scale", -(float)scale1, (float)scale1);
  vec("+2x (saturate)", (float)(2 * scale1), (float)scale1);
  vec("-2x (saturate)", -(float)(2 * scale1), (float)scale1);
  vec("half", (float)(0.5 * scale1), (float)scale1);
  vec("zero-scale", 0.5f, 0.0f);
  vec("neg-scale", 0.5f, -1.0f);
  return 0;
}
