// Focused automated tests for the 2R inverse-dynamics model in Calculate_Lib.
// Validates the model against independently derived values (inertia formulas,
// energy/Lagrangian gravity gradient, analytic mass-matrix structure) rather
// than comparing two copies of the same formula.
#include "Calculate_Lib.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>
using namespace std;

static int fails = 0, checks = 0;
static void ck(bool c, const char *m) {
  ++checks;
  printf("  [%s] %s\n", c ? "PASS" : "FAIL", m);
  if (!c)
    ++fails;
}
static bool isClose(double a, double b, double tol) { return fabs(a - b) <= tol; }

// Independent link parameters (must match the task specification).
static const double M1 = 0.1362, M2 = 0.082, L1 = 0.20, L2 = 0.15, B1 = 0.04,
                    B2 = 0.03, R1 = L1 / 2, R2 = L2 / 2, G = 9.81;
static const double I1C = M1 * (L1 * L1 + B1 * B1) / 12.0;
static const double I2C = M2 * (L2 * L2 + B2 * B2) / 12.0;
static const double I1O = I1C + M1 * R1 * R1;
static const double I2O = I2C + M2 * R2 * R2;
static const double A = M2 * L1 * R2;

// Independent gravity vector from the Lagrangian potential U(q) = sum m g y,
// y_i measured upward (+y vertical), gravity along -y.  Uses finite differences
// of U so it is independent of the closed-form G used in the library.
static double U(double q1, double q2) {
  const double y1 = R1 * sin(q1);
  const double y2 = L1 * sin(q1) + R2 * sin(q1 + q2);
  return M1 * G * y1 + M2 * G * y2;
}
static void gravityRef(double q1, double q2, double &g1, double &g2) {
  const double h = 1e-6;
  g1 = (U(q1 + h, q2) - U(q1 - h, q2)) / (2 * h);
  g2 = (U(q1, q2 + h) - U(q1, q2 - h)) / (2 * h);
}

int main() {
  DATAA calc;

  // 1) rectangular-link inertias
  ck(isClose(I1C, 0.00047216, 1e-8), "I1c = 0.00047216 kg m^2");
  ck(isClose(I2C, 0.00015990, 1e-8), "I2c = 0.00015990 kg m^2");
  ck(isClose(I1O, 0.00183416, 1e-8), "I1O = 0.00183416 kg m^2");
  ck(isClose(I2O, 0.00062115, 1e-8), "I2O = 0.00062115 kg m^2");

  // Helper: extract mass-matrix columns via ddq basis with gravity disabled.
  auto Mcol = [&](double q1, double q2, int col, double &m1, double &m2) {
    joint q = {(float)q1, (float)q2};
    Torque dq = {0, 0};
    Torque ddq = {col == 0 ? 1.0f : 0.0f, col == 1 ? 1.0f : 0.0f};
    Torque t = calc.inverseDynamics(q, dq, ddq, 0.0f);
    m1 = t.t1; m2 = t.t2;
  };

  // 2/3) symmetry and positive-definiteness over a q2 grid
  bool sym = true, pd = true;
  for (int i = 0; i <= 24; ++i) {
    const double q2 = -3.14159265358979323846 + i * (2 * 3.14159265358979323846 / 24);
    double m11, m21, m12, m22;
    Mcol(0.7, q2, 0, m11, m21); // column 1
    Mcol(0.7, q2, 1, m12, m22); // column 2
    if (!isClose(m21, m12, 1e-6)) sym = false;
    if (!(m11 > 0 && (m11 * m22 - m12 * m12) > 0)) pd = false;
  }
  ck(sym, "M(q) symmetric (M12 == M21) over q2 grid");
  ck(pd, "M(q) positive definite over q2 grid");

  // Mass-matrix numeric anchors at q2=0
  double a11, a21, a12, a22;
  Mcol(0.0, 0.0, 0, a11, a21);
  Mcol(0.0, 0.0, 1, a12, a22);
  ck(isClose(a11, 0.00573531 + 0.00246, 1e-7), "M11(q2=0) = 0.00573531+0.00246");
  ck(isClose(a12, 0.00062115 + 0.00123, 1e-7), "M12(q2=0) = 0.00062115+0.00123");
  ck(isClose(a22, 0.00062115, 1e-8), "M22 = I2O = 0.00062115");

  // 4) qd=0, qdd=0 -> G(q)
  {
    joint q = {0.0f, 0.0f};
    Torque z = {0, 0};
    Torque t = calc.inverseDynamics(q, z, z);
    ck(isClose(t.t1, 0.2944962 + 0.0603315, 1e-6), "G1(q1=0,q2=0) = 0.3548277");
    ck(isClose(t.t2, 0.0603315, 1e-7), "G2(q1=0,q2=0) = 0.0603315");
  }

  // 5) gravity disabled, qd=0, qdd=0 -> zero torque
  {
    joint q = {0.4f, -0.9f};
    Torque z = {0, 0};
    Torque t = calc.inverseDynamics(q, z, z, 0.0f);
    ck(t.t1 == 0.0f && t.t2 == 0.0f, "gravity=0, qd=0, qdd=0 -> tau=0");
  }

  // 6) qd=0 -> Coriolis/centrifugal zero: tau(q,0,ddq) - G == M*ddq
  {
    joint q = {0.9f, 0.6f};
    Torque z = {0, 0}, ddq = {0.7f, -0.4f};
    Torque full = calc.inverseDynamics(q, z, ddq);
    Torque grav = calc.inverseDynamics(q, z, z);
    double m11, m21, m12, m22;
    Mcol(0.9, 0.6, 0, m11, m21);
    Mcol(0.9, 0.6, 1, m12, m22);
    ck(isClose(full.t1 - grav.t1, m11 * 0.7 + m12 * -0.4, 1e-6) &&
           isClose(full.t2 - grav.t2, m21 * 0.7 + m22 * -0.4, 1e-6),
       "qd=0: tau-G equals M*ddq (no Coriolis)");
  }

  // 7/8) independent ddq1 / ddq2 coupling, joint 2 uses M12*ddq1 + M22*ddq2
  {
    joint q = {0.3f, 0.8f};
    Torque z = {0, 0}, d1 = {1.0f, 0.0f}, d2 = {0.0f, 1.0f};
    Torque t1 = calc.inverseDynamics(q, z, d1, 0.0f);
    Torque t2 = calc.inverseDynamics(q, z, d2, 0.0f);
    double m11, m21, m12, m22;
    Mcol(0.3, 0.8, 0, m11, m21);
    Mcol(0.3, 0.8, 1, m12, m22);
    ck(isClose(t1.t2, m12, 1e-6) && fabs(m12) > 1e-4, "ddq2=0: tau2 = M12*ddq1 (coupling present)");
    ck(isClose(t2.t2, m22, 1e-6) && isClose(t2.t1, m12, 1e-6), "joint2 includes M22*ddq2 and M12*ddq1");
  }

  // 9) Coriolis/centrifugal signs (independent of the closed form)
  {
    const double q2 = 1.0; // sin>0
    joint q = {0.0f, (float)q2};
    Torque dqp = {0.5f, 0.4f};
    Torque z = {0, 0};
    Torque cpos = calc.inverseDynamics(q, dqp, z); // gravity adds; isolate by g=0
    Torque cpos0 = calc.inverseDynamics(q, dqp, z, 0.0f);
    Torque cneg0 = calc.inverseDynamics(q, {-0.5f, -0.4f}, z, 0.0f);
    // C2 = a sin(q2) dq1^2 > 0 for q2 in (0,pi); independent of dq1 sign
    ck(cpos0.t2 > 0, "C2 > 0 for q2 in (0,pi)");
    ck(isClose(cneg0.t2, cpos0.t2, 1e-9), "C2 independent of dq1 sign (dq1^2)");
    // C1 = -a sin(q2)(2 dq1 dq2 + dq2^2) < 0 when dq1,dq2>0 and sin(q2)>0
    ck(cpos0.t1 < 0, "C1 < 0 for q2>0, dq1>0, dq2>0");
    (void)cpos;
  }

  // 10) gravity vector vs independent energy-gradient reference
  {
    bool ok = true;
    for (int i = 0; i < 6; ++i) {
      double q1 = -1.0 + 0.4 * i, q2 = -0.9 + 0.35 * i;
      double rg1, rg2;
      gravityRef(q1, q2, rg1, rg2);
      joint q = {(float)q1, (float)q2};
      Torque z = {0, 0};
      Torque t = calc.inverseDynamics(q, z, z);
      if (!isClose(t.t1, rg1, 1e-5) || !isClose(t.t2, rg2, 1e-5))
        ok = false;
    }
    ck(ok, "G matches independent energy-gradient (finite-difference dU/dq)");
  }

  // 11) non-finite propagation (so callers' finite guard can reject it)
  {
    joint q = {NAN, 0.2f};
    Torque z = {0, 0};
    Torque t = calc.inverseDynamics(q, z, z);
    ck(!isfinite(t.t1), "non-finite input yields non-finite torque (caller must reject)");
  }

  // 12) protocol: signed encode/decode + additive checksum
  {
    auto enc = [](double nm) -> int16_t {
      double l = lround(nm * 1000.0);
      if (l > 32767) l = 32767;
      if (l < -32767) l = -32767;
      return (int16_t)l;
    };
    int16_t a = enc(0.6415), b = enc(-100.0);
    uint8_t f[11] = {0xAA, 0x55, (uint8_t)(a >> 8), (uint8_t)a, (uint8_t)(b >> 8),
                     (uint8_t)b, 0, 0, 0, 0, 0};
    uint8_t s = 0;
    for (int i = 0; i < 10; ++i) s = (uint8_t)(s + f[i]);
    int16_t da = (int16_t)((uint16_t(f[2]) << 8) | f[3]);
    ck(da == 642 && b == -32767 && s == (uint8_t)(0xAA + 0x55 + f[2] + f[3] + f[4] + f[5]),
       "torque encode/decode + checksum");
  }

  printf("DYNAMICS: %s (%d/%d checks passed)\n", fails ? "FAIL" : "PASS",
         checks - fails, checks);
  return fails ? 1 : 0;
}


