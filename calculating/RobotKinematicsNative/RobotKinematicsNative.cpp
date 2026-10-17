#include "../Calculate_Lib/Calculate_Lib.h"

// This is intentionally only an ABI adapter.  The inverse-kinematics
// implementation remains DATAA::RRplanar_in in Calculate_Lib.cpp, which is
// also the function used by calculate.cpp.
extern "C" __declspec(dllexport) int RobotCalculateInverseKinematics(
    int x, int y, float *q1, float *q2) {
  if (q1 == nullptr || q2 == nullptr)
    return 0;

  point target = {x, y, 0};
  joint result = {0.0f, 0.0f};
  DATAA calculator;
  if (!calculator.RRplanar_in(target, result))
    return 0;

  *q1 = result.theta1;
  *q2 = result.theta2;
  return 1;
}

// Full point-calculation result: runs the existing RRplanar_in solution
// filter and then converts the selected joint solution to the desired encoder
// counts with the shared Calculate_Lib conversion.  The HMI sends exactly
// these counts, so it no longer needs its own count conversion.
extern "C" __declspec(dllexport) int RobotCalculatePoint(
    int x, int y, float *q1, float *q2, short *count1, short *count2) {
  if (q1 == nullptr || q2 == nullptr || count1 == nullptr || count2 == nullptr)
    return 0;

  point target = {x, y, 0};
  joint result = {0.0f, 0.0f};
  DATAA calculator;
  if (!calculator.RRplanar_in(target, result))
    return 0;

  int16_t desired1 = 0;
  int16_t desired2 = 0;
  calculator.jointToCounts(result, desired1, desired2);

  *q1 = result.theta1;
  *q2 = result.theta2;
  *count1 = desired1;
  *count2 = desired2;
  return 1;
}

// 2R inverse dynamics for the streamed reference sample.  Returns the model
// torque tau = M(q)ddq + C(q,dq)dq + G(q) in N*m for each joint, using the same
// physical parameters as the firmware model.  q in rad, dq in rad/s, ddq in
// rad/s^2.  The HMI calls this per reference sample to build the 11-byte
// command packet feedforward torque fields.
extern "C" __declspec(dllexport) int RobotComputeTorque(
    float q1, float q2, float dq1, float dq2, float ddq1, float ddq2, float *t1,
    float *t2) {
  if (t1 == nullptr || t2 == nullptr)
    return 0;

  DATAA calculator;
  joint q = {q1, q2};
  Torque dq = {dq1, dq2};
  Torque ddq = {ddq1, ddq2};
  Torque out = calculator.inverseDynamics(q, dq, ddq);
  *t1 = out.t1;
  *t2 = out.t2;
  return 1;
}

// Quintic time scaling and analytic derivatives, exposed so the PC pipeline has
// a single definition: s(u), ds/dt, d2s/dt2 for total duration T (seconds).
extern "C" __declspec(dllexport) void RobotQuintic(float u, float T, float *s,
                                                   float *sd, float *sdd) {
  if (s != nullptr)
    *s = DATAA::quinticS(u);
  if (sd != nullptr)
    *sd = DATAA::quinticSd(u, T);
  if (sdd != nullptr)
    *sdd = DATAA::quinticSdd(u, T);
}

// Sample a straight-line Cartesian path into desired encoder counts.  Reuses
// DATAA::planLine (existing RRplanar_in filter + jointToCounts conversion).
// counts receives interleaved (count1, count2) pairs and must hold 2*maxSamples
// int16 values.  Returns 1 on success, 0 if the command is invalid or any
// sampled point is unreachable.
extern "C" __declspec(dllexport) int RobotPlanLine(
    int x1, int y1, int x2, int y2, short *counts, int maxSamples,
    int *sampleCount, float *duration) {
  if (counts == nullptr || sampleCount == nullptr || duration == nullptr ||
      maxSamples < 2)
    return 0;

  DATAA calculator;
  point start = {x1, y1, 0};
  point end = {x2, y2, 0};
  int samples = 0;
  float total = 0.0f;
  if (!calculator.planLine(start, end, reinterpret_cast<int16_t *>(counts),
                           maxSamples, samples, total))
    return 0;

  *sampleCount = samples;
  *duration = total;
  return 1;
}

// Sample a full circle Cartesian path into desired encoder counts.  Reuses
// DATAA::planCircle.  Returns 1 on success, 0 if the radius is invalid, the
// buffer is too small, or any sampled point is unreachable.
extern "C" __declspec(dllexport) int RobotPlanCircle(
    int cx, int cy, int radius, short *counts, int maxSamples,
    int *sampleCount, float *duration) {
  if (counts == nullptr || sampleCount == nullptr || duration == nullptr ||
      maxSamples < 8 || radius <= 0)
    return 0;

  DATAA calculator;
  circle path;
  path.p = {cx, cy, 0};
  path.radius = radius;
  int samples = 0;
  float total = 0.0f;
  if (!calculator.planCircle(path, reinterpret_cast<int16_t *>(counts),
                             maxSamples, samples, total))
    return 0;

  *sampleCount = samples;
  *duration = total;
  return 1;
}
