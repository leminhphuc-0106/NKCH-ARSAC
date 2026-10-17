#pragma once
#include <cmath>
#include <cstdint>
constexpr float PI = 3.14159265358979323846f;
using byte = uint8_t;
using boolean = bool;
struct point {
  int x, y, z;
};
struct joint {
  float theta1, theta2;
};
struct circle {
  point p;
  int radius;
};
struct Torque {
  float t1, t2;
};
class INPUT_DATA {
public:
  bool RRplanar_in(point &target, joint &theta);
  bool fk_Match(joint &theta, point &target);
  point forwardKinematics(const joint &theta) const;
  bool checkLine2R(const point &, const point &, const joint &, const joint &);
  bool checkCircle2R(const circle &);
  point projectionCircle(const circle &, point) const;
  void moveLine(const point &, const point &, point &, float) const;
  void moveCircle(const circle &, point &, float) const;
};
class PROCESSING {
public:
  PROCESSING(uint8_t, uint8_t, uint8_t, uint8_t, float) {}
  void setPID(float kp, float ki, float kd);
  void resetPID();
  float PIDcalc(float error, float dt);
  float interQuintic(float u) const;
  float movePoint(float start, float target, float u) const;
  Torque MassMatrix(joint theta);
  Torque CoriolisMatrix(joint theta, Torque dq);
  Torque GravityMatrix(joint theta);
  Torque Torquecalc(joint theta, Torque dq, Torque ddq);

private:
  float kp_ = 0, ki_ = 0, kd_ = 0, previousError_ = 0, integral_ = 0;
};
using KINEMATICS = INPUT_DATA;
extern const float mass_link[2], link_length[2], center_length[2];
