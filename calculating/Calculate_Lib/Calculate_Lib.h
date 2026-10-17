#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <math.h>
#include <mutex>
#include <queue>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <windows.h>

using namespace std;
using namespace chrono;
using std::string;
// using boolean = bool;

constexpr float PI = 3.14159265358979323846f;

// Encoder counts per complete revolution of each joint.  These mirror the
// firmware (mega2560) and the standalone calculate.cpp pipeline (pwm_num1 /
// pwm_num2) so the count conversion has a single C++ definition shared by the
// native calculation API.
#define COUNTS_PER_REV_1 11880
#define COUNTS_PER_REV_2 1980

#define TIME1_PER_90 1.8
#define TIME2_PER_90 2

#define Tolerance_Theta1 1 * PI / 180
#define Tolerance_Theta2 4 * PI / 180

#define LEFT_CONSTRAINT_MOTOR1 130 * PI / 180
#define RIGHT_CONSTRAINT_MOTOR1 50 * PI / 180
#define LEFT_CONSTRAINT_MOTOR2 140 * PI / 180
#define RIGHT_CONSTRAINT_MOTOR2 -140 * PI / 180

enum CMD_TYPE { NONE, HOME, COUNT, POINT_CMD, LINE, CIRCLE };
enum MOVE_MODE { NORMAL, THETA1_FIRST, THETA2_FIRST };

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
struct stateTheta {
  float Start, Current, Target;
};
struct statePoint {
  float Start, Current, Target;
};

struct Command {
  CMD_TYPE type;
  uint16_t count = 0;
  point p1,   // POINT: target | LINE: start | CIRCLE: center
      p2;     // LINE: end
  int radius; // CIRCLE
};
struct CmdBuffer {
  uint8_t data[20], len;
};
struct PositionPacket {
  int32_t pulse1;
  int32_t pulse2;
};

uint32_t millis();
extern const float mass_link[2], link_length[], center_length[];

class DATAA {
public:
  // convert Data
  bool recvCommand(Command &cmd);
  bool parseCommand(char *line, Command &cmd);
  void convert_Pulse2Byte(int32_t pulse, uint8_t sendchar[], uint8_t startpos);
  PositionPacket packPosition(int32_t pulse1, int32_t pulse2);

  // Queue command
  bool enqueueCmd(const Command &cmd);
  bool dequeueCmd(Command &cmd);
  bool isEmpty();
  bool isFull();

  bool enqueueBuffer(uint8_t buffer[], uint8_t length);
  bool dequeueBuffer(uint8_t buffer[], uint8_t &length);

  bool RRplanar_in(point &Point, joint &theta);
  bool fk_Match(joint &theta, point &point);

  // Convert a joint-angle solution to the desired encoder counts that are sent
  // to the firmware.  Uses the same rounding the existing calculate.cpp
  // trajectory pipeline uses, so the HMI does not need its own conversion.
  void jointToCounts(const joint &theta, int16_t &count1, int16_t &count2);
  bool checkLine2R(point &start, point &end, joint &startTheta,
                   joint &endTheta);
  bool checkCircle2R(circle &Circle);

  // ---- Reference derivatives and 2R inverse dynamics (PC-side feedforward) ----
  // Quintic time scaling s(u) = 10u^3 - 15u^4 + 6u^5 and its analytic time
  // derivatives for a move of total duration T seconds:
  //   sdot  = (30u^2 - 60u^3 + 30u^4) / T
  //   sddot = (60u  - 180u^2 + 120u^3) / T^2
  static float quinticS(float u);
  static float quinticSd(float u, float T);
  static float quinticSdd(float u, float T);

  // Full 2R planar inverse dynamics: tau = M(q)ddq + C(q,dq)dq + G(q), SI units.
  // Links are uniform rectangular prisms (length L, in-plane width b, mass m,
  // CoM at r=L/2); rotation axis is normal to the motion plane.  q1 is the
  // absolute angle of link 1 from the horizontal, q2 the relative angle of link
  // 2 (absolute link-2 angle is q1+q2); gravity acts along -y with g=9.81 m/s^2.
  // q in rad, dq in rad/s, ddq in rad/s^2, tau in N*m.  Only link masses and
  // inertias are modelled (no motor/gearbox/rotor/payload terms).
  Torque inverseDynamics(const joint &Theta, const Torque &dq,
                         const Torque &ddq);
  // Gravity strength is a parameter so the model can be validated with gravity
  // disabled (gravity = 0) against energy/Lagrangian references.
  Torque inverseDynamics(const joint &Theta, const Torque &dq,
                         const Torque &ddq, float gravity);

  // Sample an existing Cartesian path into desired encoder counts.
  //
  // Both planners reuse the existing RRplanar_in solution filter (nearest to
  // the previous sample, so the solution branch stays continuous) and the
  // existing jointToCounts conversion.  counts receives interleaved
  // (count1, count2) pairs and must hold at least 2*maxSamples int16 values.
  // duration is derived with the existing TIME*_PER_90 timing constants from
  // the accumulated joint travel, matching the Point Motion timing model.
  bool planLine(point start, point end, int16_t *counts, int maxSamples,
                int &sampleCount, float &duration);
  bool planCircle(circle Circle, int16_t *counts, int maxSamples,
                  int &sampleCount, float &duration);

private:
  static const uint8_t SIZE = 32;
  Command queue[SIZE];
  CmdBuffer Queue[SIZE];
  uint8_t head = 0, tail = 0;
};

class PROCESSING {
public:
  // constructor
  PROCESSING(uint8_t a, uint8_t b, uint8_t c, uint8_t d, float e)
      : read_A(a), read_B(b), dir_A(c), dir_B(d), Veloc(e) {}
  uint8_t read_A, read_B, dir_A, dir_B, Veloc;

  volatile long count_enc = 0;

  Torque masstorque, cori, grav, torque;
  // get pulse
  uint8_t getcount() { return count_enc; };
  uint8_t getstate() { return state_4bit; };
  uint8_t getlaststate() { return last_state; };

  // calculate & executive function
  Torque MassMatrix(joint Theta);
  Torque CoriolisMatrix(joint Theta, Torque dq);
  Torque GravityMatrix(joint Theta);
  Torque Torquecalc(joint Theta, Torque dq, Torque ddq);
  void setPID(float kp, float ki, float kd);
  float PIDcalc(float error);
  void writeData();

  // non-linear interpolation
  // 3 mode trajectory
  float interQuintic(float delta_time, float time_sum);
  float movePoint(stateTheta theta, float time_sum);

  int pwm_num, data;
  float error_interpolate, timeStart, timming, timeSum;
  stateTheta Theta;
  statePoint Point;

private:
  uint8_t last_state = 0, state_2bit = 0, state_4bit = 0;
  unsigned long last_time = 0;

  // PID parameters:
  float Kp = 0, Ki = 0, Kd = 0; // PID for JGB37-520 4 1 2
  float pre_error = 0, integral = 0, output = 0;
};
