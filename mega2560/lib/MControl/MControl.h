#pragma once

#include "driver/gpio.h"
#include <Arduino.h>
#include <math.h>

#define square(x) ((x) * (x))

#define TIME1_PER_90 1.8
#define TIME2_PER_90 2

#define Tolerance_Theta1 1 * PI / 180
#define Tolerance_Theta2 4 * PI / 180

#define LEFT_CONSTRAINT_MOTOR1 130 * PI / 180
#define RIGHT_CONSTRAINT_MOTOR1 50 * PI / 180
#define LEFT_CONSTRAINT_MOTOR2 140 * PI / 180
#define RIGHT_CONSTRAINT_MOTOR2 -140 * PI / 180

enum CMD_TYPE { NONE, HOME, POINT, LINE, CIRCLE };
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

struct Command {
  CMD_TYPE type;
  point p1,     // POINT: target | LINE: start | CIRCLE: center
      p2;       // LINE: end
  float radius; // CIRCLE
};
struct CmdBuffer {
  byte data[20], len;
};

// PC -> ESP32 command packet (exactly 11 bytes):
// AA 55 J1_MSB J1_LSB J2_MSB J2_LSB TAU1_MSB TAU1_LSB TAU2_MSB TAU2_LSB
// CHECKSUM
//
// JOINT1/JOINT2 are signed int16_t desired encoder counts (MSB first).
// TAU1/TAU2 are signed int16_t dynamics feedforward torques (MSB first),
// 1 LSB = 0.001 N*m (range +-32.767 N*m).
// checksum = low 8 bits of the sum of bytes 0..9 (i.e. including the 0xAA 0x55
// header and the four 16-bit fields).
//
// The control frame keeps the original 7-byte form:
// AA 56 CMD 00 00 00 CHECKSUM (checksum over bytes 2..5, unchanged).

enum PACKET_RX_STATE {
  WAIT_HEADER_1,
  WAIT_HEADER_2,
  READ_JOINT1_MSB,
  READ_JOINT1_LSB,
  READ_JOINT2_MSB,
  READ_JOINT2_LSB,
  READ_TAU1_MSB,
  READ_TAU1_LSB,
  READ_TAU2_MSB,
  READ_TAU2_LSB,
  READ_CHECKSUM
};

// Packet buffer
struct BinaryPacket {
  uint8_t joint1Msb;
  uint8_t joint1Lsb;
  uint8_t joint2Msb;
  uint8_t joint2Lsb;
  uint8_t tau1Msb;
  uint8_t tau1Lsb;
  uint8_t tau2Msb;
  uint8_t tau2Lsb;
  uint8_t checksum;
};

point operator*(float k, point p);

extern const float mass_link[2], link_length[], center_length[];

class INPUT_DATA {
public:
  // input Data
  void scan(char *line, byte buffer[], byte *length, boolean *newData);
  void recvByte(char *line, byte buffer[], boolean *newData);
  void input_coor(byte buffer[], byte *length, boolean *newData);
  void input_angle(byte buffer[], byte *length, boolean *newData);

  // parse Data & Motion
  void parseData(char *line, byte buffer[], boolean *newData);
  bool parseCommand(char *line, Command &cmd);
  void parseQueue(char *line);
  void parsePoint(char *line, byte buffer[], boolean *newData);
  void parseLine(char *line, byte buffer[], boolean *newData);
  void parseCircle(char *line, byte buffer[], boolean *newData);

  // convert Data
  long convertbyte2enc(unsigned char recvchar[], byte startpos,
                       float pulse_num);
  void convertbyte2coor(point &p, unsigned char recvchar[], byte startpos);
  circle convertData2Circle(circle Circle, unsigned char recvchar[],
                            byte startpos);

  // Queue command
  bool enqueueCmd(const Command &cmd);
  bool dequeueCmd(Command &cmd);
  bool isEmpty();
  bool isFull();

  bool enqueueBuffer(byte buffer[], byte length);
  bool dequeueBuffer(byte buffer[], byte &length);

private:
  static const byte SIZE = 32;
  Command queue[SIZE];
  CmdBuffer Queue[SIZE];
  byte head = 0, tail = 0;
};

class PROCESSING {
public:
  // constructor
  byte read_A, read_B, dir_A, dir_B, Veloc, Veloc_A, Veloc_B;

  PROCESSING(byte a, byte b, byte c, byte d, float e)
      : read_A(a), read_B(b), dir_A(c), dir_B(d), Veloc(e) {}

  PROCESSING(byte a1, byte b1, byte c1, byte d1)
      : read_A(a1), read_B(b1), Veloc_A(c1), Veloc_B(d1) {}

  volatile long count_enc = 0;

  Torque masstorque, cori, grav, torque;
  // get pulse
  byte getcount() { return count_enc; };
  byte getstate() { return state_4bit; };
  byte getlaststate() { return last_state; };

  // calculate & executive function
  void count(byte shift);
  void running(float pwm, int pwm_resolution);
  Torque MassMatrix(joint Theta);
  Torque CoriolisMatrix(joint Theta, Torque dq);
  Torque GravityMatrix(joint Theta);
  Torque Torquecalc(joint Theta, Torque dq, Torque ddq);
  void setPID(float kp, float ki, float kd);
  float PIDcalc(float error);
  // Clear integral/derivative history.  Called only at controller lifecycle
  // boundaries (entering a settled hold, or re-engaging after a stop), never
  // every control cycle.
  void resetPID();

  // non-linear interpolation
  void splinePoint(point start, point target, point vector0, point vector1,
                   point &Point, float t);
  void splineTheta(joint start, joint target, joint vector0, joint vector1,
                   joint &Joint, float t);
  float interQuintic(float delta_time, float time_sum);

  // 3 mode trajectory
  float movePoint(float startMotor, float targetMotor, float time_sum);
  void moveLine(point start, point target2, point &p, float time_sum);
  point projectionCircle(circle &Circle, point currentPos);
  void moveCircle(circle Circle, point &POINT, float time_sum);
  void emergencyStop(byte stopPin);
  void homing(byte homePin, boolean activeState, boolean *isHoming);

private:
  byte last_state = 0, state_2bit = 0, state_4bit = 0;
  unsigned long last_time = 0;

  // PID parameters:
  float Kp = 0, Ki = 0, Kd = 0; // PID for JGB37-520 4 1 2
  float pre_error = 0, integral = 0, output = 0;

  // moving coordinates
  // point currentPos = {0,0,0}, startPos = {0,0,0}, targetPos  = {0,0,0},
  // centerPos  = {0,0,0};
};

class KINEMATICS {
public:
  bool RRplanar_in(point &POINT, joint &theta);
  bool fk_Match(joint &theta, point &point);
  bool checkLine2R(point &start, point &end, joint &startTheta,
                   joint &endTheta);
  bool bresenhamLine(point start, point target, point &Point);
  bool checkCircle2R(circle &Circle /*, joint &target*/);

private:
};
