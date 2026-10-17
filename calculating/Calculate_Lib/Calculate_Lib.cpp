#include "Calculate_Lib.h"

using namespace chrono;

const float mass_link[2] = {0.1362, 0.0826}, link_length[] = {20, 15},
            center_length[2] = {10, 7.5};

uint32_t millis() {
  static auto start = steady_clock::now();
  return duration_cast<milliseconds>(steady_clock::now() - start).count();
}

bool DATAA::recvCommand(Command &cmd) {
  string line;

  cout << "> ";
  if (!getline(cin, line))
    return false;

  if (line.empty())
    return false;

  char buffer[128];
  strncpy(buffer, line.c_str(), sizeof(buffer));
  buffer[sizeof(buffer) - 1] = '\0';

  return parseCommand(buffer, cmd);
}

bool DATAA::parseCommand(char *line, Command &cmd) {
  char keyword[10];

  if (sscanf(line, "%9s", keyword) != 1)
    return false;

  if (strcmp(keyword, "COUNT") == 0) {
    if (sscanf(line, "%*s %hu", &cmd.count) != 1)
      return false;
    cmd.type = COUNT;
    return true;
  }

  if (strcmp(keyword, "HOME") == 0) {
    cmd.type = HOME;
    return true;
  }

  if (strcmp(keyword, "POINT_CMD") == 0) {
    if (sscanf(line, "%*s %d %d %d", &cmd.p1.x, &cmd.p1.y, &cmd.p1.z) != 3)
      return false;

    cmd.type = POINT_CMD;
    return true;
  }

  if (strcmp(keyword, "LINE") == 0) {
    char to[5];

    if (sscanf(line, "%*s %d %d %d %4s %d %d %d", &cmd.p1.x, &cmd.p1.y,
               &cmd.p1.z, to, &cmd.p2.x, &cmd.p2.y, &cmd.p2.z) != 7)
      return false;

    if (strcmp(to, "TO") != 0)
      return false;

    cmd.type = LINE;
    return true;
  }

  if (strcmp(keyword, "CIRCLE") == 0) {
    if (sscanf(line, "%*s %d %d %d %d", &cmd.p1.x, &cmd.p1.y, &cmd.p1.z,
               &cmd.radius) != 4)
      return false;

    cmd.type = CIRCLE;
    return true;
  }

  return false;
}

void DATAA::convert_Pulse2Byte(int32_t pulse, uint8_t sendchar[],
                               uint8_t startpos) {
  sendchar[startpos] = (pulse >> 0) & 0xFF;
  sendchar[startpos + 1] = (pulse >> 8) & 0xFF;
  sendchar[startpos + 2] = (pulse >> 16) & 0xFF;
  sendchar[startpos + 3] = (pulse >> 24) & 0xFF;
}

bool DATAA::enqueueCmd(const Command &cmd) {
  int next = (tail + 1) % SIZE;
  if (next == head)
    return false;

  queue[tail] = cmd;
  tail = next;
  return true;
}

bool DATAA::dequeueCmd(Command &cmd) {
  if (head == tail)
    return false;

  cmd = queue[head];
  head = (head + 1) % SIZE;
  return true;
}

bool DATAA::isEmpty() { return head == tail; }

bool DATAA::isFull() { return ((tail + 1) % SIZE) == head; }

bool DATAA::enqueueBuffer(uint8_t buffer[], uint8_t length) {
  int next = (tail + 1) % SIZE;
  if (next == head)
    return false;

  memcpy(Queue[tail].data, buffer, length);
  Queue[tail].len = length;

  tail = next;
  return true;
}

bool DATAA::dequeueBuffer(uint8_t buffer[], uint8_t &length) {
  if (head == tail)
    return false;

  memcpy(buffer, Queue[head].data, Queue[head].len);
  length = Queue[head].len;

  head = (head + 1) % SIZE;
  return true;
}

bool DATAA::RRplanar_in(point &Point, joint &theta) {
  float bestCost = 1e9;
  bool found = false;

  joint opt1, opt2, opt3, opt4;
  bool valid1 = false, valid2 = false, valid3 = false, valid4 = false;

  float x = Point.x, y = Point.y, l1 = link_length[0], l2 = link_length[1],
        r = sqrt(x * x + y * y);
  if (r > l1 + l2 || r < fabs(l1 - l2))
    return false;

  float tempTheta2_1 =
            acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2)),
        tempTheta2_2 =
            -acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));

  float tempTheta1_11 = atan2(y, x) - atan2((l2 * sin(tempTheta2_1)),
                                            (l1 + l2 * cos(tempTheta2_1))),
        tempTheta1_12 = atan2(y, x) + atan2((l2 * sin(tempTheta2_1)),
                                            (l1 + l2 * cos(tempTheta2_1))),
        tempTheta1_21 = atan2(y, x) - atan2((l2 * sin(tempTheta2_2)),
                                            (l1 + l2 * cos(tempTheta2_2))),
        tempTheta1_22 = atan2(y, x) + atan2((l2 * sin(tempTheta2_2)),
                                            (l1 + l2 * cos(tempTheta2_2)));

  if (tempTheta1_11 >= 0 && tempTheta1_11 <= PI && tempTheta2_1 <= PI &&
      tempTheta2_1 >= -170 * PI / 180) {
    opt1 = {tempTheta1_11, tempTheta2_1};
    valid1 = true;
  }

  if (tempTheta1_12 >= 0 && tempTheta1_12 <= PI && tempTheta2_1 <= PI &&
      tempTheta2_1 >= -170 * PI / 180) {
    opt2 = {tempTheta1_12, tempTheta2_1};
    valid2 = true;
  }

  if (tempTheta1_21 >= 0 && tempTheta1_21 <= PI && tempTheta2_2 <= PI &&
      tempTheta2_2 >= -170 * PI / 180) {
    opt3 = {tempTheta1_21, tempTheta2_2};
    valid3 = true;
  }

  if (tempTheta1_22 >= 0 && tempTheta1_22 <= PI && tempTheta2_2 <= PI &&
      tempTheta2_2 >= -170 * PI / 180) {
    opt4 = {tempTheta1_22, tempTheta2_2};
    valid4 = true;
  }

  if (valid1 && !fk_Match(opt1, Point))
    valid1 = false;
  if (valid2 && !fk_Match(opt2, Point))
    valid2 = false;
  if (valid3 && !fk_Match(opt3, Point))
    valid3 = false;
  if (valid4 && !fk_Match(opt4, Point))
    valid4 = false;

  if (valid1) {
    float c =
        fabs(opt1.theta1 - theta.theta1) + fabs(opt1.theta2 - theta.theta2);
    if (c < bestCost) {
      bestCost = c;
      theta = opt1;
      found = true;
    }
  }
  if (valid2) {
    float c =
        fabs(opt2.theta1 - theta.theta1) + fabs(opt2.theta2 - theta.theta2);
    if (c < bestCost) {
      bestCost = c;
      theta = opt2;
      found = true;
    }
  }
  if (valid3) {
    float c =
        fabs(opt3.theta1 - theta.theta1) + fabs(opt3.theta2 - theta.theta2);
    if (c < bestCost) {
      bestCost = c;
      theta = opt3;
      found = true;
    }
  }
  if (valid4) {
    float c =
        fabs(opt4.theta1 - theta.theta1) + fabs(opt4.theta2 - theta.theta2);
    if (c < bestCost) {
      bestCost = c;
      theta = opt4;
      found = true;
    }
  }

  return found;
}

void DATAA::jointToCounts(const joint &theta, int16_t &count1,
                          int16_t &count2) {
  count1 = static_cast<int16_t>(lround(
      theta.theta1 * static_cast<double>(COUNTS_PER_REV_1) / (2.0 * PI)));
  count2 = static_cast<int16_t>(lround(
      theta.theta2 * static_cast<double>(COUNTS_PER_REV_2) / (2.0 * PI)));
}

bool DATAA::fk_Match(joint &theta, point &point) {
  float x = link_length[0] * cos(theta.theta1) +
            link_length[1] * cos(theta.theta1 + theta.theta2);

  float y = link_length[0] * sin(theta.theta1) +
            link_length[1] * sin(theta.theta1 + theta.theta2);

  return fabs(x - point.x) < 1e-5 && fabs(y - point.y) < 1e-5;
}

bool DATAA::checkLine2R(point &start, point &end, joint &startTheta,
                        joint &endTheta) {
  if (!RRplanar_in(start, startTheta))
    return false;
  if (!RRplanar_in(end, endTheta))
    return false;

  const float dx = static_cast<float>(end.x - start.x);
  const float dy = static_cast<float>(end.y - start.y);
  const float distance = sqrt(dx * dx + dy * dy);
  if (distance < 1e-6f)
    return true;

  joint q = startTheta;
  const float step = 0.1f;
  for (float s = step; s < distance; s += step) {
    const float t = s / distance;
    point p;
    p.x = static_cast<int>(lround(start.x + dx * t));
    p.y = static_cast<int>(lround(start.y + dy * t));
    p.z = start.z;
    if (!RRplanar_in(p, q))
      return false;
  }
  return true;
}

bool DATAA::checkCircle2R(circle &Circle) {
  if (Circle.radius <= 0)
    return false;

  const float circumference = 2.0f * PI * static_cast<float>(Circle.radius);
  int samples = static_cast<int>(ceil(circumference / 0.1f));
  if (samples < 8)
    samples = 8;
  if (samples > 720)
    samples = 720;

  point start;
  start.x = Circle.p.x + Circle.radius;
  start.y = Circle.p.y;
  start.z = 0;
  joint q = {0.0f, 0.0f};
  if (!RRplanar_in(start, q))
    return false;

  for (int i = 1; i < samples; ++i) {
    const float angle = 2.0f * PI * static_cast<float>(i) / samples;
    point p;
    p.x = Circle.p.x + static_cast<int>(lround(Circle.radius * cos(angle)));
    p.y = Circle.p.y + static_cast<int>(lround(Circle.radius * sin(angle)));
    p.z = 0;
    if (!RRplanar_in(p, q))
      return false;
  }
  return true;
}

bool DATAA::planLine(point start, point end, int16_t *counts, int maxSamples,
                     int &sampleCount, float &duration) {
  sampleCount = 0;
  duration = 0.0f;
  if (counts == nullptr || maxSamples < 2)
    return false;

  const float dx = static_cast<float>(end.x - start.x);
  const float dy = static_cast<float>(end.y - start.y);
  const float dz = static_cast<float>(end.z - start.z);
  const float distance = sqrt(dx * dx + dy * dy);

  joint q = {0.0f, 0.0f};
  if (!RRplanar_in(start, q))
    return false;

  int samples;
  if (distance < 1e-6f) {
    // start == end: a single hold sample, no division by zero.
    samples = 1;
  } else {
    const float step = 0.1f;
    samples = static_cast<int>(ceil(distance / step)) + 1;
    if (samples > maxSamples)
      samples = maxSamples;
    if (samples < 2)
      samples = 2;
  }

  joint previous = q;
  float travel1 = 0.0f;
  float travel2 = 0.0f;
  for (int i = 0; i < samples; ++i) {
    const float t = (samples == 1) ? 0.0f
                                   : static_cast<float>(i) /
                                         static_cast<float>(samples - 1);
    point p;
    p.x = static_cast<int>(lround(start.x + dx * t));
    p.y = static_cast<int>(lround(start.y + dy * t));
    p.z = static_cast<int>(lround(start.z + dz * t));

    joint qi = previous;
    if (!RRplanar_in(p, qi))
      return false;

    if (i > 0) {
      const float d1 = fabs(qi.theta1 - previous.theta1);
      const float d2 = fabs(qi.theta2 - previous.theta2);
      if (d1 > 1.0f || d2 > 1.0f)
        return false; // discontinuous IK branch
      travel1 += d1;
      travel2 += d2;
    }
    jointToCounts(qi, counts[2 * i], counts[2 * i + 1]);
    previous = qi;
  }

  sampleCount = samples;
  duration = std::max(static_cast<float>(travel1 * TIME1_PER_90 / (PI / 2.0f)),
                      static_cast<float>(travel2 * TIME2_PER_90 / (PI / 2.0f)));
  if (duration <= 1e-6f)
    duration = 0.2f;
  return true;
}

bool DATAA::planCircle(circle Circle, int16_t *counts, int maxSamples,
                       int &sampleCount, float &duration) {
  sampleCount = 0;
  duration = 0.0f;
  if (counts == nullptr || maxSamples < 8 || Circle.radius <= 0)
    return false;

  const float circumference = 2.0f * PI * static_cast<float>(Circle.radius);
  int samples = static_cast<int>(ceil(circumference / 0.1f));
  if (samples < 8)
    samples = 8;
  if (samples > maxSamples - 1)
    samples = maxSamples - 1; // n+1 samples including the closing point
  if (samples < 8)
    return false;

  point start;
  start.x = Circle.p.x + Circle.radius;
  start.y = Circle.p.y;
  start.z = 0;
  joint previous = {0.0f, 0.0f};
  if (!RRplanar_in(start, previous))
    return false;

  float travel1 = 0.0f;
  float travel2 = 0.0f;
  for (int i = 0; i <= samples; ++i) {
    const float angle =
        2.0f * PI * static_cast<float>(i) / static_cast<float>(samples);
    point p;
    p.x = Circle.p.x + static_cast<int>(lround(Circle.radius * cos(angle)));
    p.y = Circle.p.y + static_cast<int>(lround(Circle.radius * sin(angle)));
    p.z = 0;

    joint qi = previous;
    if (!RRplanar_in(p, qi))
      return false;

    if (i > 0) {
      const float d1 = fabs(qi.theta1 - previous.theta1);
      const float d2 = fabs(qi.theta2 - previous.theta2);
      if (d1 > 1.0f || d2 > 1.0f)
        return false; // discontinuous IK branch
      travel1 += d1;
      travel2 += d2;
    }
    jointToCounts(qi, counts[2 * i], counts[2 * i + 1]);
    previous = qi;
  }

  sampleCount = samples + 1;
  duration = std::max(static_cast<float>(travel1 * TIME1_PER_90 / (PI / 2.0f)),
                      static_cast<float>(travel2 * TIME2_PER_90 / (PI / 2.0f)));
  if (duration <= 1e-6f)
    duration = 0.2f;
  return true;
}

void PROCESSING::setPID(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

float PROCESSING::PIDcalc(float error) {
  unsigned long now = millis();
  float dt = (now - last_time) * 1e-3f;

  if (dt <= 0.0f)
    return 0.0f;

  integral += error * dt;
  float derivative = (error - pre_error) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;

  pre_error = error;
  last_time = now;

  return output;
}

float DATAA::quinticS(float u) {
  return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

float DATAA::quinticSd(float u, float T) {
  if (T <= 0.0f)
    return 0.0f;
  const float num = 30.0f * u * u - 60.0f * u * u * u + 30.0f * u * u * u * u;
  return num / T;
}

float DATAA::quinticSdd(float u, float T) {
  if (T <= 0.0f)
    return 0.0f;
  const float num = 60.0f * u - 180.0f * u * u + 120.0f * u * u * u;
  return num / (T * T);
}

Torque DATAA::inverseDynamics(const joint &Theta, const Torque &dq,
                              const Torque &ddq, float gravity) {
  const float m1 = 0.1362f, m2 = 0.082f;
  const float L1 = 0.20f, L2 = 0.15f;
  const float b1 = 0.04f, b2 = 0.03f;
  const float r1 = 0.5f * L1, r2 = 0.5f * L2;

  const float I1c = m1 * (L1 * L1 + b1 * b1) / 12.0f;
  const float I2c = m2 * (L2 * L2 + b2 * b2) / 12.0f;
  const float I1O = I1c + m1 * r1 * r1;
  const float I2O = I2c + m2 * r2 * r2;
  const float a = m2 * L1 * r2;

  const float c2 = cosf(Theta.theta2);
  const float s2 = sinf(Theta.theta2);
  const float c1 = cosf(Theta.theta1);
  const float c12 = cosf(Theta.theta1 + Theta.theta2);

  const float M11 = I1O + m2 * L1 * L1 + I2O + 2.0f * a * c2;
  const float M12 = I2O + a * c2;
  const float M22 = I2O;

  const float C1 = -a * s2 * (2.0f * dq.t1 * dq.t2 + dq.t2 * dq.t2);
  const float C2 = a * s2 * dq.t1 * dq.t1;

  const float G1 = (m1 * r1 + m2 * L1) * gravity * c1 + m2 * r2 * gravity * c12;
  const float G2 = m2 * r2 * gravity * c12;

  Torque out;
  out.t1 = M11 * ddq.t1 + M12 * ddq.t2 + C1 + G1;
  out.t2 = M12 * ddq.t1 + M22 * ddq.t2 + C2 + G2;

  return out;
}

Torque DATAA::inverseDynamics(const joint &Theta, const Torque &dq,
                              const Torque &ddq) {
  return inverseDynamics(Theta, dq, ddq, 9.81f);
}
