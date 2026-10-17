#include "MControl.h"

// link_length is expressed in the same unit as the Command coordinates used by
// the production HMI and by Calculate_Lib (centimetres).  Keeping metres here
// made RRplanar_in reject every real command because int coordinates cannot
// represent a sub-metre reach.
const float mass_link[2] = {0.1362, 0.0826}, link_length[] = {20.0f, 15.0f},
            center_length[2] = {0.1, 0.075};

// point operator*(float k, point p) { return {k * p.x, k * p.y, k * p.z}; }
// joint operator*(float k, joint q) { return {k * q.theta1, k * q.theta2}; }

/*
void INPUT_DATA ::scan(char *line, byte buffer[], byte *length,
                       boolean *newData) {
  static byte idx = 0;
  static byte cursor = 0;
  static bool recv = false;

  const byte MAX_LEN = 50;

  while (Serial.available()) {

    char c = Serial.read();

    // START LINE
    if (!recv) {
      recv = true;
      idx = 0;
      cursor = 0;
      line[0] = '\0';
      Serial.print(">   ");
    }

    // ARROW KEY
    if (c == 27) {
      while (!Serial.available())
        ;
      char c1 = Serial.read();

      if (c1 == '[') {
        while (!Serial.available())
          ;
        char c2 = Serial.read();

        // LEFT
        if (c2 == 'D') {
          if (cursor > 0) {
            cursor--;
            Serial.print("\b");
          }
        }

        // RIGHT
        else if (c2 == 'C') {
          if (cursor < idx) {
            Serial.print(line[cursor]);
            cursor++;
          }
        }
      }
    }

    // ENTER
    else if (c == '\r' || c == '\n') {
      if (idx == 0) {
        recv = false;
        continue;
      }

      line[idx] = '\0';
      Serial.println();

      parseData(line, buffer, newData);
      *newData = true;

      recv = false;
      idx = 0;
      cursor = 0;
    }

    // BACKSPACE
    else if (c == 8 || c == 127) {
      if (cursor > 0) {

        for (byte i = cursor - 1; i < idx; i++)
          line[i] = line[i + 1];

        idx--;
        cursor--;

        Serial.print("\b");
        for (byte i = cursor; i < idx; i++)
          Serial.print(line[i]);

        Serial.print(" ");
        for (byte i = cursor; i <= idx; i++)
          Serial.print("\b");
      }
    }

    // NORMAL CHAR
    else {
      // sửa lỗi chỉ nhập 1 ký tự: không dùng sizeof(line)
      if (idx < MAX_LEN - 1) {

        for (byte i = idx; i > cursor; i--)
          line[i] = line[i - 1];

        line[cursor] = c;
        idx++;
        cursor++;
        line[idx] = '\0';

        for (byte i = cursor - 1; i < idx; i++)
          Serial.print(line[i]);
        for (byte i = cursor; i < idx; i++)
          Serial.print("\b");
      }
    }
  }
}

void INPUT_DATA ::input_coor(byte buffer[], byte *length, boolean *newData) {

  static char temp[20]; // input <x,y>
  static byte idx = 0;
  static boolean recv = false;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '<') {
      recv = true;
      idx = 0;
      Serial.print(c);
    }

    else if (c == '>' && recv) {
      temp[idx] = '\0';
      char *comma = strchr(temp, ','); // tách chuỗi tại dấu phẩy

      if (comma != NULL) {

        *comma = '\0';           // cắt chuỗi
        int x = atoi(temp);      // phần trước dấu ,
        int y = atoi(comma + 1); // phần sau dấu ,

        // lưu x
        buffer[0] = x & 0xFF;
        buffer[1] = (x >> 8) & 0xFF;

        // lưu y
        buffer[2] = y & 0xFF;
        buffer[3] = (y >> 8) & 0xFF;

        *length = 4;
        *newData = true;
      }
      recv = false;
      Serial.println(c);
    }

    else if (recv) {
      if (c == 8 || c == 127) { // backspace
        if (idx > 0) {
          idx--;
          Serial.print("\b \b");
        }
      } else if ((c >= '0' && c <= '9') || c == '-' || c == ',') {
        if (idx < sizeof(temp) - 1) {
          temp[idx++] = c;
          Serial.print(c);
        }
      }
    }
  }
}

void INPUT_DATA ::input_angle(byte buffer[], byte *length, boolean *newData) {
  static char temp[12]; // lưu chuỗi tạm
  static byte idx = 0;
  static boolean recv = false;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') {
      recv = true;
      idx = 0;
      Serial.print(c);
    }

    else if (c == '>' && recv) {
      temp[idx] = '\0';

      int value = atoi(temp); // đổi sang số

      // lưu dạng binary 16-bit
      buffer[0] = value & 0xFF;        // LSB
      buffer[1] = (value >> 8) & 0xFF; // MSB
      *length = 2;

      *newData = true;
      recv = false;

      Serial.println(c);
    }

    else if (recv) {
      if (c == 8 || c == 127) { // backspace
        if (idx > 0) {
          idx--;
          Serial.print("\b \b");
        }
      } else if ((c >= '0' && c <= '9') || c == '-') {
        if (idx < sizeof(temp) - 1) {
          temp[idx++] = c;
          Serial.print(c);
        }
      }
    }
  }
}
*/

void INPUT_DATA ::recvByte(char *line, byte buffer[], boolean *newData) {
  static byte idx = 0;
  const byte MAX_LEN = 50;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      if (idx == 0)
        return;
      line[idx] = '\0';
      parseData(line, buffer, newData);
      idx = 0;
      *newData = true;
    } else {
      if (idx < MAX_LEN - 1)
        line[idx++] = c;
    }
  }
}

// parse Data
void INPUT_DATA ::parseData(char *line, byte buffer[], boolean *newData) {
  char cmd[10];

  if (sscanf(line, "%9s", cmd) != 1)
    buffer[0] = (byte)NONE;

  else if (strcmp(cmd, "HOME") == 0) {
    buffer[0] = (byte)HOME;
    *newData = true;
  }

  else if (strcmp(cmd, "POINT") == 0) {
    buffer[0] = (byte)POINT;
    parsePoint(line, buffer, newData);
  }

  else if (strcmp(cmd, "LINE") == 0) {
    buffer[0] = (byte)LINE;
    parseLine(line, buffer, newData);
  }

  else if (strcmp(cmd, "CIRCLE") == 0) {
    buffer[0] = (byte)CIRCLE;
    parseCircle(line, buffer, newData);
  }

  else {
    buffer[0] = (byte)NONE;
    Serial.println("INVALID COMMAND, INPUT AGAIN !");
  }
}

bool INPUT_DATA::parseCommand(char *line, Command &cmd) {
  char keyword[10];

  if (sscanf(line, "%9s", keyword) != 1)
    return 0;

  if (strcmp(keyword, "HOME") == 0) {
    cmd.type = HOME;
    return 1;
  }

  if (strcmp(keyword, "POINT") == 0) {
    int x, y, z;
    if (sscanf(line, "%*s %d %d %d", &x, &y, &z) < 3)
      return 0;

    cmd.type = POINT;
    cmd.p1 = {x, y, z};
    return 1;
  }

  if (strcmp(keyword, "LINE") == 0) {
    int x1, y1, z1, x2, y2, z2;
    if (sscanf(line, "%*s %d %d %d %*s %d %d %d", &x1, &y1, &z1, &x2, &y2,
               &z2) < 6)
      return 0;

    cmd.type = LINE;
    cmd.p1 = {x1, y1, z1};
    cmd.p2 = {x2, y2, z2};
    return 1;
  }

  if (strcmp(keyword, "CIRCLE") == 0) {
    float x, y, z, r;
    if (sscanf(line, "%*s %f %f %f %f", &x, &y, &z, &r) < 4)
      return 0;

    cmd.type = CIRCLE;
    cmd.p1 = {(int)x, (int)y, (int)z};
    cmd.radius = r;
    return 1;
  }

  return 0;
}

void INPUT_DATA::parseQueue(char *line) {
  byte tempBuffer[20];
  byte len = 0;
  boolean newData = false;

  parseData(line, tempBuffer, &newData);

  if (newData) {
    if (!enqueueBuffer(tempBuffer, len)) {
      Serial.println("QUEUE FULL!");
    } else {
      Serial.println("CMD ADDED TO QUEUE");
    }
  }
}

void INPUT_DATA ::parsePoint(char *line, byte buffer[], boolean *newData) {
  // Serial.println ("INPUT THE TARGET COORDINATE: ");

  int x, y, z;
  if (sscanf(line, "%*s %d %d %d", &x, &y, &z) < 3)
    return;

  // x value
  buffer[1] = x & 0xFF;
  buffer[2] = (x >> 8) & 0xFF;

  // y value
  buffer[3] = y & 0xFF;
  buffer[4] = (y >> 8) & 0xFF;

  // z value
  buffer[5] = z & 0xFF;
  buffer[6] = (z >> 8) & 0xFF;

  *newData = true;
}

void INPUT_DATA ::parseLine(char *line, byte buffer[], boolean *newData) {
  // Serial.println ("INPUT THE START AND THE END: x y z TO x y z: ");

  int x1, y1, z1, x2, y2, z2;
  if (sscanf(line, "%*s %d %d %d %*s %d %d %d", &x1, &y1, &z1, &x2, &y2, &z2) <
      6)
    return;

  // x start value
  buffer[1] = x1 & 0xFF;
  buffer[2] = (x1 >> 8) & 0xFF;

  // y start value
  buffer[3] = y1 & 0xFF;
  buffer[4] = (y1 >> 8) & 0xFF;

  // z start value
  buffer[5] = z1 & 0xFF;
  buffer[6] = (z1 >> 8) & 0xFF;

  // x target value
  buffer[7] = x2 & 0xFF;
  buffer[8] = (x2 >> 8) & 0xFF;

  // y target value
  buffer[9] = y2 & 0xFF;
  buffer[10] = (y2 >> 8) & 0xFF;

  // z target value
  buffer[11] = z2 & 0xFF;
  buffer[12] = (z2 >> 8) & 0xFF;

  *newData = true;
}

void INPUT_DATA ::parseCircle(char *line, byte buffer[], boolean *newData) {
  // Serial.println ("INPUT THE CENTRAL COORDINATE AND THE RADIUS: ");

  point center;
  float radius;
  if (sscanf(line, "%*s %f %f %f %f", &center.x, &center.y, &center.z,
             &radius) < 2)
    return;

  int16_t x_fix = (int16_t)(center.x * 100);
  int16_t y_fix = (int16_t)(center.y * 100);
  int16_t r_fix = (int16_t)(radius * 100);

  // center coordinate
  buffer[1] = x_fix & 0xFF;
  buffer[2] = (x_fix >> 8) & 0xFF;

  buffer[3] = y_fix & 0xFF;
  buffer[4] = (y_fix >> 8) & 0xFF;

  // radius value
  buffer[5] = r_fix & 0xFF;
  buffer[6] = (r_fix >> 8) & 0xFF;

  *newData = true;
}

// convert Data
long INPUT_DATA ::convertbyte2enc(unsigned char recvchar[], byte startpos,
                                  float pulse_num) {
  float tmp = recvchar[startpos] + recvchar[startpos + 1] * 256;
  return (int)round(tmp * pulse_num / 360);
}

void INPUT_DATA ::convertbyte2coor(point &p, unsigned char recvchar[],
                                   byte startpos) {

  p.x = recvchar[startpos] + recvchar[startpos + 1] * 256;
  p.y = recvchar[startpos + 2] + recvchar[startpos + 3] * 256;
  p.z = recvchar[startpos + 4] + recvchar[startpos + 5] * 256;
}

circle INPUT_DATA ::convertData2Circle(circle Circle, unsigned char recvchar[],
                                       byte startpos) {
  Circle.p.x = recvchar[startpos] + recvchar[startpos + 1] * 256;
  Circle.p.y = recvchar[startpos + 2] + recvchar[startpos + 3] * 256;
  Circle.radius = recvchar[startpos + 4] + recvchar[startpos + 5] * 256;

  return Circle;
}

// QUEUE COMMAND
bool INPUT_DATA::enqueueCmd(const Command &cmd) {
  int next = (tail + 1) % SIZE;
  if (next == head)
    return 0; // full

  queue[tail] = cmd;
  tail = next;
  return 1;
}

bool INPUT_DATA::dequeueCmd(Command &cmd) {
  if (head == tail)
    return 0; // empty

  cmd = queue[head];
  head = (head + 1) % SIZE;
  return 1;
}

bool INPUT_DATA::isEmpty() { return head == tail; }

bool INPUT_DATA::isFull() { return ((tail + 1) % SIZE) == head; }

bool INPUT_DATA::enqueueBuffer(byte buffer[], byte length) {
  int next = (tail + 1) % SIZE;
  if (next == head)
    return 0; // FULL

  memcpy(Queue[tail].data, buffer, length);
  Queue[tail].len = length;

  tail = next;
  return 1;
}

bool INPUT_DATA::dequeueBuffer(byte buffer[], byte &length) {
  if (head == tail)
    return 0; // empty

  memcpy(buffer, Queue[head].data, Queue[head].len);
  length = Queue[head].len;

  head = (head + 1) % SIZE;
  return 1;
}

// non-liner interpolation
void PROCESSING ::splinePoint(point start, point target, point vector0,
                              point vector1, point &Point, float t) {
  if (t < 0)
    t = 0;
  if (t > 1)
    t = 1;

  float t2 = square(t), t3 = pow(t, 3);

  float h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t,
        h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;

  Point.x = h00 * start.x + h10 * vector0.x + h01 * target.x + h11 * vector1.x;
  Point.y = h00 * start.y + h10 * vector0.y + h01 * target.y + h11 * vector1.y;
  Point.z = h00 * start.z + h10 * vector0.z + h01 * target.z + h11 * vector1.z;
}

void PROCESSING ::splineTheta(joint start, joint target, joint vector0,
                              joint vector1, joint &Joint, float t) {
  if (t < 0)
    t = 0;
  if (t > 1)
    t = 1;

  float t2 = square(t), t3 = pow(t, 3);

  float h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t,
        h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;

  Joint.theta1 = h00 * start.theta1 + h10 * vector0.theta1 +
                 h01 * target.theta1 + h11 * vector1.theta1;
  Joint.theta2 = h00 * start.theta2 + h10 * vector0.theta2 +
                 h01 * target.theta2 + h11 * vector1.theta2;
}

// tạo timescale
float PROCESSING ::interQuintic(float start, float time_sum) {
  float t = (millis() - start) * 1e-3;
  float u = t / time_sum;

  if (u < 0)
    u = 0;
  else if (u > 1)
    u = 1;

  return 10 * pow(u, 3) - 15 * pow(u, 4) + 6 * pow(u, 5);
}

// time_now: thời điểm hiện tại trong quỹ đạo = (t - t0) / T
float PROCESSING ::movePoint(float startMotor, float targetMotor,
                             float time_now) {
  return startMotor + (targetMotor - startMotor) * time_now;
}

void PROCESSING ::moveLine(point start, point target, point &p,
                           float time_now) {

  p.x = start.x + (target.x - start.x) * time_now;
  p.y = start.y + (target.y - start.y) * time_now;
  p.z = start.z + (target.z - start.z) * time_now;
}

point PROCESSING ::projectionCircle(circle &Circle, point currentPos) {
  float dx = currentPos.x - Circle.p.x;
  float dy = currentPos.y - Circle.p.y;
  float dist = sqrt(dx * dx + dy * dy);

  if (dist <= 1e-2f)
    return {Circle.p.x + Circle.radius, Circle.p.y, currentPos.z};

  float scale = Circle.radius / dist;

  return {(int)(Circle.p.x + dx * scale), (int)(Circle.p.y + dy * scale),
          currentPos.z};
}

void PROCESSING ::moveCircle(circle Circle, point &p, float time_now) {
  float ang = 2.0f * PI * time_now;

  p.x = Circle.p.x + Circle.radius * cos(ang);
  p.y = Circle.p.y + Circle.radius * sin(ang);
  // p.z = currentPos.z;
}

void PROCESSING ::emergencyStop(byte stopPin) {
  (void)stopPin;
  // BTS7960 drives the motor through the two Veloc_A/Veloc_B PWM inputs; the
  // old TB6612 dir_A/dir_B/Veloc fields are not wired for this hardware.
  // running(0, pow(2, PWM_RESOLUTION));
}

void PROCESSING ::homing(byte homePin, boolean activeState, boolean *isHoming) {
  if (digitalRead(homePin) == activeState) {
    // Drive both BTS7960 inputs low when the home switch is reached.
    // running(0, pow(2, PWM_RESOLUTION));
    Serial.println("HOMING IS DONE");
    *isHoming = false;
    return;
  }

  // Move toward the home switch using the same BTS7960 input mapping as the
  // closed-loop running() path (negative command = Veloc_B PWM).
  // running(-80, pow(2, PWM_RESOLUTION));
}

// calculate & execute function
void PROCESSING ::count(byte shift) {
  state_2bit = (digitalRead(read_A) << 1) | digitalRead(read_B);
  // state_2bit = (PIND >> shift) & 0b11;
  state_4bit = (last_state << 2) | state_2bit;

  switch (state_4bit) {
  case 0b0010:
  case 0b1011:
  case 0b1101:
  case 0b0100:
    count_enc++;
    break;
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    count_enc--;
  }

  last_state = state_2bit;
}

/*void PROCESSING ::running(float pwm) {
  int ctrl = (int)round(pwm);

  if (ctrl > 0) {
    if (pwm > 255) {
      digitalWrite(dir_A, HIGH);
      digitalWrite(dir_B, LOW);
      ledcWrite(Veloc, 255);
    } else {
      digitalWrite(dir_A, HIGH);
      digitalWrite(dir_B, LOW);
      ledcWrite(Veloc, ctrl);
    }

  } else if (ctrl < 0) {
    if (pwm < -255) {
      digitalWrite(dir_A, LOW);
      digitalWrite(dir_B, HIGH);
      ledcWrite(Veloc, 255);
    } else {
      digitalWrite(dir_A, LOW);
      digitalWrite(dir_B, HIGH);
      ledcWrite(Veloc, abs(ctrl));
    }
  } else {
    digitalWrite(dir_A, LOW);
    digitalWrite(dir_B, LOW);
    ledcWrite(Veloc, 0);
  }
}*/

void PROCESSING ::running(float pwm, int resolution) {
  if (!isfinite(pwm)) {
    analogWrite(Veloc_A, 0);
    analogWrite(Veloc_B, 0);
    return;
  }

  int ctrl = (int)roundf(pwm);

  if (ctrl > 0) {
    if (ctrl > resolution - 1)
      ctrl = resolution - 1;
    analogWrite(Veloc_A, ctrl);
    analogWrite(Veloc_B, 0);
  }

  else if (ctrl < 0) {
    if (ctrl < -(resolution - 1))
      ctrl = -(resolution - 1);
    analogWrite(Veloc_A, 0);
    analogWrite(Veloc_B, -ctrl);
  }

  else {
    analogWrite(Veloc_A, 0);
    analogWrite(Veloc_B, 0);
  }
}

void PROCESSING ::setPID(float kp, float ki, float kd) {
  Kp = kp, Ki = ki, Kd = kd;
}

float PROCESSING ::PIDcalc(float error) {
  unsigned long now = millis();
  float dt = (now - last_time) * (1e-3);

  if (dt <= 0)
    return 0;

  integral += error * dt;
  float derivative = (error - pre_error) / dt;

  output = Kp * error + Ki * integral + Kd * derivative;

  pre_error = error;
  last_time = now;

  return output;
}

void PROCESSING ::resetPID() {
  integral = 0;
  pre_error = 0;
  output = 0;
  // Seed the timestamp so the next PIDcalc() sees a small, realistic dt instead
  // of the whole time spent in the settled/hold state.
  last_time = millis();
}

Torque PROCESSING ::MassMatrix(joint Theta) {
  masstorque.t1 =
      mass_link[0] * pow(center_length[0], 2) +
      mass_link[1] *
          (pow(center_length[0], 2) + pow(center_length[1], 2) +
           2 * center_length[0] * center_length[1] * cos(Theta.theta2)
           /* I1 + I2*/) +
      mass_link[1] *
          (pow(center_length[1], 2) +
           center_length[0] * center_length[1] * cos(Theta.theta2)) /*+I2*/;

  masstorque.t2 = mass_link[1] * (pow(center_length[1], 2) +
                                  center_length[0] * center_length[1] *
                                      cos(Theta.theta2)) /*+I2*/
                  + mass_link[1] * pow(center_length[1], 2);
  return masstorque;
}

Torque PROCESSING ::CoriolisMatrix(joint Theta, Torque dq) {
  cori.t1 = -mass_link[1] * center_length[0] * center_length[1] *
            sin(Theta.theta2) * (2 * dq.t1 * dq.t2 + pow(dq.t2, 2));

  cori.t2 = mass_link[1] * center_length[0] * center_length[1] *
            sin(Theta.theta2) * pow(dq.t1, 2);

  return cori;
}

Torque PROCESSING ::GravityMatrix(joint Theta) {
  grav.t1 =
      (mass_link[0] + mass_link[1]) * center_length[0] * 9.78 *
          cos(Theta.theta1) +
      mass_link[1] * center_length[1] * 9.78 * cos(Theta.theta1 + Theta.theta2);

  grav.t2 =
      mass_link[1] * 9.78 * center_length[1] * cos(Theta.theta1 + Theta.theta2);

  return grav;
}

Torque PROCESSING ::Torquecalc(joint Theta, Torque dq, Torque ddq) {
  torque.t1 = MassMatrix(Theta).t1 * ddq.t1 + CoriolisMatrix(Theta, dq).t1 +
              GravityMatrix(Theta).t1;
  torque.t2 = MassMatrix(Theta).t2 * ddq.t1 + CoriolisMatrix(Theta, dq).t2 +
              GravityMatrix(Theta).t2;
  return torque;
}

bool KINEMATICS ::RRplanar_in(point &POINT, joint &theta) {
  float bestCost = 1e9;
  boolean found = false;

  joint opt1, opt2, opt3, opt4;
  boolean valid1 = false, valid2 = false, valid3 = false, valid4 = false;

  float x = POINT.x, y = POINT.y, l1 = link_length[0], l2 = link_length[1],
        r = sqrt(x * x + y * y);
  if (r > l1 + l2 || r < fabs(l1 - l2))
    return 0;

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

  /*
  if (x < 0) {
    if (tempTheta1_11 < PI/2) valid1 = false;
    if (tempTheta1_12 < PI/2) valid2 = false;
    if (tempTheta1_21 < PI/2) valid3 = false;
    if (tempTheta1_22 < PI/2) valid4 = false;
  }
  else if ( x > 0) {
    if (tempTheta1_11 > PI/2) valid1 = false;
    if (tempTheta1_12 > PI/2) valid2 = false;
    if (tempTheta1_21 > PI/2) valid3 = false;
    if (tempTheta1_22 > PI/2) valid4 = false;
  }
  */

  if (valid1 && !fk_Match(opt1, POINT))
    valid1 = false;
  if (valid2 && !fk_Match(opt2, POINT))
    valid2 = false;
  if (valid3 && !fk_Match(opt3, POINT))
    valid3 = false;
  if (valid4 && !fk_Match(opt4, POINT))
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

  // Serial.print("POINT:");
  // Serial.print(POINT.x);
  // Serial.print(",");
  // Serial.println(POINT.y);

  return found;
}

bool KINEMATICS ::fk_Match(joint &theta, point &Point) {
  float q1 = theta.theta1, q2 = theta.theta2, l1 = link_length[0],
        l2 = link_length[1];

  float fx = l1 * cos(q1) + l2 * cos(q1 + q2);
  float fy = l1 * sin(q1) + l2 * sin(q1 + q2);

  if (fabs(fx - Point.x) <= 1e-2 && fabs(fy - Point.y) <= 1e-2)
    return 1;
  else
    return 0;
}

bool KINEMATICS ::checkLine2R(point &start, point &end, joint &startTheta,
                              joint &endTheta) {
  float d = sqrt(square(end.x - start.x) + square(end.y - start.y));

  if (RRplanar_in(start, startTheta) && RRplanar_in(end, endTheta)) {

    point p;
    joint q = startTheta;

    for (float i = 0; i <= d; i += 0.1) {
      float t = i / d;

      p.x = start.x + t * (end.x - start.x);
      p.y = start.y + t * (end.y - start.y);
      p.z = start.z + t * (end.z - start.z);

      if (!RRplanar_in(p, q))
        return 0;
    }
    return 1;
  }
  return 0;
}

bool KINEMATICS::checkCircle2R(circle &c /*, joint &refTheta*/) {
  const int N = 720;
  joint tempTheta;

  for (int i = 0; i < N; i++) {
    float ang = 2.0f * PI * ((float)i / N);

    point p;
    p.x = c.p.x + c.radius * cos(ang);
    p.y = c.p.y + c.radius * sin(ang);
    p.z = 0;

    if (!RRplanar_in(p, tempTheta))
      return 0;
  }

  // lấy reference pose (entry)
  // point entry;
  // entry.x = c.p.x + c.radius;
  // entry.y = c.p.y;
  // entry.z = 0;

  // RRplanar_in(entry, link_length, refTheta);
  return true;
}

// tính toán vận tốc và góc va chạm để chọn đường đi
