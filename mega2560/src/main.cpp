#include "driver/pulse_cnt.h"
#include <Arduino.h>
#include <MControl.h>

#define HOME_SWITCHES_WIRED 0
#define HOME1 GPIO_NUM_25
#define HOME2 GPIO_NUM_26

#define PHYSICAL_STOP_BUTTON_WIRED 0
#define STOP GPIO_NUM_27

// Motor 1
#define Veloc1_A GPIO_NUM_23
#define Veloc1_B GPIO_NUM_19
#define Veloc1_A_EN GPIO_NUM_21
#define Veloc1_B_EN GPIO_NUM_22
#define ENC_A GPIO_NUM_32
#define ENC_B GPIO_NUM_33

// Motor 2
#define Veloc2_A GPIO_NUM_18
#define Veloc2_B GPIO_NUM_15
#define Veloc2_A_EN GPIO_NUM_5
#define Veloc2_B_EN GPIO_NUM_4
#define ENC_C GPIO_NUM_34
#define ENC_D GPIO_NUM_35

// Encoder scale
#define PWM_NUM1 11880 // 33 pulse / degree
#define PWM_NUM2 1980  // 5.5 pulse / degree

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define SERIAL_BAUD 921600
#define SERIAL_TIMEOUT_MS 500

// Debug flag
#define DEBUG_SERIAL 0
#define NO_SENSOR_TEST_MODE 0

// Telemetry period
#define TELEMETRY_PERIOD_MS 10
#define POS_TOLERANCE_2 4
#define DISTURB_THRESHOLD_2 20 //
#define SETTLE_TIME_MS 120
#define MAX_PLAUSIBLE_ERROR_COUNTS 6000

// PC -> ESP32 dynamics torque field: signed int16, 1 LSB = 0.001 N*m.
#define TORQUE_LSB_NM 0.001f

// feedforward gains.
#define TAU_SCALE_1_NM 1.382f
#define TAU_SCALE_2_NM 0.205f

// motor 1.
#define MOTOR2_INVERT_DRIVE 0

static const uint8_t UART_RX_PIN = GPIO_NUM_3;
static const uint8_t UART_TX_PIN = GPIO_NUM_1;

static const uint8_t TELEMETRY_HEADER_1 = 0x55;
static const uint8_t TELEMETRY_HEADER_2 = 0xAA;
static const uint8_t TELEMETRY_FRAME_SIZE = 7;

byte shift1 = 2;
byte shift2 = 0;

INPUT_DATA indata;

PROCESSING encode1(ENC_A, ENC_B, Veloc1_A, Veloc1_B);
PROCESSING encode2(ENC_C, ENC_D, Veloc2_A, Veloc2_B);

// Drive motor 2 with the confirmed polarity (see MOTOR2_INVERT_DRIVE).  Kept in
// one place so the target-reached/hold/recovery paths all share the mapping.
static inline void driveMotor2(float pwm) {
#if MOTOR2_INVERT_DRIVE
  encode2.running(-pwm, pow(2, PWM_RESOLUTION));
#else
  encode2.running(pwm, pow(2, PWM_RESOLUTION));
#endif
}

// Current and desired joint angles
joint Theta = {0};
joint desiredTheta = {0};

float error_interpolate1 = 0.0f;
float error_interpolate2 = 0.0f;

float speed1 = 0.0f;
float speed2 = 0.0f;

volatile float tauDyn1Nm = 0.0f;
volatile float tauDyn2Nm = 0.0f;

volatile int16_t pwmNorm1 = 0;
volatile int16_t pwmNorm2 = 0;

// Communication diagnostics: additive-checksum failures since boot.
volatile uint32_t checksumErrors = 0;
static int16_t normalizeTorquePwm(float tauNm, float scaleNm) {
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

// Motor 2 position state machine.  The HMI streams the moving reference; this
// only decides when to actively drive, when to hold at zero output, and when to
// re-engage after the shaft is disturbed away from the retained target.
enum Motor2State { M2_TRACKING, M2_HOLDING, M2_RECOVERING, M2_STOPPED };
Motor2State motor2State = M2_TRACKING;
int16_t motor2Reference = 0;
bool motor2Settling = false;
unsigned long motor2SettleStart = 0;

boolean isStop = false, isHoming1 = false, isHoming2 = false,
        receivedFirstPacket = false, isTimeout = false;

// Remote (UART) control state: latched emergency stop, control-frame marker.
volatile bool remoteStop = false;
bool controlFrame = false;

// Status frames (55 AB ...) are queued by the UART task and emitted by the
// control task, so both tasks never write to UART0 concurrently.
volatile uint8_t pendingStatusCode = 0;
volatile bool pendingStatusFlag = false;

// Status codes reported to the HMI (55 AB CODE 00 00 00 CHECKSUM).
static const uint8_t STATUS_STOPPED = 0x50; // STOP latched
static const uint8_t STATUS_HOME_UNAVAILABLE =
    0x51;                                  // homing switches not wired
static const uint8_t STATUS_HOMING = 0x52; // homing active
static const uint8_t STATUS_HOMED = 0x53;  // homing finished

PACKET_RX_STATE rxState = WAIT_HEADER_1;
BinaryPacket currentPacket;

unsigned long lastPacketTime = 0;
unsigned long lastRxByteTime = 0;
unsigned long lastTelemetryTime = 0;

// Desired encoder position received from PC.
// These are shared between communicationTask and controlTask.
volatile int16_t desiredJoint1Count = 0;
volatile int16_t desiredJoint2Count = 0;

// Protect shared communication state
portMUX_TYPE sharedStateMux = portMUX_INITIALIZER_UNLOCKED;

// Encoder 1
pcnt_unit_handle_t pcnt_unit_1 = NULL;
pcnt_channel_handle_t pcnt_chan_1A = NULL;
pcnt_channel_handle_t pcnt_chan_1B = NULL;

// Encoder 2
pcnt_unit_handle_t pcnt_unit_2 = NULL;
pcnt_channel_handle_t pcnt_chan_2A = NULL;
pcnt_channel_handle_t pcnt_chan_2B = NULL;

// PCNT stores raw counts; motor 2 keeps the existing home-position offset in
// software.
int32_t encoder_offset_1 = 0;
int32_t encoder_offset_2 = PWM_NUM2 / 2;

// ENCODER 1 INITIALIZATION
void init_encoder_1() {
  // Configure PCNT Unit 1
  pcnt_unit_config_t unit_config = {
      .low_limit = -32000,
      .high_limit = 32000,
  };

  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_1));

  // Hardware glitch filter
  pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 1000};

  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_1, &filter_config));

  // Encoder A channel
  pcnt_chan_config_t chan_A_config = {.edge_gpio_num = ENC_A,
                                      .level_gpio_num = ENC_B};

  // Encoder B channel
  pcnt_chan_config_t chan_B_config = {.edge_gpio_num = ENC_B,
                                      .level_gpio_num = ENC_A};

  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_1, &chan_A_config, &pcnt_chan_1A));

  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_1, &chan_B_config, &pcnt_chan_1B));

  // Channel A
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      pcnt_chan_1A, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE));

  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
      pcnt_chan_1A, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // Channel B
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      pcnt_chan_1B, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE));

  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
      pcnt_chan_1B, PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP));

  // Enable and start PCNT
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_1));

  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_1));
}

void init_encoder_2() {
  // Configure PCNT Unit 2
  pcnt_unit_config_t unit_config = {
      .low_limit = -32000,
      .high_limit = 32000,
  };

  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_2));

  // Hardware glitch filter
  pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 1000};

  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_2, &filter_config));

  // Encoder A channel
  pcnt_chan_config_t chan_A_config = {.edge_gpio_num = ENC_C,
                                      .level_gpio_num = ENC_D};

  // Encoder B channel
  pcnt_chan_config_t chan_B_config = {.edge_gpio_num = ENC_D,
                                      .level_gpio_num = ENC_C};

  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_2, &chan_A_config, &pcnt_chan_2A));
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_2, &chan_B_config, &pcnt_chan_2B));

  // Channel A
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      pcnt_chan_2A, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE));

  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
      pcnt_chan_2A, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // Channel B
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      pcnt_chan_2B, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE));

  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
      pcnt_chan_2B, PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP));

  // Enable and start PCNT
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_2));

  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_2));
}

void updateEncoderCounts() {

#if NO_SENSOR_TEST_MODE
  encode1.count_enc = 0;
  encode2.count_enc = 0;
#else
  int actualJoint1Count = 0;
  int actualJoint2Count = 0;

  ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_1, &actualJoint1Count));
  ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_2, &actualJoint2Count));

  encode1.count_enc = actualJoint1Count + encoder_offset_1;
  encode2.count_enc = actualJoint2Count + encoder_offset_2;
#endif
}

void setDesiredCounts(int16_t desiredJoint1, int16_t desiredJoint2) {

  portENTER_CRITICAL(&sharedStateMux);

  desiredJoint1Count = desiredJoint1;
  desiredJoint2Count = desiredJoint2;

  lastPacketTime = millis();
  receivedFirstPacket = true;

  portEXIT_CRITICAL(&sharedStateMux);
}

void getDesiredCounts(int16_t &desiredJoint1, int16_t &desiredJoint2) {

  portENTER_CRITICAL(&sharedStateMux);

  desiredJoint1 = desiredJoint1Count;
  desiredJoint2 = desiredJoint2Count;

  portEXIT_CRITICAL(&sharedStateMux);
}
// void inline ISR1() { encode1.count(shift1); }
// void inline ISR2() { encode2.count(shift2); }

// ENCODER TELEMETRY
void sendEncoderTelemetry() {

  unsigned long currentTime = millis();

  if ((unsigned long)(currentTime - lastTelemetryTime) < TELEMETRY_PERIOD_MS)
    return;

  lastTelemetryTime = currentTime;

  // Local snapshot variables.
  // These are NOT global variables.
  int16_t actualJoint1Count = 0;
  int16_t actualJoint2Count = 0;

  noInterrupts();

  actualJoint1Count = static_cast<int16_t>(encode1.count_enc);
  actualJoint2Count = static_cast<int16_t>(encode2.count_enc);

  interrupts();

  // Build telemetry packet
  uint8_t packet[TELEMETRY_FRAME_SIZE] = {

      TELEMETRY_HEADER_1,
      TELEMETRY_HEADER_2,

      static_cast<uint8_t>((actualJoint1Count >> 8) & 0xFF),
      static_cast<uint8_t>(actualJoint1Count & 0xFF),
      static_cast<uint8_t>((actualJoint2Count >> 8) & 0xFF),
      static_cast<uint8_t>(actualJoint2Count & 0xFF),

      0x00};

  // Calculate checksum
  packet[6] =
      static_cast<uint8_t>(packet[2] + packet[3] + packet[4] + packet[5]);

  // Send only when UART has enough free space
  if (Serial.availableForWrite() >= TELEMETRY_FRAME_SIZE) {

    Serial.write(packet, TELEMETRY_FRAME_SIZE);
  }
}

static void writeStatusFrame(uint8_t code) {
  uint8_t frame[7] = {0x55, 0xAB, code, 0x00, 0x00, 0x00, 0x00};
  frame[6] = static_cast<uint8_t>(frame[2] + frame[3] + frame[4] + frame[5]);
  Serial.write(frame, sizeof(frame));
}

static inline void queueStatus(uint8_t code) {
  pendingStatusCode = code;
  pendingStatusFlag = true;
}

void receiveBinaryPacket() {
  while (Serial.available() > 0) {
    uint8_t receivedByte = Serial.read();
    lastRxByteTime = millis();

    switch (rxState) {

    case WAIT_HEADER_1:
      if (receivedByte == 0xAA)
        rxState = WAIT_HEADER_2;

      break;

    case WAIT_HEADER_2:
      if (receivedByte == 0x55) {
        controlFrame = false;
        rxState = READ_JOINT1_MSB;
      } else if (receivedByte == 0x56) {
        controlFrame = true;
        rxState = READ_JOINT1_MSB;
      } else
        rxState = WAIT_HEADER_1;
      break;

    case READ_JOINT1_MSB:
      currentPacket.joint1Msb = receivedByte;
      rxState = READ_JOINT1_LSB;
      break;

    case READ_JOINT1_LSB:
      currentPacket.joint1Lsb = receivedByte;
      rxState = READ_JOINT2_MSB;
      break;

    case READ_JOINT2_MSB:
      currentPacket.joint2Msb = receivedByte;
      rxState = READ_JOINT2_LSB;
      break;

    case READ_JOINT2_LSB:
      currentPacket.joint2Lsb = receivedByte;
      // Control frames stay 7 bytes; data frames carry the two torque fields.
      rxState = controlFrame ? READ_CHECKSUM : READ_TAU1_MSB;
      break;

    case READ_TAU1_MSB:
      currentPacket.tau1Msb = receivedByte;
      rxState = READ_TAU1_LSB;
      break;

    case READ_TAU1_LSB:
      currentPacket.tau1Lsb = receivedByte;
      rxState = READ_TAU2_MSB;
      break;

    case READ_TAU2_MSB:
      currentPacket.tau2Msb = receivedByte;
      rxState = READ_TAU2_LSB;
      break;

    case READ_TAU2_LSB:
      currentPacket.tau2Lsb = receivedByte;
      rxState = READ_CHECKSUM;
      break;

    case READ_CHECKSUM: {

      currentPacket.checksum = receivedByte;

      uint8_t calculatedChecksum;
      if (controlFrame) {
        // Unchanged control-frame checksum (bytes 2..5).
        calculatedChecksum = currentPacket.joint1Msb + currentPacket.joint1Lsb +
                             currentPacket.joint2Msb + currentPacket.joint2Lsb;
      } else {
        // 8-bit additive checksum over bytes 0..9 (header + four int16 fields).
        calculatedChecksum = static_cast<uint8_t>(
            0xAA + 0x55 + currentPacket.joint1Msb + currentPacket.joint1Lsb +
            currentPacket.joint2Msb + currentPacket.joint2Lsb +
            currentPacket.tau1Msb + currentPacket.tau1Lsb +
            currentPacket.tau2Msb + currentPacket.tau2Lsb);
      }

      // Verify checksum
      if ((calculatedChecksum & 0xFF) == currentPacket.checksum) {

        if (controlFrame) {
          // Control frame: AA 56 CMD 00 00 00 CMD
          const uint8_t command = currentPacket.joint1Msb;
          if (command == 0x01) { // STOP -> immediate, latched safe state
            remoteStop = true;
            // A STOP must also abort any homing that is in progress.
            isHoming1 = false;
            isHoming2 = false;
            encode1.running(0, pow(2, PWM_RESOLUTION));
            encode2.running(0, pow(2, PWM_RESOLUTION));
            queueStatus(STATUS_STOPPED);
            if (DEBUG_SERIAL)
              Serial.println("[CTRL] STOP");
          } else if (command == 0x02) { // HOME -> existing MControl homing only
#if HOME_SWITCHES_WIRED
            if (isHoming1 || isHoming2) {
              // Already homing: do not restart or overlap the sequence.
              queueStatus(STATUS_HOMING);
            } else {
              remoteStop = false;
              isHoming1 = true;
              isHoming2 = true;
              queueStatus(STATUS_HOMING);
            }
            if (DEBUG_SERIAL)
              Serial.println("[CTRL] HOME");
#else
            queueStatus(STATUS_HOME_UNAVAILABLE);
            Serial.println("[CTRL] HOME ignored: home switches not verified");
#endif
          }
        } else {

          // Reconstruct signed int16_t desired position
          const int16_t receivedJoint1Count = static_cast<int16_t>(
              (static_cast<uint16_t>(currentPacket.joint1Msb) << 8) |
              currentPacket.joint1Lsb);

          const int16_t receivedJoint2Count = static_cast<int16_t>(
              (static_cast<uint16_t>(currentPacket.joint2Msb) << 8) |
              currentPacket.joint2Lsb);

          // Reconstruct the signed int16 dynamics torques (N*m).
          const int16_t receivedTau1 = static_cast<int16_t>(
              (static_cast<uint16_t>(currentPacket.tau1Msb) << 8) |
              currentPacket.tau1Lsb);
          const int16_t receivedTau2 = static_cast<int16_t>(
              (static_cast<uint16_t>(currentPacket.tau2Msb) << 8) |
              currentPacket.tau2Lsb);

          remoteStop = false;
          setDesiredCounts(receivedJoint1Count, receivedJoint2Count);

          // Decode the torque and evaluate the normalization mapping.
          // pwmNorm1/2 are isolated: they are NOT fed to the PID or the BTS7960
          // outputs.
          tauDyn1Nm = receivedTau1 * TORQUE_LSB_NM;
          tauDyn2Nm = receivedTau2 * TORQUE_LSB_NM;
          pwmNorm1 = normalizeTorquePwm(tauDyn1Nm, TAU_SCALE_1_NM);
          pwmNorm2 = normalizeTorquePwm(tauDyn2Nm, TAU_SCALE_2_NM);

          if (DEBUG_SERIAL) {
            Serial.print("[RX] joint1=");
            Serial.print(receivedJoint1Count);
            Serial.print(" joint2=");
            Serial.println(receivedJoint2Count);
          }
        }

      } else {
        // Reject the whole packet: reference/torque are not applied.  Expose
        // the error count and resynchronise on the next 0xAA header.
        checksumErrors++;
        if (DEBUG_SERIAL)
          Serial.println("[ERR] Checksum mismatch");
      }

      // Ready for next packet
      rxState = WAIT_HEADER_1;

      break;
    }
    }
  }
}

void controlMotor2(int16_t target) {
  const int32_t actual = encode2.count_enc;
  float error = static_cast<float>(target) - static_cast<float>(actual);

  if (target != motor2Reference) {
    if (motor2State != M2_TRACKING)
      encode2.resetPID();
    motor2Reference = target;
    motor2State = M2_TRACKING;
    motor2Settling = false;
    motor2SettleStart = 0;
  }

  // Stop released (physical STOP pin) -> resume ordinary control.
  if (motor2State == M2_STOPPED) {
    encode2.resetPID();
    motor2State = M2_TRACKING;
    motor2Settling = false;
  }

  // Never drive on non-finite or implausible feedback.
  if (!isfinite(error) ||
      fabs(error) > static_cast<float>(MAX_PLAUSIBLE_ERROR_COUNTS)) {
    encode2.running(0, pow(2, PWM_RESOLUTION));
    speed2 = 0.0f;
    return;
  }

  const float absError = fabs(error);

  if (motor2State == M2_HOLDING) {
    if (absError <= static_cast<float>(DISTURB_THRESHOLD_2)) {
      encode2.running(0, pow(2, PWM_RESOLUTION));
      speed2 = 0.0f;
      return;
    }
    encode2.resetPID();
    motor2State = M2_RECOVERING;
    motor2Settling = false;
  }

  // Target-reached detection: within tolerance continuously for SETTLE_TIME_MS.
  if (absError <= static_cast<float>(POS_TOLERANCE_2)) {
    const unsigned long now = millis();
    if (!motor2Settling) {
      motor2Settling = true;
      motor2SettleStart = now;
    } else if (now - motor2SettleStart >= SETTLE_TIME_MS) {
      encode2.running(0, pow(2, PWM_RESOLUTION));
      encode2.resetPID();
      motor2State = M2_HOLDING;
      motor2Settling = false;
      speed2 = 0.0f;
      return;
    }
  } else {
    motor2Settling = false;
  }

  float out = encode2.PIDcalc(error);
  if (!isfinite(out)) {
    encode2.running(0, pow(2, PWM_RESOLUTION));
    speed2 = 0.0f;
    return;
  }
  // The motor command is the PID output only: no dynamics feedforward is added.
  driveMotor2(out);
  speed2 = MOTOR2_INVERT_DRIVE ? -out : out;
}

void runMotorControl() {
  // Latest valid desired reference received over UART.  The PC generates the
  // time-varying trajectory and streams successive reference samples; the
  // ESP32 only follows the most recent valid sample (held between packets), so
  // the PID loop runs independently of packet arrivals.
  int16_t targetJoint1Count = 0;
  int16_t targetJoint2Count = 0;

  getDesiredCounts(targetJoint1Count, targetJoint2Count);

  error_interpolate1 = static_cast<float>(targetJoint1Count) -
                       static_cast<float>(encode1.count_enc);
  // Motor command is the PID output only: no dynamics feedforward is added.
  speed1 = encode1.PIDcalc(error_interpolate1);
  encode1.running(speed1, pow(2, PWM_RESOLUTION));

  error_interpolate2 = static_cast<float>(targetJoint2Count) -
                       static_cast<float>(encode2.count_enc);
  controlMotor2(targetJoint2Count);

  Serial.println("Speed1: " + String(speed1) + " Speed2: " + String(speed2));
}

void checkEmergencyStop() {
#if PHYSICAL_STOP_BUTTON_WIRED
  if (digitalRead(STOP) == LOW) {

    isStop = true;
    encode1.running(0, pow(2, PWM_RESOLUTION));
    encode2.running(0, pow(2, PWM_RESOLUTION));

    if (DEBUG_SERIAL)
      Serial.println("[STOP] Emergency stop activated");

  } else
    isStop = false;
#else
  // Operator STOP is now issued from the HMI (control frame) and latched in
  // remoteStop.  There is no physical STOP push-button input.
  isStop = false;
#endif
}

void runHoming() {
  if (isHoming1) {
    encode1.homing(HOME1, HIGH, &isHoming1);
    if (!isHoming1) {
      Serial.println("[HOMING] Motor 1 homed");
      Theta.theta1 = 0.0f;
      ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_1));

      encoder_offset_1 = 0;
      encode1.count_enc = 0;
      if (!isHoming2)
        queueStatus(
            STATUS_HOMED); // delivered by controlTask (retries if TX busy)
    }
  }

  if (isHoming2) {
    encode2.homing(HOME2, HIGH, &isHoming2);
    if (!isHoming2) {
      Serial.println("[HOMING] Motor 2 homed");
      Theta.theta2 = PI;
      const int motor2HomeCount = PWM_NUM2 / 2;
      ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_2));

      encoder_offset_2 = motor2HomeCount;
      encode2.count_enc = motor2HomeCount;
      if (!isHoming1)
        queueStatus(
            STATUS_HOMED); // delivered by controlTask (retries if TX busy)
    }
  }
}

void checkCommunicationTimeout() {

  unsigned long currentTime = millis();
  bool hasReceivedPacket = false;
  unsigned long lastValidPacketTime = 0;
  portENTER_CRITICAL(&sharedStateMux);

  hasReceivedPacket = receivedFirstPacket;
  lastValidPacketTime = lastPacketTime;
  portEXIT_CRITICAL(&sharedStateMux);

  // Do not activate timeout before first valid packet
  if (!hasReceivedPacket) {
    isTimeout = false;
    return;
  }
  if ((currentTime - lastValidPacketTime) > SERIAL_TIMEOUT_MS) {
    isTimeout = true;

    if (DEBUG_SERIAL)
      Serial.println("[TIMEOUT] No packet received, holding last target");

  } else
    isTimeout = false;
}

void communicationTask(void *parameter);
void controlTask(void *parameter);

void setup() {
  // Use the actual USB-serial adapter path exposed by the ESP32 DevKit board.
  Serial.begin(SERIAL_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  pinMode(GPIO_NUM_2, OUTPUT);
  digitalWrite(GPIO_NUM_2, HIGH);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_C, INPUT);
  pinMode(ENC_D, INPUT);

  pinMode(Veloc1_A, OUTPUT);
  pinMode(Veloc1_B, OUTPUT);
  pinMode(Veloc2_A, OUTPUT);
  pinMode(Veloc2_B, OUTPUT);

  pinMode(Veloc1_A_EN, OUTPUT);
  pinMode(Veloc1_B_EN, OUTPUT);
  pinMode(Veloc2_A_EN, OUTPUT);
  pinMode(Veloc2_B_EN, OUTPUT);

  // HOME limit-switch sensor inputs (used by homing()).
#if HOME_SWITCHES_WIRED
  pinMode(HOME1, INPUT_PULLUP);
  pinMode(HOME2, INPUT_PULLUP);
#endif

  // Old physical STOP push-button input; operator STOP is now an HMI command.
#if PHYSICAL_STOP_BUTTON_WIRED
  pinMode(STOP, INPUT_PULLUP);
#endif

  init_encoder_1();
  init_encoder_2();

  Theta.theta1 = 0.0f;
  Theta.theta2 = PI;

  // Initial encoder positions
  encode1.count_enc = 0;
  encode2.count_enc = PWM_NUM2 / 2;

  // Initial desired positions
  desiredJoint1Count = 0;
  desiredJoint2Count = PWM_NUM2 / 2;

  encode1.setPID(0.5f, 0.05f, 0.3f);
  encode2.setPID(0.05f, 0.01f, 0.03f);

  analogWriteFrequency(Veloc1_A, PWM_FREQ);
  analogWriteFrequency(Veloc1_B, PWM_FREQ);
  analogWriteFrequency(Veloc2_A, PWM_FREQ);
  analogWriteFrequency(Veloc2_B, PWM_FREQ);

  analogWriteResolution(Veloc1_A, PWM_RESOLUTION);
  analogWriteResolution(Veloc1_B, PWM_RESOLUTION);
  analogWriteResolution(Veloc2_A, PWM_RESOLUTION);
  analogWriteResolution(Veloc2_B, PWM_RESOLUTION);

  analogWrite(Veloc1_A, 0);
  analogWrite(Veloc1_B, 0);
  analogWrite(Veloc2_A, 0);
  analogWrite(Veloc2_B, 0);

  digitalWrite(Veloc1_A_EN, HIGH);
  digitalWrite(Veloc1_B_EN, HIGH);
  digitalWrite(Veloc2_A_EN, HIGH);
  digitalWrite(Veloc2_B_EN, HIGH);

  Serial.println();
  Serial.println("[ESP32] 2R planar robot controller ready");

  Serial.println("[ESP32] Binary protocol: "
                 "AA 55 JOINT1_MSB JOINT1_LSB "
                 "JOINT2_MSB JOINT2_LSB CHECKSUM");

  Serial.print("[ESP32] Baud: ");
  Serial.println(SERIAL_BAUD);
  Serial.println("[ESP32] JOINT1 and JOINT2 are "
                 "signed int16_t encoder counts");
  Serial.println();

  lastPacketTime = millis();
  lastTelemetryTime = millis();

  xTaskCreatePinnedToCore(communicationTask, "communicationTask", 4096, nullptr,
                          2, nullptr, 0);

  xTaskCreatePinnedToCore(controlTask, "controlTask", 4096, nullptr, 1, nullptr,
                          1);
}

void communicationTask(void *parameter) {
  (void)parameter;
  for (;;) {
    receiveBinaryPacket();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void controlTask(void *parameter) {
  (void)parameter;

  for (;;) {
    updateEncoderCounts();
    sendEncoderTelemetry();
    checkEmergencyStop();

    // Emit any status queued by the UART task (HOME/STOP feedback).
    if (pendingStatusFlag) {
      const uint8_t code = pendingStatusCode;
      pendingStatusFlag = false;
      writeStatusFrame(code);
    }

    if (isStop || remoteStop) {
      isHoming1 = false;
      isHoming2 = false;
      encode1.running(0, pow(2, PWM_RESOLUTION));
      encode2.running(0, pow(2, PWM_RESOLUTION));
      if (motor2State != M2_STOPPED) {
        encode2.resetPID();
        motor2State = M2_STOPPED;
        motor2Settling = false;
        speed2 = 0.0f;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    if (isHoming1 || isHoming2) {
      runHoming();
      if (isHoming1 || isHoming2) {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
    }
    checkCommunicationTimeout();
    runMotorControl();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }