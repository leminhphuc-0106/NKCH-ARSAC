#pragma once
#include <cstdint>
struct DesiredAngles {
  float q1, q2;
  uint32_t receivedAtMs;
};
struct DesiredCounts {
  int16_t count1, count2;
  uint32_t receivedAtMs;
};
struct EncoderCounts {
  int32_t count1, count2;
};
enum class CommandType : uint8_t { HOME, POINT, LINE, CIRCLE };
struct Command {
  CommandType type;
  int x1, y1, z1, x2, y2, z2, radius;
  uint32_t receivedAtMs;
};
