#pragma once
#include <cstdint>
#include <driver/pulse_cnt.h>

void pcnt_configuration();
int32_t encoder1_count();
int32_t encoder2_count();
void encoder1_reset(int32_t count = 0);
void encoder2_reset(int32_t count = 0);
