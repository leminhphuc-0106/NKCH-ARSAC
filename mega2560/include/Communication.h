#pragma once
#include "CommunicationTypes.h"
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

void communication_task(void *arg);
void send_status_text(const char *text);
extern QueueHandle_t highLevelCommandQueue;
