#pragma once

#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <driver/pulse_cnt.h>
#include <driver/uart.h>
#include <esp_adc/adc_oneshot.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define UART_PORT_NUM UART_NUM_0
#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3
#define SERIAL_BAUD 115200

#define R_PWM GPIO_NUM_25
#define L_PWM GPIO_NUM_26
#define R_EN GPIO_NUM_27
#define L_EN GPIO_NUM_14
#define MOTOR1_ENC_A GPIO_NUM_32
#define MOTOR1_ENC_B GPIO_NUM_33
#define IN_A0 GPIO_NUM_18
#define IN_A1 GPIO_NUM_19
#define PWM_A GPIO_NUM_21
#define MOTOR2_ENC_A GPIO_NUM_34
#define MOTOR2_ENC_B GPIO_NUM_35
#define STBY GPIO_NUM_4
#define IN_B0 GPIO_NUM_22
#define IN_B1 GPIO_NUM_23
#define PWM_B GPIO_NUM_5
#define MOTOR3_ENC_A GPIO_NUM_36
#define MOTOR3_ENC_B GPIO_NUM_39

#define CONTROL_FREQUENCY_HZ 100
#define CONTROL_DT 0.01f
#define TIMER_RESOLUTION_HZ 10000
#define TIMER_ALARM_VALUE (TIMER_RESOLUTION_HZ / CONTROL_FREQUENCY_HZ)
#define TIME1_PER_90 1.8f
#define TIME2_PER_90 2.0f
#define SERIAL_TIMEOUT_MS 500
#define CMD_QUEUE_SIZE 32
#define BUFFER_QUEUE_SIZE 32
#define HOME1_ANGLE 0.0f
#define HOME2_ANGLE 3.14159265358979323846f
#define HOMING_TIMEOUT_MS 10000U
#define HOMING_PWM 80.0f
#define CIRCLE_DIRECTION 1.0f
// Define these in build_flags only when the corresponding hardware exists.
// HOME1_GPIO, HOME2_GPIO and STOP_GPIO are intentionally undefined here.
#if defined(HOME1_GPIO) || defined(HOME2_GPIO) || defined(STOP_GPIO)
#define OPTIONAL_SAFETY_INPUTS 1
#endif

extern QueueHandle_t cmdQueue;
extern QueueHandle_t bufferQueue;
extern TaskHandle_t controlTaskHandle;

void gpio_configuration();
void timer_configuration();
void pwm_configuration();
void adc_configuration();
void uart_configuration();
void i2c_configuration();
void start_control_timer();
