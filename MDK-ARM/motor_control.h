//控制函数头文件

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "dodo_BMI270.h"
#include "assist_function.h"
#include "motor_control.h"
#include "PID.h"
#include "multiplexer.h"

typedef struct{
    float target_speed;
    float current_speed;
    PID_Controller speed_pid;
    int16_t pwm_output;

    GPIO_TypeDef* motor_GPIO;
    uint16_t motor_PIN;
    uint8_t motor_DIR;
    uint8_t CCR;
}motor_t;

extern uint8_t flag;

#endif