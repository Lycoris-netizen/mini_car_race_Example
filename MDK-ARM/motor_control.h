//控制函数头文件

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "dodo_BMI270.h"
#include "assist_function.h"
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

typedef enum{
	ZhiDao=0,
	During_ZhiJiao=1,
	During_WanDao=2,
	Roundabout_Enter=3,  // 预留状态
	Roundabout_Run=4,    // 环岛内行驶
	Roundabout_Exit=5,   // 出环确认
}condition_t;

void motor_init(void);

void setPWM(motor_t motor);

void stopPWM(motor_t motor);

uint8_t ZhiJiaoEdge(uint8_t active_count);

uint8_t ZhiJiaoHold(uint8_t active_count);

uint8_t WanDaoCheck(uint8_t active_count);

// 【补上这一行，保持规范】
uint8_t RoundaboutDetect(uint8_t active_count);

void update_condition(uint8_t active_count);

float calculate_line_position(uint16_t mux_value);

void parallel_pid_control(uint16_t mux_value,float gyro_z,float left_encoder,float right_encoder);

extern uint8_t flag;

#endif