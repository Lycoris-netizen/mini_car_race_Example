#include "motor_control.h"

//电机端口定义
#define MotorGPIO_Left	L_DIR_GPIO_Port
#define MotorPIN_Left	L_DIR_Pin
#define MotorGPIO_Right	R_DIR_GPIO_Port
#define MotorPIN_Right	R_DIR_Pin

//位置环ppdd参数
#define PPDD_KP1 6.25f
/*6.2f*/
#define PPDD_KP2 0.0f
#define PPDD_KD  0.0f
#define PPDD_GKD 0.0f

//速度环PID参数
#define L_SPEED_KP 200.0f
#define L_SPEED_KI 0.0f
#define L_SPEED_KD 0.0f
#define R_SPEED_KP 200.0f
#define R_SPEED_KI 0.0f
#define R_SPEED_KD 0.0f

//目标速度
#define TARGET_SPEED 15.0f
/*15.0f*/

//速度积分限幅
#define SPEED_I_MAX 2000.0f
// #define GUOCHONG 0.0f

//PWM输出限制
#define PWM_MAX 700
#define PWM_MIN -700

float target_speed_now=TARGET_SPEED;
float differential_speed=0;
float weighted_sum=0.0f;
float last_position_error=0.0f;
uint8_t active_count=0;
uint8_t stop_flag=0;
uint8_t mux_array[12]={0};
uint8_t left_group_last=0,right_group_last=0;
uint8_t flag=0;
float debug1=0,debug2=0,debug3=0,debug4=0;


//控制器
PID_Controller steering_ppdd={
    .kp=PPDD_KP1,
    .kp2=PPDD_KP2,
    .kd=PPDD_KD,
	.gkd=PPDD_GKD,
};

PID_Controller left_pid={
	.kp=L_SPEED_KP,
	.ki=L_SPEED_KI,
	.kd=L_SPEED_KD,
	.integral_max=SPEED_I_MAX
};

PID_Controller right_pid={
	.kp=R_SPEED_KP,
	.ki=R_SPEED_KI,
	.kd=R_SPEED_KD,
	.integral_max=SPEED_I_MAX
};

//左右电机
motor_t motor_left;
motor_t motor_right;

//状态
condition_t condition_now=ZhiDao;

//motor初始化函数
void motor_init(void){
    motor_left.speed_pid=left_pid;
    motor_left.motor_GPIO=MotorGPIO_Left;
    motor_left.motor_PIN=MotorPIN_Left;
    motor_left.motor_DIR=GPIO_PIN_SET;
    motor_left.CCR=1;

    motor_right.speed_pid=right_pid;
    motor_right.motor_GPIO=MotorGPIO_Right;
    motor_right.motor_PIN=MotorPIN_Right;
    motor_right.motor_DIR=GPIO_PIN_SET;
    motor_right.CCR=2;
}

//设置PWM波
void setPWM(motor_t motor){
    uint16_t pwm_output=my_abs_int(motor.pwm_output);
    if(motor.pwm_output==0||stop_flag){
        if(motor.CCR==1){
            TIM1->CCR1=pwm_output;
        }
        else if(motor.CCR==2){
            TIM1->CCR2=pwm_output;
        }
        return;
    }
    if(motor.CCR==1){
        TIM1->CCR1=pwm_output;
    }
    else if(motor.CCR==2){
        TIM1->CCR2=pwm_output;
    }
    if(motor.pwm_output>0){
        HAL_GPIO_WritePin(motor.motor_GPIO,motor.motor_PIN,motor.motor_DIR);
    }
    else{
        HAL_GPIO_WritePin(motor.motor_GPIO,motor.motor_PIN,!motor.motor_DIR);
    }
}

//停止PWM
void stopPWM(motor_t motor){
    motor_left.pwm_output=0;
    motor_right.pwm_output=0;
    TIM1->CCR1=0;
    TIM1->CCR2=0;
    stop_flag=1;
}

//弯道（阈值 9.0 不可达，实际永不触发，保留供以后调整）
uint8_t WanDaoCheck(uint8_t active_count){
    uint8_t result=0;
    if(my_abs(weighted_sum)>9.0) result=2;
    return result;
}

//直角边沿判定：只在"非2 -> 2"时触发一次，用于进入直角状态
//active_count >= 9 时视为宽线/十字，不判直角
uint8_t ZhiJiaoEdge(uint8_t active_count)
{
    uint8_t left_group  = mux_array[0] + mux_array[1];
    uint8_t right_group = mux_array[10] + mux_array[11];
    uint8_t result = 0;

    if (active_count < 9) {
        if (left_group == 2 && left_group_last != 2)        { result = 1; flag = 1; }
        else if (right_group == 2 && right_group_last != 2) { result = 2; flag = 1; }
    }

    /* last 更新必须放在 active_count 判断之外，否则宽线过后边沿判断会错 */
    left_group_last  = left_group;
    right_group_last = right_group;
    return result;
}

//直角保持判定：只要 left_group==2 或 right_group==2 就维持直角状态
uint8_t ZhiJiaoHold(uint8_t active_count)
{
    if (active_count >= 9) return 0;
    uint8_t left_group  = mux_array[0] + mux_array[1];
    uint8_t right_group = mux_array[10] + mux_array[11];
    if (left_group  == 2) return 1;
    if (right_group == 2) return 2;
    return 0;
}

//状态机
void update_condition(uint8_t active_count)
{
    uint8_t zhi_angle = ZhiJiaoEdge(active_count);   /* ← 移到外面，每拍都调 */

    if (condition_now == ZhiDao) {
        if (zhi_angle)                            condition_now = During_ZhiJiao;
        else if (WanDaoCheck(active_count))       condition_now = During_WanDao;
    }
    else if (condition_now == During_ZhiJiao) {
        if (ZhiJiaoHold(active_count) == 0)       condition_now = ZhiDao;
    }
    else if (condition_now == During_WanDao) {
        if (WanDaoCheck(active_count) == 0)       condition_now = ZhiDao;
    }
}

//计算光电管中心位置偏差
float calculate_line_position(uint16_t mux_value){
    weighted_sum=0.0f;
    active_count=0;
    //更新读数
    for(uint8_t i=0;i<12;i++){
        mux_array[i]=MUX_GET_CHANNEL(mux_value,i);
		if(i==11) continue;//神的逻辑
        if(MUX_GET_CHANNEL(mux_value,i)){
            weighted_sum+=i-5.5;
            active_count++;
        }
    }
    if(active_count==0){
        return last_position_error;
    }
    return weighted_sum/active_count;
}

//并联PID控制主函数
void parallel_pid_control(uint16_t mux_value,float gyro_z,float left_encoder,float right_encoder){
    //偏差计算
    float line_position=calculate_line_position(mux_value);
    last_position_error=line_position;
    //PD控制计算位置偏差
    float positional_deviation=ppdd_calculate(&steering_ppdd,0.0f,line_position,gyro_z);
    //差速度映射
	  differential_speed=positional_deviation;

    update_condition(active_count);
    if(condition_now==During_ZhiJiao) differential_speed*=1.5;
    else if(condition_now==During_WanDao) //differential_speed*=1.5;

    //target_speed_now=0;//调位置环时解除注释
	debug2=differential_speed;
	debug1=target_speed_now;

    float tl=target_speed_now-differential_speed;
    float tr=target_speed_now+differential_speed;
    if(tl<0) tl=0;
    if(tr<0) tr=0;
    motor_left.target_speed=tl;
    motor_right.target_speed=tr;
	
    debug3=motor_left.target_speed;
	debug4=motor_right.target_speed;

    // 更新当前速度（从编码器读取）
    motor_left.current_speed=left_encoder;
    motor_right.current_speed=right_encoder;

    // 左右电机PID控制
    float pwm_l=pid_calculate(&motor_left.speed_pid,motor_left.target_speed,motor_left.current_speed);
    float pwm_r=pid_calculate(&motor_right.speed_pid,motor_right.target_speed,motor_right.current_speed);
    pwm_l = constrain(pwm_l, PWM_MIN, PWM_MAX);   // 先限幅 float
    pwm_r = constrain(pwm_r, PWM_MIN, PWM_MAX);
    if (motor_left.target_speed  < 0.5f && pwm_l < 0.0f) pwm_l = 0.0f;
    if (motor_right.target_speed < 0.5f && pwm_r < 0.0f) pwm_r = 0.0f;
    motor_left.pwm_output=(int16_t)pwm_l;
    motor_right.pwm_output=(int16_t)pwm_r;
	setPWM(motor_left);
	setPWM(motor_right);
}