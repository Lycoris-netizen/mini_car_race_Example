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

// 环岛参数
#define RA_ENTER_CONFIRM  4     // 入环连续确认拍数（防抖）
#define RA_TIMEOUT        10000  // 环岛超时保护
#define RA_COOLDOWN       1000   // 出环冷却时间

// 【入环强制打角】
#define RA_ENTER_FORCE_TICKS 30 // 入环前 30 拍强制打角
#define RA_FORCE_DIFF        15.0f // 强制打角差速

// 【出环强制打角】(新增)
#define RA_EXIT_FORCE_TICKS  40 // 出环时强制打角的拍数
uint8_t  ra_exit_force_cnt = 0;         // 出环打角倒计时

uint8_t  roundabout_enter_cnt = 0;
uint16_t roundabout_time_cnt  = 0;
uint8_t  roundabout_corner_cnt = 0;   
uint8_t  in_corner_flag = 0;          
uint16_t roundabout_cooldown = 0;     
uint8_t  roundabout_dir = 0;          // 0=未知, 1=左入环, 2=右入环
// 宽线判定范围（下限建议保持 3 防止直道误判，上限放宽到 7）
#define WIDE_LINE_MIN 3
#define WIDE_LINE_MAX 7

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

//弯道
uint8_t WanDaoCheck(uint8_t active_count){
    uint8_t result=0;
    if(my_abs(weighted_sum)>9.0) result=2;
    return result;
}

//直角边沿判定
uint8_t ZhiJiaoEdge(uint8_t active_count)
{
    uint8_t left_group  = mux_array[0] + mux_array[1];
    uint8_t right_group = mux_array[10] + mux_array[11];
    uint8_t result = 0;

    if (active_count < 9) {
        if (left_group == 2 && left_group_last != 2)        { result = 1; flag = 1; }
        else if (right_group == 2 && right_group_last != 2) { result = 2; flag = 1; }
    }

    left_group_last  = left_group;
    right_group_last = right_group;
    return result;
}

//直角保持判定
uint8_t ZhiJiaoHold(uint8_t active_count)
{
    if (active_count >= 9) return 0;
    uint8_t left_group  = mux_array[0] + mux_array[1];
    uint8_t right_group = mux_array[10] + mux_array[11];
    if (left_group  == 2) return 1;
    if (right_group == 2) return 2;
    return 0;
}

// 入环检测：分叉/变宽
uint8_t RoundaboutDetect(uint8_t active_count)
{
    if (active_count >= 9) return 0; // 排除十字/超宽线
    if (active_count >= WIDE_LINE_MIN && active_count <= WIDE_LINE_MAX) return 1; 
    return 0;
}

//状态机
void update_condition(uint8_t active_count)
{
    static uint8_t straight_debounce = 0;

    // 冷却期保护：强行让直角判定失效，防止出环的宽线吃掉后续直角
    if (roundabout_cooldown > 0) {
        roundabout_cooldown--;
        left_group_last = mux_array[0] + mux_array[1]; 
        right_group_last = mux_array[10] + mux_array[11];
    }

    uint8_t zhi_angle = ZhiJiaoEdge(active_count);

    switch (condition_now) {
    case ZhiDao:
        if (RoundaboutDetect(active_count) && roundabout_cooldown == 0) {
            roundabout_enter_cnt++;
            if (roundabout_enter_cnt >= RA_ENTER_CONFIRM) {
                condition_now = Roundabout_Run; 
                roundabout_corner_cnt = 0; 
                in_corner_flag = 0;
                straight_debounce = 0;   
                roundabout_time_cnt = 0; 
                roundabout_dir = (weighted_sum < 0) ? 1 : 2; // 记录入环方向
            }
        } 
        else if (zhi_angle) {
            condition_now = During_ZhiJiao;
            roundabout_enter_cnt = 0; 
        } 
        else {
            roundabout_enter_cnt = 0; 
        }
        break;

    case During_ZhiJiao:
        if (RoundaboutDetect(active_count) && roundabout_cooldown == 0) {
            roundabout_enter_cnt++;
            if (roundabout_enter_cnt >= RA_ENTER_CONFIRM) {
                condition_now = Roundabout_Run; 
                roundabout_corner_cnt = 0; 
                in_corner_flag = 0;
                straight_debounce = 0;
                roundabout_time_cnt = 0; 
                roundabout_dir = (weighted_sum < 0) ? 1 : 2;
                break;
            }
            break; 
        } else {
            roundabout_enter_cnt = 0;
        }
        
        if (ZhiJiaoHold(active_count) == 0) condition_now = ZhiDao;
        break;

    case During_WanDao:
        if (WanDaoCheck(active_count) == 0) condition_now = ZhiDao;
        break;

    case Roundabout_Run:
        if (++roundabout_time_cnt > RA_TIMEOUT) {
            condition_now = ZhiDao;
            roundabout_cooldown = RA_COOLDOWN;
            roundabout_corner_cnt = 0; 
            in_corner_flag = 0;
            break;
        }
        
        // 数角判定：使用放宽后的宽线范围
        if (active_count >= WIDE_LINE_MIN && active_count <= WIDE_LINE_MAX) {
            if (in_corner_flag == 0) {
                roundabout_corner_cnt++; 
                in_corner_flag = 1;
            }
            straight_debounce = 0; 
        } 
        else if (active_count >= 1 && active_count <= 2) {
            straight_debounce++;
            if (straight_debounce >= 10) { 
                in_corner_flag = 0; 
            }
        }

        // 走完 6 个角，触发强制出环
        if (roundabout_corner_cnt >= 6) {
            condition_now = Roundabout_Exit;
            ra_exit_force_cnt = RA_EXIT_FORCE_TICKS; // 【新增】启动出环打角倒计时
        }
        break;

    case Roundabout_Exit:
        condition_now = ZhiDao;
        roundabout_enter_cnt = 0;
        roundabout_corner_cnt = 0;
        roundabout_time_cnt = 0;
        roundabout_cooldown = RA_COOLDOWN; 
        break;

    default:
        condition_now = ZhiDao;
        break;
    }
}

//计算光电管中心位置偏差
float calculate_line_position(uint16_t mux_value){
    weighted_sum=0.0f;
    active_count=0;
    for(uint8_t i=0;i<12;i++){
        mux_array[i]=MUX_GET_CHANNEL(mux_value,i);
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
    float line_position=calculate_line_position(mux_value);
    last_position_error=line_position;
    
    float positional_deviation=ppdd_calculate(&steering_ppdd,0.0f,line_position,gyro_z);
    differential_speed=positional_deviation;

    update_condition(active_count);
    
    // 直角放大
    if(condition_now == During_ZhiJiao) {
        differential_speed *= 1.5;
    }
    
    // 【入环强制打角】进入环岛前 30 拍，硬拉进去
    if (condition_now == Roundabout_Run && roundabout_time_cnt <= RA_ENTER_FORCE_TICKS) {
        if (roundabout_dir == 1) {
            differential_speed = -RA_FORCE_DIFF; // 左入环 -> 强制左打角
        } else if (roundabout_dir == 2) {
            differential_speed = RA_FORCE_DIFF;  // 右入环 -> 强制右打角
        }
    }

    // 【出环强制打角】刚数完 6 个角，强制反向打角汇入直道
    if (ra_exit_force_cnt > 0) {
        ra_exit_force_cnt--;
        if (roundabout_dir == 1) {
            differential_speed = RA_FORCE_DIFF;  // 左入环的出环口 -> 向右汇入直道
        } else if (roundabout_dir == 2) {
            differential_speed = -RA_FORCE_DIFF; // 右入环的出环口 -> 向左汇入直道
        }
    }

    debug2 = differential_speed;
    debug1 = target_speed_now;

    float tl = target_speed_now - differential_speed;
    float tr = target_speed_now + differential_speed;
    if(tl < 0) tl = 0;
    if(tr < 0) tr = 0;
    motor_left.target_speed = tl;
    motor_right.target_speed = tr;
	
    debug3 = motor_left.target_speed;
    debug4 = motor_right.target_speed;

    motor_left.current_speed = left_encoder;
    motor_right.current_speed = right_encoder;

    float pwm_l = pid_calculate(&motor_left.speed_pid, motor_left.target_speed, motor_left.current_speed);
    float pwm_r = pid_calculate(&motor_right.speed_pid, motor_right.target_speed, motor_right.current_speed);
    pwm_l = constrain(pwm_l, PWM_MIN, PWM_MAX);
    pwm_r = constrain(pwm_r, PWM_MIN, PWM_MAX);
    if (motor_left.target_speed  < 0.5f && pwm_l < 0.0f) pwm_l = 0.0f;
    if (motor_right.target_speed < 0.5f && pwm_r < 0.0f) pwm_r = 0.0f;
    motor_left.pwm_output  = (int16_t)pwm_l;
    motor_right.pwm_output = (int16_t)pwm_r;
    setPWM(motor_left);
    setPWM(motor_right);
}