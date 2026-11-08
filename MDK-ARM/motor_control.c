#include "motor_control.h"

//电机端口定义
#define MotorGPIO_Left	L_DIR_GPIO_Port
#define MotorPIN_Left	L_DIR_Pin
#define MotorGPIO_Right	R_DIR_GPIO_Port
#define MotorPIN_Right	R_DIR_Pin

//位置环ppdd参数
#define PPDD_KP1 10.0f
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
#define TARGET_SPEED 20.0f

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
uint8_t flag=0;
uint8_t zhi_jiao_state = 0;       // 0=无直角, 1=左直角, 2=右直角
uint8_t zhi_jiao_ticks = 0;       // 直角强打倒计时
uint8_t zhi_jiao_confirm_l = 0;   // 左直角确认计数
uint8_t zhi_jiao_confirm_r = 0;   // 右直角确认计数
float debug1=0,debug2=0,debug3=0,debug4=0;

// 环岛参数
#define RA_ENTER_CONFIRM     20   // 入环连续确认拍数
#define RA_COOLDOWN          500   // 出环冷却时间

// 【入环/出环强制打角】
#define RA_ENTER_FORCE_TICKS 150   // 入环打角拍数
#define RA_EXIT_FORCE_TICKS  200   // 出环打角拍数
#define RA_FORCE_DIFF        12.0f // 强制打角差速
#define RA_CORNER_BLANK_TICKS 250  // 入环后前 250 拍绝对不数角

// 宽线判定范围
#define WIDE_LINE_MIN        6
#define WIDE_LINE_MAX        9

uint8_t  ra_exit_force_cnt = 0;         // 出环打角倒计时
uint8_t  roundabout_enter_cnt = 0;
uint16_t roundabout_time_cnt  = 0;
uint8_t  roundabout_corner_cnt = 0;   
uint8_t  in_corner_flag = 0;          
uint16_t roundabout_cooldown = 0;     
uint8_t  roundabout_dir = 0;          // 0=未知, 1=左入环, 2=右入环

//控制器
PID_Controller steering_ppdd={
    .kp=PPDD_KP1, .kp2=PPDD_KP2, .kd=PPDD_KD, .gkd=PPDD_GKD,
};

PID_Controller left_pid={
	.kp=L_SPEED_KP, .ki=L_SPEED_KI, .kd=L_SPEED_KD, .integral_max=SPEED_I_MAX
};

PID_Controller right_pid={
	.kp=R_SPEED_KP, .ki=R_SPEED_KI, .kd=R_SPEED_KD, .integral_max=SPEED_I_MAX
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
        if(motor.CCR==1) TIM1->CCR1=pwm_output;
        else if(motor.CCR==2) TIM1->CCR2=pwm_output;
        return;
    }
    if(motor.CCR==1) TIM1->CCR1=pwm_output;
    else if(motor.CCR==2) TIM1->CCR2=pwm_output;
    
    if(motor.pwm_output>0) HAL_GPIO_WritePin(motor.motor_GPIO,motor.motor_PIN,motor.motor_DIR);
    else HAL_GPIO_WritePin(motor.motor_GPIO,motor.motor_PIN,!motor.motor_DIR);
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

// 入环检测：分叉/变宽
uint8_t RoundaboutDetect(uint8_t active_count)
{
    if (active_count >= 10) return 0; // 排除十字/超宽线
    if (active_count >= WIDE_LINE_MIN && active_count <= WIDE_LINE_MAX) return 1; 
    return 0;
}

//状态机
void update_condition(uint8_t active_count)
{
    static uint8_t straight_debounce = 0;

    // 冷却期保护：防止出环的宽线瞬间再次触发入环
    if (roundabout_cooldown > 0) {
        roundabout_cooldown--;
    }

    switch (condition_now) {
    case ZhiDao:
        // 纯直道/弯道/直角状态下，只认环岛！
        if (RoundaboutDetect(active_count) && roundabout_cooldown == 0) {
            roundabout_enter_cnt++;
            
            // 连续达到确认拍数，直接进环岛
            if (roundabout_enter_cnt >= RA_ENTER_CONFIRM) {
                condition_now = Roundabout_Run; 
                roundabout_corner_cnt = 0; 
                in_corner_flag = 0; 
                straight_debounce = 0;   
                roundabout_time_cnt = 0; 
                
                // 记录入环方向（重心判定）
                uint8_t left_light  = mux_array[0] + mux_array[1] + mux_array[2];
                uint8_t right_light = mux_array[9] + mux_array[10] + mux_array[11];
                roundabout_dir = (left_light >= right_light) ? 1 : 2;
                
                roundabout_enter_cnt = 0; // 进环岛后清零
            }
        } 
        else {
            roundabout_enter_cnt = 0; 
        }
        break;
        
    case During_WanDao:
        condition_now = ZhiDao;
        break;

    case Roundabout_Run:
        // 【取消超时】限制最大值，防止 uint16_t 溢出归零导致数角屏蔽期再次触发
        if (roundabout_time_cnt < 5000) {
            roundabout_time_cnt++;
        }
        
        // 前 250 拍（包含强制打角和缓冲期）绝对不数角
        if (roundabout_time_cnt > 250) {
            // 数角判定：限制在 6~9 路（<10 防止被十字路口干扰）
            if (active_count >= WIDE_LINE_MIN && active_count < 10) {
                if (in_corner_flag == 0) {
                    roundabout_corner_cnt++; 
                    in_corner_flag = 1;
                }
                straight_debounce = 0; 
            } 
            else { 
                // 只要不是宽线，统统算作直道开始确认
                straight_debounce++;
                if (straight_debounce >= 30) { 
                    in_corner_flag = 0; 
                }
            }
        } else {
            in_corner_flag = 0;
        }

        // 走完 6 个角，触发强制出环
        if (roundabout_corner_cnt >= 6) {
            condition_now = Roundabout_Exit;
            ra_exit_force_cnt = RA_EXIT_FORCE_TICKS; 
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
    
    // ==========================================
    // 1. 【直角强打逻辑】（绝对让位给环岛）
    // ==========================================
    if (condition_now != ZhiDao || roundabout_enter_cnt > 0) {
        // 如果在环岛或正在确认入环，打断直角
        zhi_jiao_ticks = 0;
        zhi_jiao_state = 0;
        zhi_jiao_confirm_l = 0; 
        zhi_jiao_confirm_r = 0; 
    } else {
        if (zhi_jiao_ticks == 0 && roundabout_cooldown == 0) {
            uint8_t left_edge  = mux_array[0] + mux_array[1] + mux_array[2];
            uint8_t right_edge = mux_array[9] + mux_array[10] + mux_array[11];
            
            // 只要不是十字路口级别的宽线
            if (active_count < 10) {
                if (left_edge >= 3) {
                    zhi_jiao_confirm_r = 0; 
                    zhi_jiao_confirm_l++;
                    if (zhi_jiao_confirm_l >= 3) { // 连续3拍确认
                        zhi_jiao_state = 1; 
                        zhi_jiao_ticks = 80;        // 强打80拍
                        zhi_jiao_confirm_l = 0;      
                    }
                } else if (right_edge >= 3) {
                    zhi_jiao_confirm_l = 0; 
                    zhi_jiao_confirm_r++;
                    if (zhi_jiao_confirm_r >= 3) { // 连续3拍确认
                        zhi_jiao_state = 2; 
                        zhi_jiao_ticks = 80;
                        zhi_jiao_confirm_r = 0;      
                    }
                } else {
                    zhi_jiao_confirm_l = 0;
                    zhi_jiao_confirm_r = 0;
                }
            } else {
                zhi_jiao_confirm_l = 0;
                zhi_jiao_confirm_r = 0;
            }
        }
    }

    // 执行直角打角
    uint8_t is_in_zhi_jiao = 0;
    if (zhi_jiao_ticks > 0) {
        zhi_jiao_ticks--;
        is_in_zhi_jiao = 1;
        
        // 智能退出：如果边缘线灭了，提前结束
        uint8_t left_edge  = mux_array[0] + mux_array[1] + mux_array[2];
        uint8_t right_edge = mux_array[9] + mux_array[10] + mux_array[11];
        if (zhi_jiao_state == 1 && left_edge == 0) zhi_jiao_ticks = 0;
        if (zhi_jiao_state == 2 && right_edge == 0) zhi_jiao_ticks = 0;

        // 强行覆盖差速！
        if (zhi_jiao_state == 1) {
            differential_speed = 19.0f;   // 左直角强打
        } else if (zhi_jiao_state == 2) {
            differential_speed = -19.0f;  // 右直角强打
        }
    }

    // 如果没在直角强打，且在普通循迹，才乘 1.5 倍
    if (!is_in_zhi_jiao && condition_now == ZhiDao && roundabout_enter_cnt == 0) {
        differential_speed *= 1.5f;
    }

    // ==========================================
    // 2. 环岛强制打角（优先级极高，覆盖一切）
    // ==========================================
    if (condition_now == Roundabout_Run && roundabout_time_cnt <= RA_ENTER_FORCE_TICKS) {
        if (roundabout_dir == 1) {
            differential_speed = RA_FORCE_DIFF;   
        } else if (roundabout_dir == 2) {
            differential_speed = -RA_FORCE_DIFF;  
        }
    }

    if (ra_exit_force_cnt > 0) {
        ra_exit_force_cnt--;
        if (roundabout_dir == 1) {
            differential_speed = -RA_FORCE_DIFF;  
        } else if (roundabout_dir == 2) {
            differential_speed = RA_FORCE_DIFF;   
        }
    }

    // 输出电机
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