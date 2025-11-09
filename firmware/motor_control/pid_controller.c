#include "pid_controller.h"

void pid_init(pid_controller_t* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = 100.0f;
    pid->output_limit = 255.0f;
}

float pid_update(pid_controller_t* pid, float error, float dt) {
    // 比例項
    float proportional = pid->kp * error;
    
    // 積分項
    pid->integral += error * dt;
    // 積分限制
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float integral = pid->ki * pid->integral;
    
    // 微分項
    float derivative = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // 計算輸出
    float output = proportional + integral + derivative;
    
    // 輸出限制
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}
