#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
} pid_controller_t;

void pid_init(pid_controller_t* pid, float kp, float ki, float kd);
float pid_update(pid_controller_t* pid, float error, float dt);

#endif // PID_CONTROLLER_H
