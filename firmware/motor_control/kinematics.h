#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

typedef struct {
    float x;
    float y;
    float theta;
} pose_t;

typedef struct {
    float left;
    float right;
} wheel_speed_t;

typedef struct {
    float wheel_radius;
    float wheel_base;
    float max_rpm;
} robot_config_t;

// 運動學函數聲明
void forward_kinematics(const wheel_speed_t* wheel_speeds, float dt, pose_t* pose);
void inverse_kinematics(float linear_vel, float angular_vel, wheel_speed_t* wheel_speeds);
void calculate_odometry(const wheel_speed_t* wheel_delta, pose_t* robot_pose);

#endif // KINEMATICS_H
