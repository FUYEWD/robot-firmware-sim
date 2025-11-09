#include "kinematics.h"
#include <math.h>

void forward_kinematics(const wheel_speed_t* wheel_speeds, float dt, pose_t* pose) {
    float left_distance = wheel_speeds->left * dt;
    float right_distance = wheel_speeds->right * dt;
    
    float delta_theta = (right_distance - left_distance) / WHEEL_BASE;
    float delta_distance = (left_distance + right_distance) / 2.0f;
    
    pose->x += delta_distance * cos(pose->theta + delta_theta / 2.0f);
    pose->y += delta_distance * sin(pose->theta + delta_theta / 2.0f);
    pose->theta += delta_theta;
}

void inverse_kinematics(float linear_vel, float angular_vel, wheel_speed_t* wheel_speeds) {
    wheel_speeds->left = linear_vel - (angular_vel * WHEEL_BASE / 2.0f);
    wheel_speeds->right = linear_vel + (angular_vel * WHEEL_BASE / 2.0f);
}
