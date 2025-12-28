#ifndef __LQR_H
#define __LQR_H

#include <user_lib.h>

#include "VMC.h"

typedef struct 
{
    // float l_len, l_last_len, r_len, r_last_len;
    // float l_dlen, r_dlen;

    // float l_l_angle, l_l_last_angle, l_l_gyro;
    // float l_r_angle, l_r_last_angle,l_r_gyro;

    // float l_l_phi2, l_l_last_phi2, l_l_phi2_gyro;
    // float l_r_phi2, l_r_last_phi2, l_r_phi2_gyro; 

    // float joint_balancing_torque_l, joint_balancing_torque_r;
    // float joint_stand_torque_l, joint_stand_torque_r;
    // float joint_moving_torque_l, joint_moving_torque_r;
    // float joint_vertical_torque_l, joint_vertical_torque_r;
    // float joint_horizontal_torque_l, joint_horizontal_torque_r;

    // int foot_balancing_torque_l, foot_balancing_torque_r;
    // int foot_moving_torque_l, foot_moving_torque_r;


    // float joint_combine_torque_lf, joint_combine_torque_lb;
    // float joint_combine_torque_rf, joint_combine_torque_rb;
    // float foot_combine_torque_l, foot_combine_torque_r;
    float target_speed;

    float k[12][3];
    float T_[2];
} LQR_t;

void LQR_Init(LQR_t *lqr, const float k[12][3]);
void LQR_SetSpeed(LQR_t *lqr, float tar_vx);
void LQR_Calc(LQR_t *lqr, VMC_t vmc, float dist_, float vel_, float phi_, float dphi_);
// float LQR_K_calc(float coe[40][6], float len_l, float len_r, uint8_t n);

#endif