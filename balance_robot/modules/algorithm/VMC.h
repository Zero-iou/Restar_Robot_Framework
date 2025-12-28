#ifndef _VMC_H
#define _VMC_H

#include "user_lib.h"

typedef struct 
{
    float phi_0, phi_1, phi_2, phi_3, phi_4;
    float l1, l2, l3, l4, l5;   // 腿部摆杆长度
    float theta, dtheta, lsat_dtheta;
    float l0, dl0;
    float dphi2;

    float j_[4];  

    float fn;
    float ddzw;
} VMC_t;

void VMC_Init(VMC_t *vmc, float l1, float l2, float l5, float dt);
void VMC_Calc(VMC_t *vmc, float phi_1, float phi_4, float dphi_1, float dphi_4, float phi_, float z_accel);
void VMC_Jacobian(VMC_t *VMC);
void VMC_Force(VMC_t *vmc, float torq_f, float torq_b, float wr);

#endif