#include "LQR.h"

// float LQR_K_calc(float coe[40][6], float len_l, float len_r, uint8_t n)
// {
//     return coe[n][0] + coe[n][1] * len_l + coe[n][2] * len_r + coe[n][3] * len_l * len_l + coe[n][4] * len_l * len_r + coe[n][5] * len_r * len_r;
// }

void LQR_Init(LQR_t *lqr, const float k[12][3]) {
  for (uint8_t i = 0; i < 12; ++i) {
    for (uint8_t j = 0; j < 3; ++j) {
      lqr->k[i][j] = k[i][j];
    }
  }
}

void LQR_SetSpeed(LQR_t *lqr, float tar_vx) {
  lqr->target_speed = tar_vx;
}

void LQR_Calc(LQR_t *lqr, VMC_t vmc, float dist_, float vel_, float phi_, float dphi_) {
  
    float T_K_[2][6];
    for (uint8_t i = 0; i < 2; ++i) {
      uint8_t j = i * 6;
      T_K_[i][0] =
          (lqr->k[j + 0][0] * powf(vmc.l0, 2) + lqr->k[j + 0][1] * vmc.l0 + lqr->k[j + 0][2]) * -vmc.theta;
      T_K_[i][1] =
          (lqr->k[j + 1][0] * powf(vmc.l0, 2) + lqr->k[j + 1][1] * vmc.l0 + lqr->k[j + 1][2]) * -vmc.dtheta;
      T_K_[i][2] =
          (lqr->k[j + 2][0] * powf(vmc.l0, 2) + lqr->k[j + 2][1] * vmc.l0 + lqr->k[j + 2][2]) * -dist_;
      T_K_[i][3] = (lqr->k[j + 3][0] * powf(vmc.l0, 2) + lqr->k[j + 3][1] * vmc.l0 + lqr->k[j + 3][2]) *
                   (lqr->target_speed - vel_);
      T_K_[i][4] =
          (lqr->k[j + 4][0] * powf(vmc.l0, 2) + lqr->k[j + 4][1] * vmc.l0 + lqr->k[j + 4][2]) * -phi_;
      T_K_[i][5] =
          (lqr->k[j + 5][0] * powf(vmc.l0, 2) + lqr->k[j + 5][1] * vmc.l0 + lqr->k[j + 5][2]) * -dphi_;
    }
  
    // if (vmc.fn < 20.0f) {
    //   for (uint8_t i = 0; i < 6; ++i) {
    //     T_K_[0][i] = 0.0f;
    //   };
    //   T_K_[1][2] = T_K_[1][3] = T_K_[1][4] = T_K_[1][5] = 0.0f;
    // }
  
    for (uint8_t i = 0; i < 2; ++i) {
      lqr->T_[i] = T_K_[i][0] + T_K_[i][1] + T_K_[i][2] + T_K_[i][3] + T_K_[i][4] +
              T_K_[i][5];
    }
  }

