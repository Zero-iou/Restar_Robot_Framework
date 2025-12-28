#include "VMC.h"

static float lenth_1, lenth_2, lenth_3, lenth_4, lenth_5;
static float control_dt;

void VMC_Init(VMC_t *vmc, float l1, float l2, float l5, float dt)
{
	control_dt = dt;

	lenth_1 = l1;
	lenth_2 = l2;
	lenth_3 = l2;
	lenth_4 = l1;
	lenth_5 = l5;

	// VMC腿长数据
    vmc->l1 = l1;
    vmc->l2 = l2;
    vmc->l3 = l2;
    vmc->l4 = l1;
    vmc->l5 = l5;
}


void VMC_Calc(VMC_t *vmc, float phi_1, float phi_4, float dphi_1, float dphi_4, float phi_, float z_accel)
{
    float xb_, xd_, yb_, yd_, xc_, yc_, lbd2_;
    float a0_, b0_, c0_;
	float phi0_, phi2_, phi3_, theta_;
	float cos_phi1, cos_phi4, sin_phi1, sin_phi4;
	float lenth_0;

	float p_xb_,p_xd_, p_yb_, p_yd_, p_xc_, p_yc_, p_lbd2_;
	float p_a0_, p_b0_, p_c0_;
	float p_phi0_, p_phi2_, p_theta_;
	float p_phi1_, p_phi4_;

	float dphi2_, dphi0_, dl0_, ddl0_, dtheta_, ddtheta_, ddzw_;
	static float last_dl0_, last_dtheta_;
	
	// 计算当前时刻参数
    cos_phi1 = arm_cos_f32(phi_1);
	sin_phi1 = arm_sin_f32(phi_1);
	cos_phi4 = arm_cos_f32(phi_4);
	sin_phi4 = arm_sin_f32(phi_4);

    xb_ = lenth_1 * cos_phi1;
 	xd_ = lenth_5 + lenth_4 * cos_phi4;
	yb_ = lenth_1 * sin_phi1;
	yd_ = lenth_4 * sin_phi4;

 	lbd2_ = powf(xd_ - xb_, 2) + powf(yd_ - yb_, 2);

	a0_ = 2.0f * lenth_2 * (xd_ - xb_);
	b0_ = 2.0f * lenth_2 * (yd_ - yb_);
	c0_ = powf(lenth_2, 2) + lbd2_ - powf(lenth_3, 2);


	phi2_ = 2.0f * atan2f((b0_ + sqrtf(powf(a0_, 2) + powf(b0_, 2) - powf(lbd2_, 2))), a0_ + lbd2_);
	xc_ = xb_ + lenth_2 * arm_cos_f32(phi2_);
	yc_ = yb_ + lenth_2 * arm_sin_f32(phi2_);

	phi3_ = atan2f(yc_ - yd_, xc_ - xd_);

	phi0_ = atan2f(yc_, (xc_ - lenth_5 / 2.0f));
	// 机体角度
	theta_ = phi0_ - 0.5 * PI - phi_;

	// 腿长
	lenth_0 = sqrtf(powf(xc_ - lenth_5 / 2.0f, 2) + powf(yc_, 2));

	// 预测下一时刻参数
	p_phi1_ = phi_1 + dphi_1 * control_dt;  
	p_phi4_ = phi_4 + dphi_4 * control_dt;

	p_xb_ = lenth_1 * arm_cos_f32(p_phi1_);
	p_xd_ = lenth_5 + lenth_4 * arm_cos_f32(p_phi4_);
	p_yb_ = lenth_1 * arm_sin_f32(p_phi1_);
	p_yd_ = lenth_4 * arm_sin_f32(p_phi4_);
	p_lbd2_ = powf(p_xd_ - p_xb_, 2) + powf(p_yd_ - p_yb_, 2);
  	
	p_a0_ = 2 * lenth_2 * (p_xd_ - p_xb_);
  	p_b0_ = 2 * lenth_2 * (p_yd_ - p_yb_);
 
	p_phi2_ = 2 * atan2f(p_b0_ + sqrtf(powf(p_a0_, 2) + powf(p_b0_, 2) - powf(p_lbd2_, 2)), p_a0_ + p_lbd2_);
	p_xc_ = p_xb_ + lenth_2 * arm_cos_f32(p_phi2_);
  	p_yc_ = p_yb_ + lenth_2 * arm_sin_f32(p_phi2_);
	
	p_phi0_ = atan2f(p_yc_, p_xc_ - lenth_5 / 2);
  	
	// 计算腿长变化率和腿角速度
 	dphi2_ = (p_phi2_ - phi2_) / control_dt;
  	dphi0_ = (p_phi0_ - phi0_) / control_dt;
	dtheta_ = (p_phi0_ - 0.5 * PI - phi_ - theta_) / control_dt;
	dl0_ = ((sqrtf(powf(p_xc_ - lenth_5 / 2, 2) + powf(p_yc_, 2))) - lenth_0) / control_dt;
	ddl0_ = (dl0_ - last_dl0_) / (0.002f + 0.008f) + ddl0_ * 0.008f / (0.002f + 0.008f);
	ddtheta_ = (dtheta_ - last_dtheta_) / (0.002f + 0.008f) + ddtheta_ * 0.008f / (0.002f + 0.008f);
	ddzw_ = z_accel - ddl0_ * arm_cos_f32(theta_) + 
			2.0f * dl0_ * dtheta_ * arm_sin_f32(theta_) +
			lenth_0 * ddtheta_ * arm_cos_f32(theta_) +
			lenth_0 * powf(dtheta_, 2) * arm_sin_f32(theta_);

	// 获取VMC参数
	vmc->phi_0 = phi0_;
	vmc->phi_1 = phi_1;
	vmc->phi_2 = phi2_;
	vmc->phi_3 = phi3_;
	vmc->phi_4 = phi_4;
    vmc->l0 = lenth_0;
	vmc->dl0 = dl0_;
    vmc->dphi2 = dphi2_;
    vmc->theta = theta_;
	vmc->lsat_dtheta = p_phi0_ - 0.5 * PI - phi_;
    vmc->dtheta = dtheta_;
	vmc->ddzw = ddzw_;

	last_dtheta_ = dtheta_;
	last_dl0_ = dl0_;
}

void VMC_Jacobian(VMC_t *vmc)
{
	float phi32 = vmc->phi_3 - vmc->phi_2;
	float phi03 = vmc->phi_0 - vmc->phi_3;
	float phi02 = vmc->phi_0 - vmc->phi_2;
	float phi12 = vmc->phi_1 - vmc->phi_2;
	float phi34 = vmc->phi_3 - vmc->phi_4;
  
	vmc->j_[0] = vmc->l1 * arm_sin_f32(phi03) * arm_sin_f32(phi12) / arm_sin_f32(phi32);
	vmc->j_[1] = vmc->l1 * arm_cos_f32(phi03) * arm_sin_f32(phi12) /
			(vmc->l0 * arm_sin_f32(phi32));
	vmc->j_[2] = vmc->l4 * arm_sin_f32(phi02) * arm_sin_f32(phi34) / arm_sin_f32(phi32);
	vmc->j_[3] = vmc->l4 * arm_cos_f32(phi02) * arm_sin_f32(phi34) /
			(vmc->l0 * arm_sin_f32(phi32));

	// vmc->j_[0] = ( vmc->l1 * arm_sin_f32(vmc->phi_0 - vmc->phi_3) * arm_sin_f32( vmc->phi_1 - vmc->phi_2 ) ) 
	// 	/ (arm_sin_f32( vmc->phi_3 - vmc->phi_2 ));
	// vmc->j_[1] = ( vmc->l1 * arm_cos_f32(vmc->phi_0 - vmc->phi_3) * arm_sin_f32( vmc->phi_1 - vmc->phi_2 ) ) 
	// 	/ (arm_sin_f32( vmc->phi_3 - vmc->phi_2 ) * vmc->l0);
	// vmc->j_[2] = ( vmc->l4 * arm_sin_f32(vmc->phi_0 - vmc->phi_2) * arm_sin_f32( vmc->phi_3 - vmc->phi_4 ) ) 
	// 	/ (arm_sin_f32( vmc->phi_3 - vmc->phi_2 ));
	// vmc->j_[3] = ( vmc->l4 * arm_cos_f32(vmc->phi_0 - vmc->phi_2) * arm_sin_f32( vmc->phi_3 - vmc->phi_4 ) ) 
	// 	/ (arm_sin_f32( vmc->phi_3 - vmc->phi_2 ) * vmc->l0);
}

void VMC_Force(VMC_t *vmc, float torq_f, float torq_b, float wr) {
	float det = vmc->j_[0] * vmc->j_[3] - vmc->j_[1] * vmc->j_[2];
	float temp1_, temp2_, temp3_;
	float inv_j_[4];

	inv_j_[0] = vmc->j_[3] / det;
	inv_j_[1] = -vmc->j_[1] / det;
	inv_j_[2] = -vmc->j_[2] / det;
	inv_j_[3] = vmc->j_[0] / det;
  
	temp1_ = inv_j_[0] * torq_f + inv_j_[1] * torq_b;
	temp2_ = inv_j_[2] * torq_f + inv_j_[3] * torq_b;
	temp3_ = temp1_ * arm_cos_f32(vmc->theta) + temp2_ * arm_sin_f32(vmc->theta) / vmc->l0;
	vmc->fn = temp3_ + wr * (9.8f + vmc->ddzw);
  }
