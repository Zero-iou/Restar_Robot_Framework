/**
 * @file chassis.c
 * @author Zero 1433026627@qq.com
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2024-11-18
 *
 * @copyright Copyright (c) 2022 Team JiaoLong-SJTU
 *
 */

#include "balance_chassis.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "power_manager.h"   
#include "message_center.h"
#include "kalman_filter.h"
#include "user_lib.h"
#include "VMC.h"
#include "LQR.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"

//这里使用了extern INS后续需要删掉
#include "ins_task.h"

static Chassis_Config_s *chassis_config;
 
/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

//  static SuperCapInstance *cap;                                       // 超级电容
//  static PowerManagerInstance *power_manager;                         // 功率管理
 
static DMMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
static DJIMotorInstance *foot_motor_lf, *foot_motor_rf;

static VMC_t vmc_l, vmc_r;
static LQR_t lqr_l, lqr_r;

static const float k[12][3] = {
    131.778304, -137.557891, -13.340548,
    14.450102, -14.817570, -2.881438,
    14.055766, -15.065086, -6.124135,
    12.382752, -16.090410, -13.327124,
    38.775877, -69.636522, 40.157613,
    6.353800, -7.748552, 4.724512,
    -176.546894, 55.045389, 32.904874,
    -0.484142, -13.475066, 10.489045,
    -35.258015, 5.650781, 10.570830,
    -32.883040, -11.998524, 22.408794,
    -151.696537, 148.889444, 36.419835,
    -10.525397, 11.837334, 1.114983
};

static float len_l = 0.2, len_r = 0.2; 

static float target_speed, speed, speed_m, dist;
static float joint_lf_tor, joint_lb_tor, joint_rf_tor, joint_rb_tor;
static float foot_l_tor, foot_r_tor;


static KalmanFilter_t kf;

static PIDInstance l_leg_PID, r_leg_PID;
/* 私有变量用于底盘旋转的机械参数常量 */
//  static float lf_center, rf_center, lb_center, rb_center; // 左右前后轮子中心
//  static float rpm_2_wheel_vector;
/* 用于自旋变速策略的时间变量 */
// static float t;
 
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
//  static float chassis_vx, chassis_vy;                 // 将云台系的速度投影到底盘
//  static float vt_lf, vt_rf, vt_lb, vt_rb;             // 底盘速度解算后的输出
//  static float rl_vt_lf, rl_vt_rf, rl_vt_lb, rl_vt_rb; // 底盘真实速度 m/s
// static DRMotorInstance *drmotor;

static void EstimateSpeed();

#ifndef GIMBAL_BOARD

#define VEL_PROCESS_NOISE 100   // 速度过程噪声
#define VEL_MEASURE_NOISE 200  // 速度测量噪声
// 同时估计加速度和速度时对加速度的噪声
// 更好的方法是设置为动态,当有冲击时/加加速度大时更相信轮速
#define ACC_PROCESS_NOISE 2000  // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.05  // 加速度测量噪声
#define TASK_CONTROL_PERIOD 0.001f

// 不能用运算符，算出结果有误
#define REDUCE_RATIO_FOOT 0.063432f

 void BalanceChassisInit()
 {
    //  static float half_wheel_base, half_track_width, perimeter_wheel;    // 半轴距,半轮距,轮子周长
    //  static float center_gimbal_offset_x, center_gimbal_offset_y;        // 中心云台偏移量
    //  static float reduction_ratio_wheel;                                 // 轮子减速比

    chassis_config = ChassisConfigFeed();

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_UP_LF];
    foot_motor_lf = DJIMotorInit(&chassis_config->chassis_motor_config);    

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_UP_RF];
    foot_motor_rf = DJIMotorInit(&chassis_config->chassis_motor_config);    

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_LF];
    chassis_config->chassis_motor_config.can_init_config.rx_id = 1;
    motor_lf = DMMotorInit(&chassis_config->chassis_motor_config);    

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_RF];
    chassis_config->chassis_motor_config.can_init_config.rx_id = 2;
    motor_rf = DMMotorInit(&chassis_config->chassis_motor_config);    

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_RB];
    chassis_config->chassis_motor_config.can_init_config.rx_id = 3;
    motor_rb = DMMotorInit(&chassis_config->chassis_motor_config);    

    chassis_config->chassis_motor_config.can_init_config.tx_id = chassis_config->chassis_motor_id[MOTOR_LB];
    chassis_config->chassis_motor_config.can_init_config.rx_id = 4;
    motor_lb = DMMotorInit(&chassis_config->chassis_motor_config);    

    // PID会影响是否能站立正常
    PID_Init_Config_s l_config_PID = {
        .Kp = 200.0f,
        .Ki = 0.0f,
        .Kd = 45.0f,
        .MaxOut = 10.0f,
        .DeadBand = 0.0001f,

        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | 
                PID_DerivativeFilter | PID_Derivative_On_Measurement,

        .CoefB = 0.01f,
        .Output_LPF_RC = 0.02f,
        .Derivative_LPF_RC = 0.08f,
    };

    PID_Init_Config_s r_config_PID = {
        .Kp = 200.0f,
        .Ki = 0.0f,
        .Kd = 45.0f,
        .MaxOut = 30.0f,
        .DeadBand = 0.0001f,

        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | 
                PID_DerivativeFilter | PID_Derivative_On_Measurement,

        .CoefB = 0.01f,
        .Output_LPF_RC = 0.02f,
        .Derivative_LPF_RC = 0.08f,
    };

    PIDInit(&l_leg_PID, &l_config_PID);
    PIDInit(&r_leg_PID, &r_config_PID);

    // 使用kf同时估计速度和加速度
    Kalman_Filter_Init(&kf, 2, 0, 2);
    float F[4] = {1, 0.001, 0, 1};
    float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_MEASURE_NOISE};
    float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};
    float P[4] = {100000, 0, 0, 100000};
    float H[4] = {1, 0, 0, 1};
    memcpy(kf.F_data, F, sizeof(F));
    memcpy(kf.Q_data, Q, sizeof(Q));
    memcpy(kf.R_data, R, sizeof(R));
    memcpy(kf.P_data, P, sizeof(P));
    memcpy(kf.H_data, H, sizeof(H));

    VMC_Init(&vmc_l, 0.15, 0.27, 0.15, TASK_CONTROL_PERIOD);
    VMC_Init(&vmc_r, 0.15, 0.27, 0.15, TASK_CONTROL_PERIOD);

    LQR_Init(&lqr_l, k);
    LQR_Init(&lqr_r, k);

     // 发布订阅初始化,如果为双板,则需要can comm来传递消息
 #ifdef CHASSIS_BOARD
     Chassis_IMU_data = INS_Init(); // 底盘IMU初始化
 
     CANComm_Init_Config_s comm_conf = {
         .can_config = {
             .can_handle = &hcan2,
             .tx_id = 0x311,
             .rx_id = 0x312,
         },
         .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
         .send_data_len = sizeof(Chassis_Upload_Data_s),
     };
     chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
 #endif                                          // CHASSIS_BOARD
 
 #ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
     chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
     chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
 #endif // ONE_BOARD
 }

 #define TORQ_K 3350.9803f
 /**
  * @brief 计算每个轮毂电机的输出,正运动学解算
  *        用宏进行预替换减小开销,运动解算具体过程参考教程
  */
 static float chassis_accel, chassis_vx, chassis_x, chassis_x_set, flag;

 float left_wheel_phi_vx, right_wheel_phi_vx;
 float left_body_vx, right_body_vx;
 static void BalanceControlCalculate()
 {
    EstimateSpeed();

    LQR_SetSpeed(&lqr_l, chassis_cmd_recv.vx);
    LQR_SetSpeed(&lqr_r, chassis_cmd_recv.vx);

    if (fabsf(speed) < 0.1f) {
        dist += speed * TASK_CONTROL_PERIOD;
    } else {
        dist = 0.4f;
    }

    LQR_Calc(&lqr_l, vmc_l, dist, speed, chassis_cmd_recv.pitch, chassis_cmd_recv.pitch_gyro);
    LQR_Calc(&lqr_r, vmc_r, dist, speed, chassis_cmd_recv.pitch, chassis_cmd_recv.pitch_gyro);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        len_l = 0.2f;
        len_r = 0.2f;
    }
    else
    {
        len_l = 0.2f;
        len_r = 0.2f;
    }

 }


float j_stand_torque_l, j_stand_torque_r;
float j_force_torque_l, j_force_torque_r;
 static void BalanceTorqueCombine()
 {

    // roll_comp = k_roll_extra_comp_p * INS.Roll * DEGREE_2_RAD;
    // left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp - roll_comp;
    // right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp + roll_comp;

    j_stand_torque_l = PIDCalculate(&l_leg_PID, vmc_l.l0, len_l);
    j_stand_torque_r = PIDCalculate(&r_leg_PID, vmc_r.l0, len_r);

    j_force_torque_l = lqr_l.T_[1];
    j_force_torque_r = lqr_r.T_[1];

    // joint_lb_tor = vmc_l.j_[0] * j_stand_torque_l;
    // joint_lf_tor = vmc_l.j_[2] * j_stand_torque_l;

    joint_lb_tor = vmc_l.j_[0] * j_stand_torque_l + vmc_l.j_[1] * j_force_torque_l;
    joint_lf_tor = vmc_l.j_[2] * j_stand_torque_l + vmc_l.j_[3] * j_force_torque_l;

    joint_rb_tor = vmc_r.j_[0] * j_stand_torque_r + vmc_r.j_[1] * j_force_torque_r;
    joint_rf_tor = vmc_r.j_[2] * j_stand_torque_r + vmc_r.j_[3] * j_force_torque_r;

    // if (left_leg_.GetForceNormal() < 20.0f) {
    //     foot_l_tor = lqr_l.T_[0];
    //   } else {
    //     foot_l_tor = lqr_left_.GetWheelTor() - yaw_speed_.GetOutput();
    //   }
    //   if (right_leg_.GetForceNormal() < 20.0f) {
    //     foot_r_tor = lqr_right_.GetWheelTor();
    //   } else {
    //     foot_r_tor = lqr_right_.GetWheelTor() + yaw_speed_.GetOutput();
    //   }

    foot_l_tor = lqr_l.T_[0] * TORQ_K;
    foot_r_tor = lqr_r.T_[0] * TORQ_K;

    // 限幅输出
    joint_lf_tor = float_constrain(joint_lf_tor, -8.5f, 8.5f);
    joint_lb_tor = float_constrain(joint_lb_tor, -8.5f, 8.5f);

    joint_rf_tor = float_constrain(joint_rf_tor, -8.5f, 8.5f);
    joint_rb_tor = float_constrain(joint_rb_tor, -8.5f, 8.5f);
    foot_l_tor = int_constrain(foot_l_tor, -16000, 16000);
    foot_r_tor = int_constrain(foot_r_tor, -16000, 16000);
 }


 /**
  * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
  *        对于双板的情况,考虑增加来自底盘板IMU的数据
  *
  */
 static void EstimateSpeed()
 {
    float l_w_vx_, r_w_vx_, l_b_vx_, r_b_vx_;

    l_w_vx_ = foot_motor_lf->measure.speed_aps * REDUCE_RATIO_FOOT * DEGREE_2_RAD + vmc_l.dphi2;
    r_w_vx_ = -foot_motor_rf->measure.speed_aps * REDUCE_RATIO_FOOT * DEGREE_2_RAD + vmc_r.dphi2;
    // l_w_vx_ = foot_motor_lf->measure.speed_aps * REDUCE_RATIO_FOOT * DEGREE_2_RAD;
    // r_w_vx_ = -foot_motor_rf->measure.speed_aps * REDUCE_RATIO_FOOT * DEGREE_2_RAD;


    // l_b_vx_ = l_w_vx_ * 0.09f +
    //         vmc_l.l0 * vmc_l.dtheta + vmc_l.dl0 * arm_sin_f32(vmc_l.theta);
    // r_b_vx_ = r_w_vx_ * 0.09f +
    //         vmc_r.l0 * vmc_r.dtheta + vmc_r.dl0 * arm_sin_f32(vmc_r.theta);

    l_b_vx_ = l_w_vx_ * 0.09f;
    r_b_vx_ = r_w_vx_ * 0.09f;
    speed_m = -(l_b_vx_ + r_b_vx_) / 2;

    // // 离地检测
    // if (vmc_l.fn < 20.0f && vmc_r.fn < 20.0f) {
    //     speed = 0;
    // }
  
    // 使用kf同时估计加速度和速度,滤波更新
    kf.MeasuredVector[0] = speed_m;
    kf.MeasuredVector[1] = INS.MotionAccel_n[Y];
    kf.F_data[1] = TASK_CONTROL_PERIOD;  // 更新F矩阵
    Kalman_Filter_Update(&kf);
    speed = kf.xhat_data[0];
 }

 #define JOINT_ANGLE_OFFSET   3.14f
//  #define JOINT_ANGLE_OFFSET   0.0f
 /* 机器人底盘控制核心任务 */
 void BalanceChassisTask()
 {
     // 后续增加没收到消息的处理(双板的情况)
     // 获取新的控制信息
 #ifdef ONE_BOARD
     SubGetMessage(chassis_sub, &chassis_cmd_recv);
 #endif
 #ifdef CHASSIS_BOARD
     chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
 #endif // CHASSIS_BOARD

    DJIMotorChangeFeed(foot_motor_lf, OPEN_LOOP, MOTOR_FEED);
    DJIMotorChangeFeed(foot_motor_rf, OPEN_LOOP, MOTOR_FEED);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DMMotorStop(motor_lf);
        DMMotorStop(motor_lb);
        DMMotorStop(motor_rf);
        DMMotorStop(motor_rb);

        DJIMotorStop(foot_motor_lf);
        DJIMotorStop(foot_motor_rf);
    }
    else
    { // 正常工作
        // DMMotorStop(motor_lf);
        // DMMotorStop(motor_lb);
        // DMMotorStop(motor_rf);
        // DMMotorStop(motor_rb);

        // DJIMotorStop(foot_motor_lf);
        // DJIMotorStop(foot_motor_rf);

        DMMotorEnable(motor_lf);
        DMMotorEnable(motor_lb);
        DMMotorEnable(motor_rf);
        DMMotorEnable(motor_rb);

        DJIMotorEnable(foot_motor_lf);
        DJIMotorEnable(foot_motor_rf);
    }

    VMC_Calc(&vmc_l, motor_lb->measure.position + JOINT_ANGLE_OFFSET, motor_lf->measure.position, 
        motor_lb->measure.velocity, motor_lf->measure.velocity, chassis_cmd_recv.pitch, INS.MotionAccel_n[Z]);
    VMC_Calc(&vmc_r, -motor_rb->measure.position + JOINT_ANGLE_OFFSET, -motor_rf->measure.position,
        -motor_rb->measure.velocity, -motor_rf->measure.velocity, chassis_cmd_recv.pitch, INS.MotionAccel_n[Z]);
    VMC_Jacobian(&vmc_l);
    VMC_Jacobian(&vmc_r);
    VMC_Force(&vmc_l, motor_lf->measure.torque, motor_lb->measure.torque, 0.09f);
    VMC_Force(&vmc_r, motor_rf->measure.torque, motor_rb->measure.torque, 0.09f);

    // 根据控制模式进行正运动学解算,计算底盘输出
    BalanceControlCalculate();
    BalanceTorqueCombine();

    DMMotorSetRef(motor_lf, joint_lf_tor);
    DMMotorSetRef(motor_lb, joint_lb_tor);
    DMMotorSetRef(motor_rf, -joint_rf_tor);
    DMMotorSetRef(motor_rb, -joint_rb_tor);

    DJIMotorSetRef(foot_motor_lf, foot_l_tor);
    DJIMotorSetRef(foot_motor_rf, -foot_r_tor);

 
 #ifdef CHASSIS_BOARD
     ;
 #endif // CHASSIS_BOARD
 
     // 推送反馈消息
 #ifdef ONE_BOARD
     PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
 #endif
 #ifdef CHASSIS_BOARD
     CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
 #endif // CHASSIS_BOARD
 }
 #endif 