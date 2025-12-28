#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#pragma pack(1)
typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

/* 接收数据段 */
typedef struct
{
	Target_Type_e target_type;
	Target_State_e target_state;
	struct
	{
		float pitch;
		float yaw;
		float distance;
	} vision_data;

	struct
	{
		float vx;
		float vy;
		float wz;
	} navigation_data;
} RobotPC_Recv_s;


typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef enum
{
	COLOR_BLUE = 0,
	COLOR_RED = 1,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

/* 发送数据段 */
typedef struct
{
	Enemy_Color_e enemy_color;
	struct
	{
		float roll;
		float pitch;
		float yaw;
	} imu_data;

} RobotPC_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和上位机的串口通信
 *
 * @param handle 用于和上位机通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
RobotPC_Recv_s *RobotPCInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送给上位机数据
 *
 */
void RobotPCSend();

/**
 * @brief 设置上位机发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void PCSetFlag(Enemy_Color_e enemy_color, Work_Mode_e set_mode);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param roll
 * @param pitch
 * @param yaw
 */
void PCSendIMU(float yaw, float pitch, float roll);

#endif // !MASTER_PROCESS_H