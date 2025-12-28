#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

static Shoot_Config_s *shoot_config;

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_lf, *friction_rf, *loader; // 拨盘电机
static DJIMotorInstance *friction_lb, *friction_rb;
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

#define ONE_BULLET_DELTA_ANGLE 9400    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量

void ShootInit()
{
    shoot_config = ShootConfigFeed();

    // 左前摩擦轮
    shoot_config->friction_motor_config.can_init_config.tx_id = shoot_config->friction_motor_id[MOTOR_LF];
    shoot_config->friction_motor_config.controller_setting_init_config.motor_reverse_flag = shoot_config->friction_motor_reverse_flag[MOTOR_LF];       
    friction_lf = DJIMotorInit(&shoot_config->friction_motor_config);

    // 右前摩擦轮
    shoot_config->friction_motor_config.can_init_config.tx_id = shoot_config->friction_motor_id[MOTOR_RF];
    shoot_config->friction_motor_config.controller_setting_init_config.motor_reverse_flag = shoot_config->friction_motor_reverse_flag[MOTOR_RF];   
    friction_rf = DJIMotorInit(&shoot_config->friction_motor_config);

    switch (shoot_config->shoot_type)
    {
    case DUAL_SHOOT:
        break;
    case THREE_SHOOT:
        break;
    case FOUR_SHOOT:
        // 左后摩擦轮
        shoot_config->friction_motor_config.can_init_config.tx_id = shoot_config->friction_motor_id[MOTOR_LB];
        shoot_config->friction_motor_config.controller_setting_init_config.motor_reverse_flag = shoot_config->friction_motor_reverse_flag[MOTOR_LB];       
        friction_lb = DJIMotorInit(&shoot_config->friction_motor_config);
        // 右后摩擦轮
        shoot_config->friction_motor_config.can_init_config.tx_id = shoot_config->friction_motor_id[MOTOR_RB];
        shoot_config->friction_motor_config.controller_setting_init_config.motor_reverse_flag = shoot_config->friction_motor_reverse_flag[MOTOR_RB];   
        friction_rb = DJIMotorInit(&shoot_config->friction_motor_config);        
        break;
    default:
        break;
    }

    // 拨盘电机
    loader = DJIMotorInit(&shoot_config->loader_motor_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF || shoot_cmd_recv.shoot_mode == SHOOT_ZERO_FORCE)
    {
        DJIMotorStop(friction_lf);
        DJIMotorStop(friction_rf);
        switch (shoot_config->shoot_type)
        {
        case THREE_SHOOT:
            break;
        case FOUR_SHOOT:
            // 左后摩擦轮
            DJIMotorStop(friction_lb);
            // 右后摩擦轮
            DJIMotorStop(friction_rb);      
            break;
        default:
            break;
        }

        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_lf);
        DJIMotorEnable(friction_rf);
        switch (shoot_config->shoot_type)
        {
        case THREE_SHOOT:
            break;
        case FOUR_SHOOT:
            // 左后摩擦轮
            DJIMotorEnable(friction_lb);
            // 右后摩擦轮
            DJIMotorEnable(friction_rb);      
            break;
        default:
            break;
        }

        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.angle_single_round + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 150;                                                                    // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                         
        DJIMotorSetRef(loader, loader->measure.angle_single_round + ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 500;                                                                        // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        // ...
        break;
    default:
        // while (1)
        //     ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_lf, 0);
            DJIMotorSetRef(friction_rf, 0);
            switch (shoot_config->shoot_type)
            {
            case THREE_SHOOT:
                break;
            case FOUR_SHOOT:
                // 左后摩擦轮
                DJIMotorSetRef(friction_lb, 0);
                // 右后摩擦轮
                DJIMotorSetRef(friction_rb, 0);  
                break;
            default:
                break;
            }
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_lf, 0);
            DJIMotorSetRef(friction_rf, 0);
            switch (shoot_config->shoot_type)
            {
            case THREE_SHOOT:
                break;
            case FOUR_SHOOT:
                // 左后摩擦轮
                DJIMotorSetRef(friction_lb, 0);
                // 右后摩擦轮
                DJIMotorSetRef(friction_rb, 0);  
                break;
            default:
                break;
            }
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_lf, 0);
            DJIMotorSetRef(friction_rf, 0);
            switch (shoot_config->shoot_type)
            {
            case THREE_SHOOT:
                break;
            case FOUR_SHOOT:
                // 左后摩擦轮
                DJIMotorSetRef(friction_lb, 0);
                // 右后摩擦轮
                DJIMotorSetRef(friction_rb, 0);  
                break;
            default:
                break;
            }
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_lf, 80000);
            DJIMotorSetRef(friction_rf, 80000);
            switch (shoot_config->shoot_type)
            {
            case THREE_SHOOT:
                break;
            case FOUR_SHOOT:
                // 左后摩擦轮
                DJIMotorSetRef(friction_lb, 50000);
                // 右后摩擦轮
                DJIMotorSetRef(friction_rb, 50000);  
                break;
            default:
                break;
            }            
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_lf, 0);
        DJIMotorSetRef(friction_rf, 0);
        switch (shoot_config->shoot_type)
        {
        case THREE_SHOOT:
            break;
        case FOUR_SHOOT:
            // 左后摩擦轮
            DJIMotorSetRef(friction_lb, 0);
            // 右后摩擦轮
            DJIMotorSetRef(friction_rb, 0);  
            break;
        default:
            break;
        }            
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}