#include "infantry_config.h"

Robot_Config_s *InfantryConfigInit(uint8_t robot_id) 
{
    switch (robot_id)
    {
    /*******************************************************三号步兵**********************************************************************/
    case INFANTRY_ROBOT_3:
        static Robot_Config_s infantry_config_3 = { 
            .chassis_param = {
                .chassis_motor_id[MOTOR_LF] = 1,
                .chassis_motor_id[MOTOR_RF] = 2,
                .chassis_motor_id[MOTOR_RB] = 3,
                .chassis_motor_id[MOTOR_LB] = 4,

                .wheel_type = MECANUM_WHEEL,
                .wheel_measure = {
                    .center_gimbal_offset_x = 0.0,
                    .center_gimbal_offset_y = 0.0,
                    .wheel_base = 370,
                    .track_width = 420,
                    .radius_wheel = 75,
                    .reduction_ratio_wheel = 19.0,
                },

                .chassis_motor_config = {
                    .can_init_config.can_handle = &hcan1,
                    .controller_param_init_config = {
                        .speed_PID = {
                            .Kp = 5.0,
                            .Ki = 1.0,
                            .Kd = 0.0, 
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 12000,
                        },
                        .current_PID = {
                            .Kp = 0.5, 
                            .Ki = 0.0,   
                            .Kd = 0.0,
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                    },   
                    .controller_setting_init_config = {
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .outer_loop_type = SPEED_LOOP,
                        .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                    },
                    .motor_type = M3508,             
                }
            },


            .gimbal_param = {
                .gimbal_type = SINGLE_GIMBAL,
                .gimbal_offset = {
                    .yaw_offset = 2640,
                    .pitch_min_angle = -20.0f,
                    .pitch_max_angle = 35.0f,
                },

                .yaw_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan1,
                        .tx_id = 1,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            .Kp = 15.0,
                            .Ki = 0.0, 
                            .Kd = 0.7, 
                            .DeadBand = 0.1,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 500,
                        },
                        .speed_PID = {
                            .Kp = 300.0, 
                            .Ki = 150.0, 
                            .Kd = 0.0,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 3000,
                            .MaxOut = 20000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = OTHER_FEED,
                        .speed_feedback_source = OTHER_FEED,
                        .outer_loop_type = ANGLE_LOOP,
                        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                    },
                    .motor_type = GM6020
                },        

                .pitch_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan2,
                        .tx_id = 2,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            .Kp = 15.0,
                            .Ki = 0.0, 
                            .Kd = 0.2, 
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 500,
                        },
                        .speed_PID = {
                            .Kp = 250.0, 
                            .Ki = 100.0, 
                            .Kd = 0.0,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 2500,
                            .MaxOut = 20000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = OTHER_FEED,
                        .speed_feedback_source = OTHER_FEED,
                        .outer_loop_type = ANGLE_LOOP,
                        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                        .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                    },
                    .motor_type = GM6020
                }                
            },

            .shoot_param = {
                .friction_motor_id[MOTOR_LF] = 1,
                .friction_motor_id[MOTOR_RF] = 2,
                .friction_motor_reverse_flag[MOTOR_LF] = MOTOR_DIRECTION_NORMAL,
                .friction_motor_reverse_flag[MOTOR_RF] = MOTOR_DIRECTION_REVERSE,    

                .shoot_type = DUAL_SHOOT,
                .friction_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan2,
                    },
                    .controller_param_init_config = {
                        .speed_PID = {
                            .Kp = 10.0, 
                            .Ki = 1.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 10000,
                            .MaxOut = 25000,
                        },
                        .current_PID = {
                            .Kp = 0.4, 
                            .Ki = 0.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 10000,
                            .MaxOut = 15000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .outer_loop_type = SPEED_LOOP,
                        .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    },
                    .motor_type = M3508
                },

                .loader_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan2,
                        .tx_id = 5,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                            .Kp = 10.0, 
                            .Ki = 1.0,
                            .Kd = 0.0,
                            .MaxOut = 200, 
                        },
                        .speed_PID = {
                            .Kp = 10.0, 
                            .Ki = 1.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 8000,
                            .MaxOut = 10000,
                        },
                        .current_PID = {
                            .Kp = 0.4, 
                            .Ki = 0.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 5000,
                            .MaxOut = 5000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = MOTOR_FEED, 
                        .speed_feedback_source = MOTOR_FEED,
                        .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
                        .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    },
                    .motor_type = M2006 // 英雄使用M3508
                }
            }
        };
        return &infantry_config_3;
        break;
    /*******************************************************四号步兵**********************************************************************/
    case INFANTRY_ROBOT_4:
        static Robot_Config_s infantry_config_4 = { 
            .chassis_param = {
                .chassis_motor_id[MOTOR_LF] = 1,
                .chassis_motor_id[MOTOR_RF] = 2,
                .chassis_motor_id[MOTOR_RB] = 3,
                .chassis_motor_id[MOTOR_LB] = 4,

                .chassis_motor_id[MOTOR_UP_LF] = 5,
                .chassis_motor_id[MOTOR_UP_RF] = 4,

                .wheel_type = BALANCE_WHEEL,
                .wheel_measure = {
                    .center_gimbal_offset_x = 0.0,
                    .center_gimbal_offset_y = 0.0,
                    .wheel_base = 370,
                    .track_width = 420,
                    .radius_wheel = 0.09,
                    .reduction_ratio_wheel = 19.0,
                },

                .chassis_motor_config = {
                    .can_init_config.can_handle = &hcan1,
                    .controller_param_init_config = {
                        .speed_PID = {
                            .Kp = 5.0,
                            .Ki = 1.0,
                            .Kd = 0.0, 
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 12000,
                        },
                        .current_PID = {
                            .Kp = 0.5, 
                            .Ki = 0.0,   
                            .Kd = 0.0,
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                    },   
                    .controller_setting_init_config = {
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .outer_loop_type = CURRENT_LOOP,
                        .close_loop_type = CURRENT_LOOP | CURRENT_LOOP,
                    },
                    .motor_type = M3508,             
                }
            },


            .gimbal_param = {
                .gimbal_type = SINGLE_GIMBAL,
                .gimbal_offset = {
                    .yaw_offset = 2640,
                    .pitch_min_angle = -20.0f,
                    .pitch_max_angle = 35.0f,
                },

                .yaw_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan1,
                        .tx_id = 4,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            .Kp = 15.0,
                            .Ki = 0.0, 
                            .Kd = 0.7, 
                            .DeadBand = 0.1,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 500,
                        },
                        .speed_PID = {
                            .Kp = 300.0, 
                            .Ki = 150.0, 
                            .Kd = 0.0,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 3000,
                            .MaxOut = 20000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = OTHER_FEED,
                        .speed_feedback_source = OTHER_FEED,
                        .outer_loop_type = ANGLE_LOOP,
                        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                    },
                    .motor_type = GM6020
                },        

                .pitch_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan2,
                        .tx_id = 2,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            .Kp = 15.0,
                            .Ki = 0.0, 
                            .Kd = 0.2, 
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 500,
                        },
                        .speed_PID = {
                            .Kp = 250.0, 
                            .Ki = 100.0, 
                            .Kd = 0.0,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 2500,
                            .MaxOut = 20000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = OTHER_FEED,
                        .speed_feedback_source = OTHER_FEED,
                        .outer_loop_type = ANGLE_LOOP,
                        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                        .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                    },
                    .motor_type = GM6020
                }                
            },

            .shoot_param = {
                .friction_motor_id[MOTOR_LF] = 1,
                .friction_motor_id[MOTOR_RF] = 2,
                .friction_motor_reverse_flag[MOTOR_LF] = MOTOR_DIRECTION_NORMAL,
                .friction_motor_reverse_flag[MOTOR_RF] = MOTOR_DIRECTION_REVERSE,    

                .shoot_type = DUAL_SHOOT,
                .friction_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan2,
                    },
                    .controller_param_init_config = {
                        .speed_PID = {
                            .Kp = 10.0, 
                            .Ki = 1.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 10000,
                            .MaxOut = 25000,
                        },
                        .current_PID = {
                            .Kp = 0.4, 
                            .Ki = 0.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 10000,
                            .MaxOut = 15000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .outer_loop_type = SPEED_LOOP,
                        .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    },
                    .motor_type = M3508
                },

                .loader_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan2,
                        .tx_id = 5,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                            .Kp = 10.0, 
                            .Ki = 1.0,
                            .Kd = 0.0,
                            .MaxOut = 200, 
                        },
                        .speed_PID = {
                            .Kp = 10.0, 
                            .Ki = 1.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 8000,
                            .MaxOut = 10000,
                        },
                        .current_PID = {
                            .Kp = 0.4, 
                            .Ki = 0.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 5000,
                            .MaxOut = 5000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = MOTOR_FEED, 
                        .speed_feedback_source = MOTOR_FEED,
                        .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
                        .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    },
                    .motor_type = M2006 // 英雄使用M3508
                }
            }
        };
        return &infantry_config_4;
        break;
    /*******************************************************五号步兵**********************************************************************/
    case INFANTRY_ROBOT_5:
        break;
    default:
        break;
    }
} 

static Graph_Data_t UI_pitch_line[10]; // pitch准线
static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_graph_Arc[3]; // 功率圆弧
static Graph_Data_t UI_graph_Rectangle[3]; // 绘制射速矩形
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[7] = {460, 380, 300, 538, 538, 959};
void InfantryStaticUI(referee_info_t *referee_recv_info)
{
    // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Cyan, 2, 1029, shoot_line_location[0], 880, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_Cyan, 3, 750, shoot_line_location[1], 1160, shoot_line_location[1]);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Cyan, 4, 645, shoot_line_location[2], 1274, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Cyan, 4, 600, shoot_line_location[3], 920, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Cyan, 4, 1000, shoot_line_location[4], 1220, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Cyan, 4, shoot_line_location[5],300, shoot_line_location[5],505);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_shoot_line[5]);

    // 绘制pitch基准线
    UILineDraw(&UI_pitch_line[0], "so1", UI_Graph_ADD, 7, UI_Color_Cyan, 10, 1320, 538, 1341, 538);
    UILineDraw(&UI_pitch_line[1], "so2", UI_Graph_ADD, 7, UI_Color_Cyan, 15, 1307, 679, 1258, 651);
    UILineDraw(&UI_pitch_line[2], "so3", UI_Graph_ADD, 7, UI_Color_Cyan, 10, 1297, 600, 1329, 613);
    UILineDraw(&UI_pitch_line[3], "so4", UI_Graph_ADD, 7, UI_Color_Cyan, 15, 1191, 768, 1227, 804);
    UILineDraw(&UI_pitch_line[4], "so5", UI_Graph_ADD, 7, UI_Color_Cyan, 10, 1254, 725, 1282, 742);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_pitch_line[0], UI_pitch_line[1], UI_pitch_line[2], UI_pitch_line[3], UI_pitch_line[4]); 

    // 绘制功率圆弧
    UIArcDraw(&UI_graph_Arc[0], "ss0", UI_Graph_ADD, 7, UI_Color_Cyan, 45, 135, 6, 950, 540, 400, 400);
    UIArcDraw(&UI_graph_Arc[1], "ss1", UI_Graph_ADD, 7, UI_Color_Cyan, 225, 270, 15, 960, 530, 400, 400);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_graph_Arc[0], UI_graph_Arc[1]);
    UIArcDraw(&UI_graph_Arc[2], "ss2", UI_Graph_ADD, 7, UI_Color_Cyan, 9, 300, 15, 1524, 725, 100, 100);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_graph_Arc[2]);

    // 绘制射速矩形
    UIRectangleDraw(&UI_graph_Rectangle[0], "sj1", UI_Graph_ADD, 7, UI_Color_Cyan, 4, 46, 834, 136, 858);
    UIRectangleDraw(&UI_graph_Rectangle[1], "sj2", UI_Graph_ADD, 7, UI_Color_Cyan, 5, 1481, 672, 1507, 780);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_graph_Rectangle[0], UI_graph_Rectangle[1]);
    UIRectangleDraw(&UI_graph_Rectangle[2], "sj3", UI_Graph_ADD, 7, UI_Color_Cyan, 5, 1555, 672, 1581, 780);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_graph_Rectangle[2]);
}
void InfantryDynamciscUI(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    // shoot
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        // UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
        // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);

        UIRectangleDraw(&UI_graph_Rectangle[0], "sj1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 4, 46, 834, 136, 858);
        UIRectangleDraw(&UI_graph_Rectangle[1], "sj2", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5, 1481, 672, 1507, 780);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_graph_Rectangle[0], UI_graph_Rectangle[1]);
        UIRectangleDraw(&UI_graph_Rectangle[2], "sj3", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5, 1555, 672, 1581, 780);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_graph_Rectangle[2]);
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
}