/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "main.h"
#include "CAN_receive.h"
#include "pid.h"

//控制系数
#define STD_TRANS_VX	-1
#define STD_TRANS_VY	 1
#define STD_TRANS_WZ	 1

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


//车身中心距轮子尺寸
#define MOTOR_DISTANCE_TO_CENTER 0.17f

//车身轴距
#define AXLE_LENGTH  0.16f

//陀螺仪到车身中心距离
#define GYROSCOPE_DISTANCE 0.10f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 4.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f

#define CHASSIS_WZ_SET_SCALE 0.0f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
//缓冲危险值30.0J
#define CHASSIS_POWER_BUFF_DANGER 30.0

#define SPEED_PID_MAX_OUT 10000.0f
#define SPEED_PID_MAX_IOUT 2000.0f

#define POS_PID_MAX_OUT   200.0f
#define POS_PID_MAX_IOUT  0.9f

#define FOLLOW_PID_MAX_OUT 6.0f
#define FOLLOW_PID_MAX_IOUT 0.2f

#define Sync_angle  0.0f
#define DG2Rad  57.3f
#define pi      3.14f
typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;
  float speed;
  float speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
	
  chassis_motor_t motor_chassis[4];          //底盘电机数据
  pid_type_def motor_speed_pid[4];             //底盘电机速度pid
  pid_type_def motor_pos_pid[4];               //底盘电机位置pid
  pid_type_def chassis_angle_pid;              //底盘跟随角度pid

//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值
//  first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓设定值
//  first_order_filter_type_t chassis_cmd_slow_set_wz;  //使用一阶低通滤波减缓设定值	

  float vx;                          //底盘速度 前进方向 前为正，单位 m/s
  float vy;                          //底盘速度 左右方向 左为正  单位 m/s
  float wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
	float last_vx_set;                 //底盘上次设定速度 前进方向 前为正，单位 m/s
  float last_vy_set;                 //底盘上次设定速度 左右方向 左为正，单位 m/s
  float last_wz_set;                 //底盘上次设定旋转角速度，逆时针为正 单位 rad/s
  float error_angle;
  float init_angle;

} chassis_move_t;
chassis_move_t *get_chassisMove_point(void);
const chassis_move_t* get_chassis_point(void);
void Chassis_init(void);
void chassis_ctrl(void);
#endif
