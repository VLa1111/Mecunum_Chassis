/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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

//����ϵ��
#define STD_TRANS_VX	-1
#define STD_TRANS_VY	 1
#define STD_TRANS_WZ	 1

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


//�������ľ����ӳߴ�
#define MOTOR_DISTANCE_TO_CENTER 0.17f

//�������
#define AXLE_LENGTH  0.16f

//�����ǵ��������ľ���
#define GYROSCOPE_DISTANCE 0.10f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//�����������Ƶ��
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 4.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f

#define CHASSIS_WZ_SET_SCALE 0.0f

//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
//����Σ��ֵ30.0J
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
	
  chassis_motor_t motor_chassis[4];          //���̵������
  pid_type_def motor_speed_pid[4];             //���̵���ٶ�pid
  pid_type_def motor_pos_pid[4];               //���̵��λ��pid
  pid_type_def chassis_angle_pid;              //���̸���Ƕ�pid

//  first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ
//  first_order_filter_type_t chassis_cmd_slow_set_vy;  //ʹ��һ�׵�ͨ�˲������趨ֵ
//  first_order_filter_type_t chassis_cmd_slow_set_wz;  //ʹ��һ�׵�ͨ�˲������趨ֵ	

  float vx;                          //�����ٶ� ǰ������ ǰΪ������λ m/s
  float vy;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  float wz;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float vx_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  float vy_set;                      //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float wz_set;                      //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	float last_vx_set;                 //�����ϴ��趨�ٶ� ǰ������ ǰΪ������λ m/s
  float last_vy_set;                 //�����ϴ��趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float last_wz_set;                 //�����ϴ��趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float error_angle;
  float init_angle;

} chassis_move_t;
chassis_move_t *get_chassisMove_point(void);
const chassis_move_t* get_chassis_point(void);
void Chassis_init(void);
void chassis_ctrl(void);
#endif
