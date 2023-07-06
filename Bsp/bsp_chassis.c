#include "bsp_chassis.h"
#include "pid.h"
#include "CAN_receive.h"
#include "usart.h"
#include "bsp_ros.h"
#include "ins_task.h"
#include "detect_task.h"
#define easy_abs(x) ((x) > 0 ? (x) : (-x))

// ���̸����ֳ�ʼ��
static void chassis_pid_init(void);
static void chassis_get_data_init(Ros_float_t *init);
// ���̿�����ؾ�̬����
static void chassis_feedback_update(Ros_float_t *Recv_Data);
static void chassis_set_contorl(Ros_float_t *RosCmd);
static void chassis_control_loop(void);
// ���ӿ��ƺ���
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
static void chassis_speed_control(void);
// �����˶�����
chassis_move_t chassis_move;
//�涨��תΪ�� ��תΪ��
float get_Error_Angle(Ros_float_t *recvData)
{
  float errAngle = 0;
  float cmdrelaAngle = recvData->wz;
  if (cmdrelaAngle >= 0)
  {
    errAngle = INS.Yaw - cmdrelaAngle;
  }

  if (cmdrelaAngle < 0)
  {
    errAngle = INS.Yaw - cmdrelaAngle;
  }

  return errAngle;
}

void Chassis_init(void)
{
  // ����һ��ʱ��
  HAL_Delay(200);
  // ���̳�ʼ��
  Ros_float_t *Recv_init = get_Recv_Data_Point();
  chassis_pid_init();
  chassis_get_data_init(Recv_init);
}

void chassis_ctrl(void)
{
  Ros_float_t *rosHook = get_Recv_Data_Point();

    // �������ݸ���
    chassis_feedback_update(rosHook);
    // ���̿���������
    chassis_set_contorl(rosHook);
    // ���̿���PID����
    chassis_control_loop();
    //������ֵ
    CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                    chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
  
  HAL_Delay(CHASSIS_CONTROL_TIME_MS);
}

static void chassis_set_contorl(Ros_float_t *RosCmd)
{
  while (toe_is_error(ROSTOE))
  {
      chassis_move.vx_set = 0;
      chassis_move.vy_set = 0;
      chassis_move.wz_set = 0;
  }
  // ������λ��ָ������&chassis_move.vx_set,&chassis_move.vy_set,&chassis_move.wz_set	
  chassis_move.vx_set = RosCmd->vx * STD_TRANS_VX;
  chassis_move.vy_set = RosCmd->vy * STD_TRANS_VY;
  chassis_move.wz_set = RosCmd->wz * STD_TRANS_WZ;
}

static void chassis_pid_init(void)
{
  // �����ٶȻ�pidֵ
  const static fp32 speed_pid[3] = {2000.0f, 5.0f, 20};
	// ���λ�û�pidֵ
	const static fp32 position_pid[3] = {10.0f, 0, 0};
  // ���̽Ƕ�pidֵ
  const static fp32 follow_pid[3] = {0.08f, 0.1f, 0.0f};
  uint8_t i;
  for (i = 0; i < 4; i++)
	{
    PID_init(&chassis_move.motor_speed_pid[i], PID_POSITION, speed_pid,
             SPEED_PID_MAX_OUT, SPEED_PID_MAX_IOUT);
			
		PID_init(&chassis_move.motor_pos_pid[i], PID_POSITION, position_pid,
             POS_PID_MAX_OUT, POS_PID_MAX_IOUT);
	}

  // ��ʼ���Ƕ�PID
  PID_init(&chassis_move.chassis_angle_pid, PID_POSITION, follow_pid,
           FOLLOW_PID_MAX_OUT, FOLLOW_PID_MAX_IOUT);
}

static void chassis_get_data_init(Ros_float_t *init)
{
  uint8_t i;
  // ���̿���״̬Ϊԭʼ
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
  }
  chassis_move.init_angle = INS.Yaw;
}

static void chassis_feedback_update(Ros_float_t *Recv_Data)
{
  fp32 speed[4];
  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    // ���µ���ٶȣ����ٶ����ٶȵ�PID΢��
    chassis_move.motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move.motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move.motor_chassis[i].accel = chassis_move.motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    speed[i] = chassis_move.motor_chassis[i].speed;
  }
  // ���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
  chassis_move.vx = (-speed[0] + speed[1] + speed[2] - speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move.vy = (-speed[0] - speed[1] + speed[2] + speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move.wz = (-speed[0] - speed[1] - speed[2] - speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
  // ��ȡ���Ƕ�
  chassis_move.error_angle = get_Error_Angle(Recv_Data);

  chassis_move.last_vx_set = chassis_move.vx_set;
  chassis_move.last_vy_set = chassis_move.vy_set;
  chassis_move.last_wz_set = chassis_move.wz_set;
}

static void chassis_control_loop(void)
{
  fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;
  chassis_move.wz_set = PID_calc(&chassis_move.chassis_angle_pid, chassis_move.error_angle, Sync_angle);
  // �����˶��ֽ�
  chassis_vector_to_mecanum_wheel_speed(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_speed);
  for (i = 0; i < 4; i++)
    chassis_move.motor_chassis[i].speed_set = wheel_speed[i];
  chassis_speed_control();
  // ����pid
  for (i = 0; i < 4; i++)
    PID_calc(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].speed, chassis_move.motor_chassis[i].speed_set);

  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].give_current = (int16_t)chassis_move.motor_speed_pid[i].out;
  }
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
  // ��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
  wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER  * wz_set;
  wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER   * wz_set;
  wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER  * wz_set;
  wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

static void chassis_speed_control(void)
{
  uint8_t i = 0;
  fp32 max_vector = 0.0f, vector_rate = 0.0f;
  fp32 temp = 0.0f;
  // �������ӿ�������ٶȣ�������������ٶ�
  for (i = 0; i < 4; i++)
  {
    temp = easy_abs(chassis_move.motor_chassis[i].speed_set);
    if (max_vector < temp)
      max_vector = temp;
  }
  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
      chassis_move.motor_chassis[i].speed_set *= vector_rate;
  }
}

chassis_move_t *get_chassisMove_point(void)
{
  return &chassis_move;
}

