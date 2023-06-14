#ifndef BSP_ROS_H
#define BSP_ROS_H

#include "main.h"
#include "bsp_chassis.h"



//数据帧
#define F_indentity  	0
#define F_order	  	 	1
#define F_vx			2
#define F_vy			3
#define F_wz			4
#define F_axis			8
//机器人ID
typedef enum
{
	Ros_Sevent = 1,
	Ros_Core   = 1,
	Ros_End    = 0x0D

}Ros_OwnID_e;

//允许发送位
typedef enum
{
	Tx_Request = 1,
	Tx_Allow	 = 1,
}Tx_Order_e;


typedef enum
{
	vx_mode_stop   = 0,
	vx_mode_normal = 1,
	vx_mode_dash   = 2,
}Vx_Speed_e;

typedef enum
{
	vy_mode_stop   = 0,
	vy_mode_normal = 1,
	vy_mode_dash   = 2,
}Vy_Speed_e;

//数据协议结构体
#pragma pack(1)
typedef struct {
	Ros_OwnID_e	idenitity;
	Tx_Order_e	Order;
	float vx;
	float vy;
	float axis;
	uint8_t frameEnd;
}dataFrame_t;


typedef struct 
{
	/* data */
	Ros_OwnID_e	idenitity;
	Tx_Order_e		Order;
	uint8_t 		   vx;
	uint8_t 		   vy;
	float   		   wz;
}Ros_float_t;

typedef struct {

  const chassis_move_t    *chassis_point;
  dataFrame_t             *dataStream_point;
  const Ros_float_t			  *floatData_point;
	
}Ros_t;

extern Ros_t RecvTopic;
Ros_float_t* get_Recv_Data_Point(void);
void Message_Init(void);
void Ros_Upload(void);
void Ros_Transmit(void);
void Ros_Recv(uint8_t *data_buf, Ros_float_t* Receive);
#endif

