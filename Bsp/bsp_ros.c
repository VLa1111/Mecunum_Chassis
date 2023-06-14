#include "bsp_ros.h"
#include "bsp_chassis.h"
#include "INS_task.h"
#include "stdio.h"
#include "usart.h"
#include "main.h"
#include "math.h"
typedef unsigned char byte;
Ros_t RecvTopic;
dataFrame_t TxdTopic;
uint8_t Tx_flag = 0;
Ros_float_t Recv_Data;
uint32_t rx1_buf[19] = {0}, rx2_buf[19] = {0};

void Ros_Recv(uint8_t *data_buf, Ros_float_t *Receive)
{
        Receive->idenitity  =       (Ros_OwnID_e)data_buf[F_indentity];
        Receive->Order      =       (Tx_Order_e)data_buf[F_order];
        Receive->vx         =       (Vx_Speed_e)data_buf[F_vx];
        Receive->vy         =       (Vy_Speed_e)data_buf[F_vy];

        memcpy(&Receive->wz, &data_buf[F_wz], 4);

        Tx_flag = 1;
    
}

void Ros_Upload()
{
    TxdTopic.idenitity = Ros_Sevent;
    TxdTopic.Order = Tx_Request;
    TxdTopic.vx = get_chassisMove_point()->vx;
    TxdTopic.vy = get_chassisMove_point()->vy;
		TxdTopic.axis = INS.Yaw;
    // memcpy(&upload->dataStream_point->vx,    &get_chassisMove_point()->vx, 4);
    // memcpy(&upload->dataStream_point->vy,    &get_chassisMove_point()->vy, 4);
    // memcpy(&upload->dataStream_point->wz,    &get_chassisMove_point()->wz, 4);
    // memcpy(&upload->dataStream_point->axis,  &INS.Yaw, 4);
    TxdTopic.frameEnd = Ros_End;
}

void ROS_DMA_TX(Tx_Order_e Order)
{
    if (Order == Tx_Allow)
    {
        if (HAL_UART_Transmit(&huart1, TxBuffer, TX_MAX_LEN, 1000) != HAL_OK)
        {
            Error_Handler();
        }
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET)
            ;
        Tx_flag = 0;
        
    }
}

void Ros_Transmit()
{

    memcpy(&TxBuffer[0], &TxdTopic, sizeof(TxdTopic));
	
        ROS_DMA_TX(Recv_Data.Order);
		//printf("%.02f,%.02f\r\n",INS.Yaw,Recv_Data.wz);
}

void Message_Init()
{
		memset(&TxdTopic,  0, sizeof(TxdTopic));
    float init_value = 12345;
    // 数据初始化
    TxdTopic.idenitity = Ros_Sevent;
    TxdTopic.Order = Tx_Request;
		TxdTopic.vx = init_value;
    TxdTopic.vy = init_value;
    TxdTopic.axis = init_value;
		TxdTopic.frameEnd = Ros_End;
}

Ros_float_t *get_Recv_Data_Point(void)
{
    return &Recv_Data;
}
