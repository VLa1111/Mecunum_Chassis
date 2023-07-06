#include "bsp_ros.h"
#include "bsp_chassis.h"
#include "INS_task.h"
#include "detect_task.h"
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
void Recv_Error(void);
void Ros_Recv(uint8_t *data_buf, Ros_float_t *Receive)
{
        Receive->idenitity  =       (Ros_OwnID_e)data_buf[F_indentity];
        Receive->Order      =       (Tx_Order_e)data_buf[F_order];
        memcpy(&Receive->vx, &data_buf[F_vx], 4);
        memcpy(&Receive->vy, &data_buf[F_vy], 4);
        memcpy(&Receive->wz, &data_buf[F_wz], 4);

        Tx_flag = 1;
        //Recv_Error();
        DetectHook(ROSTOE);
    
}

void Ros_Upload()
{
    TxdTopic.idenitity = Ros_Sevent;
    TxdTopic.Order = Tx_Request;
    TxdTopic.vx = tx_conctret * get_chassisMove_point()->vx;
    TxdTopic.vy = get_chassisMove_point()->vy;
    TxdTopic.axis = INS.Yaw;
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

void Recv_Error(void)
{
    if (Recv_Data.vx > err_message_hor)
    {
        goto err;
    }
        if (Recv_Data.vy > err_message_hor)
    {
        goto err;
    }
        if (Recv_Data.wz > err_message_th || Recv_Data.wz < (-1 * err_message_th))
    {
        goto err;
    }
err:
    Recv_Data.vx = 0;
    Recv_Data.vy = 0;
    Recv_Data.wz = 0;
}

Ros_float_t *get_Recv_Data_Point(void)
{
    return &Recv_Data;
}
