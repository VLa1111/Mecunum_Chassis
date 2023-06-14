/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "bsp_ros.h"
#include "bsp_chassis.h"
#include "stdio.h"
#include "string.h"
uint8_t aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���

unsigned char RxBuffer[RX_MAX_LEN];
unsigned char TxBuffer[TX_MAX_LEN];

unsigned char pidRxBuffer[RXBUFFERSIZE];
uint16_t Rxline1 = 0;
uint16_t RxLine = 0;
uint8_t  DataBuff[100];
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		HAL_UART_Receive_IT(&huart1, aRxBuffer, 1);
  /* USER CODE END USART1_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*
 * ������DataBuff�е�����
 * ���ؽ����õ�������
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // ��¼����λ��ʼ�ĵط�
    uint8_t data_End_Num = 0; // ��¼����λ�����ĵط�
    uint8_t data_Num = 0; // ��¼����λ��
    uint8_t minus_Flag = 0; // �ж��ǲ��Ǹ���
    float data_return = 0; // �����õ�������
    for(uint8_t i=0;i<200;i++) // ���ҵȺź͸�̾�ŵ�λ��
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1��ֱ�Ӷ�λ��������ʼλ
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // ����Ǹ���
    {
        data_Start_Num += 1; // ����һλ������λ
        minus_Flag = 1; // ����flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // ���ݹ�4λ
    {
        data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                (DataBuff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // ���ݹ�5λ
    {
        data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                (DataBuff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // ���ݹ�6λ
    {
        data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
//    printf("data=%.2f\r\n",data_return);
    return data_return;
}


/*
 * ���ݴ�����Ϣ����PID����
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
  chassis_move_t *chassis_move = get_chassisMove_point();
	Ros_float_t *axis = get_Recv_Data_Point();
   float data_Get = Get_Data(); // ��Ž��յ�������
//    printf("data=%.2f\r\n",data_Get);
     if(Motor_n == 1)//��ߵ��
   {
       if(DataBuff[0]=='P' && DataBuff[1]=='2') // λ�û�P
           chassis_move->chassis_angle_pid.Kp = data_Get;
       else if(DataBuff[0]=='I' && DataBuff[1]=='2') // λ�û�I
           chassis_move->chassis_angle_pid.Ki = data_Get;
       else if(DataBuff[0]=='D' && DataBuff[1]=='2') // λ�û�D
           chassis_move->chassis_angle_pid.Kd = data_Get;
       if(DataBuff[0]=='P' && DataBuff[1]=='1') // �ٶȻ�P
					chassis_move->motor_speed_pid[0].Kp = data_Get;
       else if(DataBuff[0]=='I' && DataBuff[1]=='1') // �ٶȻ�I
					chassis_move->motor_speed_pid[0].Ki = data_Get;
       else if(DataBuff[0]=='D' && DataBuff[1]=='1') // �ٶȻ�D
					chassis_move->motor_speed_pid[0].Kd = data_Get;
       else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //Ŀ���ٶ�
          chassis_move->motor_chassis[0].speed_set = data_Get;
   }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance==USART1)//����Ǵ���1
	{
    Rxline1++;
    RxBuffer[Rxline1-1]=aRxBuffer[0];
    if(aRxBuffer[0]==0x0D)
    {
      Ros_Recv(&RxBuffer[0], get_Recv_Data_Point());	
      memset(RxBuffer,0,sizeof(RxBuffer));
      Rxline1=0;
    }
		HAL_UART_Receive_IT(&huart1, aRxBuffer, 1);
    
	}
    if(huart->Instance==USART6)//����Ǵ���2
    {
        RxLine++;                      //ÿ���յ�һ�����ݣ�����ص����ݳ��ȼ�1
        DataBuff[RxLine-1]=pidRxBuffer[0];  //��ÿ�ν��յ������ݱ��浽��������
        if(pidRxBuffer[0]==0x21)            //���ս�����־λ��������ݿ����Զ��壬����ʵ����������ֻ��ʾ��ʹ�ã���һ����0x21
        {
            USART_PID_Adjust(1);//���ݽ����Ͳ�����ֵ����
            memset(DataBuff,0,sizeof(DataBuff));  //��ջ�������
            RxLine=0;  //��ս��ճ���
        }
        HAL_UART_Receive_IT(&huart6, (uint8_t *)pidRxBuffer, 1); //ÿ����һ�����ݣ��ʹ�һ�δ����жϽ��գ�����ֻ�����һ�����ݾ�ֹͣ����
    }
}	

#pragma import(__use_no_semihosting)  
   
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
#if !PID_Debug  
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0){};//ѭ������,ֱ���������   
    USART1->DR = (uint8_t) ch;      
	return ch;
}
#elif PID_Debug  
int fputc(int ch, FILE *f)
{      
	while((USART6->SR&0X40)==0){};//ѭ������,ֱ���������   
    USART6->DR = (uint8_t) ch;      
	return ch;
}
#endif 
/* USER CODE END 1 */
