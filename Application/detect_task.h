#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "main.h"

//�������Լ���Ӧ�豸˳��
enum errorList
{
    ROSTOE = 0,//ң����
    INSTOE,//������
    ChassisMotorLUTOE,
    ChassisMotorRUTOE,
    ChassisMotorRDTOE,
    ChassisMotorLDTOE,

    errorListLength,
};


typedef __packed struct
{
    uint32_t newTime;                 //���¼�¼ʱ��
    uint32_t lastTime;                //�ϴμ�¼ʱ��
    uint32_t Losttime;                //����ʱ��
    uint32_t worktime;                //����ʱ��
    uint32_t setOfflineTime : 12;     //��������ʱ����ֵ
    uint32_t setOnlineTime  : 12;     //��������ʱ����ֵ
    uint32_t enable : 1;               //�豸���ʹ�ܰ�ť
    uint32_t isLost : 1;               //�豸����״̬

} error_t;
    
extern void DetectTask(void);
extern void DetectInit(uint32_t time);
extern void DetectHook(uint8_t toe);
extern bool_t toe_is_error(uint8_t err);

#endif
