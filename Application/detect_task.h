#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "main.h"

//错误码以及对应设备顺序
enum errorList
{
    ROSTOE = 0,//遥控器
    INSTOE,//陀螺仪
    ChassisMotorLUTOE,
    ChassisMotorRUTOE,
    ChassisMotorRDTOE,
    ChassisMotorLDTOE,

    errorListLength,
};


typedef __packed struct
{
    uint32_t newTime;                 //最新记录时间
    uint32_t lastTime;                //上次记录时间
    uint32_t Losttime;                //掉线时间
    uint32_t worktime;                //上线时间
    uint32_t setOfflineTime : 12;     //设置离线时间阈值
    uint32_t setOnlineTime  : 12;     //设置上线时间阈值
    uint32_t enable : 1;               //设备监测使能按钮
    uint32_t isLost : 1;               //设备掉线状态

} error_t;
    
extern void DetectTask(void);
extern void DetectInit(uint32_t time);
extern void DetectHook(uint8_t toe);
extern bool_t toe_is_error(uint8_t err);

#endif
