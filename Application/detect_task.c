#include "detect_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/*------------------------------------------------------*/
/*                     定义监控设备结构体               */
/*------------------------------------------------------*/
static error_t errorList[errorListLength];

//掉线判断任务
void DetectTask(void)
{
    static uint8_t i;
    static uint32_t systemTime;

    systemTime = xTaskGetTickCount();

    for (i = 0; i < errorListLength; i++)
    {
        // 未使能，跳过设备监测
        if (errorList[i].enable == 0)
        {
            continue;
        }

        // 刚上线排除
        else if (systemTime - errorList[i].worktime < errorList[i].setOnlineTime)
        {
            errorList[i].isLost = 0;
        }

        // 超时掉线
        else if (systemTime - errorList[i].newTime > errorList[i].setOfflineTime)
        {
            if (errorList[i].isLost == 0)
            {
                // 记录错误以及掉线时间
                errorList[i].isLost = 1;
                errorList[i].Losttime = systemTime;
            }
        }
        // 正常工作
        else
        {
            errorList[i].isLost = 0;
        }
    }

    vTaskDelay(5);
}
/*------------------------------------------------------*/
/*                       离线检测初始化                 */
/*------------------------------------------------------*/
void DetectInit(uint32_t time)
{
    //离线时间阈值 ，上线时间阈值，使能状态
    uint16_t setItem[errorListLength][3] =
    {
        {40, 0, 1},   //ROS
        {40, 0, 1},   //INS
        {10, 0, 1},   //35081
        {10, 0, 1},   //35082
        {10, 0, 1},   //35083
        {10, 0, 1},   //35084
    };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
        errorList[i].setOfflineTime = setItem[i][0];
        errorList[i].setOnlineTime =  setItem[i][1];
        errorList[i].enable = setItem[i][2];

        errorList[i].isLost = 1;
        errorList[i].newTime =  time;
        errorList[i].lastTime = time;
        errorList[i].Losttime = time;
        errorList[i].worktime = time;
    }

}


/*------------------------------------------------------*/
/*                   返回对应的设备是否丢失             */
/*------------------------------------------------------*/

bool_t toe_is_error(uint8_t err)
{
    return (errorList[err].isLost == 1);
}

/*------------------------------------------------------*/
/*                   设备接收数据钩子函数               */
/*------------------------------------------------------*/

void DetectHook(uint8_t toe)
{
    errorList[toe].lastTime = errorList[toe].newTime;
    errorList[toe].newTime  = xTaskGetTickCount();

    //更新丢失情况
    if (errorList[toe].isLost)
    {
        errorList[toe].isLost   = 0;
        errorList[toe].worktime = errorList[toe].newTime;
    }
}
