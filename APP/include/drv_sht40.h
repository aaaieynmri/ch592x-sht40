#ifndef __DRV_SHT40_H__
#define __DRV_SHT40_H__

#include "CH59x_common.h"

typedef struct {
    float temp;
    float humi;
    BOOL  valid;
} SHT40_Result;

void DRV_SHT40_Init(void);

// 【修改点 1】拆分函数接口
void DRV_SHT40_StartMeasure(void);        // 只负责发送测量命令
SHT40_Result DRV_SHT40_GetResult(void);   // 只负责读取并计算结果

#endif