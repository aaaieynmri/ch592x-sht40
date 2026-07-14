#ifndef __DRV_ADC_H__
#define __DRV_ADC_H__

#include "CH59x_common.h"

/**
 * @brief  获取当前电池电压
 * @return 电压值，单位 mV (例如 3120 代表 3.12V)
 */
uint16_t DRV_ADC_GetVbat(void);

#endif