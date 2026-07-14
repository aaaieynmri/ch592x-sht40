#ifndef __DRV_LCD_H__
#define __DRV_LCD_H__

#include "CH59x_common.h"
#include "CH59x_lcd.h"

// 电量图标枚举
typedef enum {
    BAT_EMPTY = 0,
    BAT_LOW   = 1, 
    BAT_MED   = 2, 
    BAT_HIGH  = 3  
} BatLevel;

// 显示数据结构体
typedef struct {
    float temperature;
    float humidity;
    BatLevel battery;
    BOOL show_minus; // 使用 WCH 的大写 BOOL
    BOOL is_celsius; 
} LCD_DisplayData;

// 函数声明：不再需要参数，配置直接写死在 .c 里
void DRV_LCD_Init(void);
void DRV_LCD_Refresh(const LCD_DisplayData *data);

#endif