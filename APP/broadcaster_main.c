/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 广播应用主函数及任务系统初始化
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "HAL.h"
#include "broadcaster.h"
#include "drv_sht40.h"
#include "drv_lcd.h"

// 添加 BLE 内存池定义
__attribute__((aligned(4)))
uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

int main(void)
{
    /* 1. 开启 DCDC (确保 CONFIG.h 中 DCDC_ENABLE 宏已设置为 TRUE) */
#if(defined(DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif

    /* 2. 系统时钟 */
    

    // 进入安全访问模式
    R8_SAFE_ACCESS_SIG = 0x57;
    R8_SAFE_ACCESS_SIG = 0xA8;

    /*==================== HSE 32MHz 配置 ====================*/
    HSECFG_Current(HSE_RCur_75);
    HSECFG_Capacitance(HSECap_20p);

    /*==================== LSE 32k 配置 ====================*/
    LSECFG_Current(LSE_RCur_70);
    LSECFG_Capacitance(LSECap_20p);

    // 进入安全访问模式
    R8_SAFE_ACCESS_SIG = 0x57;
    R8_SAFE_ACCESS_SIG = 0xA8;
    
    /* 切换系统时钟 */
    SetSysClock(CLK_SOURCE_HSE_16MHz);
    /* RTC使用外部32k晶振 */
    LClk32K_Select(Clk32K_LSE);

#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    // GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    // GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
#endif

#ifdef DEBUG
    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
#endif
    PRINT("%s\n", VER_LIB);
    

    
    CH59x_BLEInit();

    HAL_Init();
    
    DRV_LCD_Init();    

    DRV_SHT40_Init();  

    GAPRole_BroadcasterInit();
    
    Broadcaster_Init();
    
    while(1)
    {
        TMOS_SystemProcess();
    }
}
/******************************** endfile @ main ******************************/
