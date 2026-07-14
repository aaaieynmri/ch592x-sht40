#include "drv_adc.h"

uint16_t DRV_ADC_GetVbat(void) {
    uint32_t raw;
    
    // 1. 初始化内部电池电压采样通道
    ADC_InterBATSampInit(); 
    
    // 2. 配置时钟：采样频率降到最低以保证精度
    ADC_SampClkCfg(SampleFreq_3_2); 
    
    // 3. 执行单次转换
    raw = ADC_ExcutSingleConver();
    
    // 4. 关闭 ADC 电源以省电
    ADC_DisablePower(); 
    
    if (raw == 0) return 0;
    
    // 【换算逻辑】
    // CH592 内部固定 1/4 分压，ADC 基准 1.05V (1050mV)，12位精度 (4096)
    // 电压 = (raw / 4096) * 1050 * 4
    // 简化公式：(raw * 4200) / 4096
    return (uint16_t)((raw * 4200) / 4096);
}