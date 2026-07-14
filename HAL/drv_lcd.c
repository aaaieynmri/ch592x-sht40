#include "drv_lcd.h"
#include <math.h>

// 7段数码管字库: A=bit0, B=bit1, C=bit2, D=bit3, E=bit4, F=bit5, G=bit6
static const uint8_t font7seg[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

typedef struct {
    uint8_t com;
    uint8_t seg;
} lcd_pixel_t;

// ===== 屏幕段码坐标映射字典 =====
// 温度栏：第2/3/4位数字段码坐标 {COM, 用户SEG}
static const lcd_pixel_t temp_d2[7] = {{3,3}, {2,3}, {1,3}, {0,2}, {1,2}, {3,2}, {2,2}};
static const lcd_pixel_t temp_d3[7] = {{3,5}, {2,5}, {1,5}, {0,4}, {1,4}, {3,4}, {2,4}};
static const lcd_pixel_t temp_d4[7] = {{3,12}, {2,12}, {1,12}, {0,11}, {1,11}, {3,11}, {2,11}};

// 湿度栏：第1/2位数字段码坐标 {COM, 用户SEG}
static const lcd_pixel_t hum_d1[7] = {{0,8}, {1,8}, {2,8}, {3,7}, {2,7}, {0,7}, {1,7}};
static const lcd_pixel_t hum_d2[7] = {{0,10}, {1,10}, {2,10}, {3,9}, {2,9}, {0,9}, {1,9}};


// ==========================================
// 核心像素驱动函数
// ==========================================
static void lcd_set_pixel(uint8_t com, uint8_t user_seg, uint8_t on) {
    uint8_t hw_seg;
    
    // 将用户定义的 SEG 编号映射到硬件真实 SEG 编号
    switch(user_seg) {
        case 1:  hw_seg = 4; break;  // PA4
        case 2:  hw_seg = 5; break;  // PA5
        case 3:  hw_seg = 6; break;  // PA15
        case 4:  hw_seg = 7; break;  // PA14
        case 5:  hw_seg = 8; break;  // PA13
        case 6:  hw_seg = 9; break;  // PA12
        case 7:  hw_seg = 16; break; // PB6
        case 8:  hw_seg = 17; break; // PB0
        case 9:  hw_seg = 14; break; // PB11
        case 10: hw_seg = 15; break; // PB10
        case 11: hw_seg = 0; break;  // PB7
        case 12: hw_seg = 1; break;  // PB4
        case 13: hw_seg = 2; break;  // PB23
        default: return;
    }

    // ★ 关键修改点：新版硬件已修正走线，彻底废弃 com = 3 - com; 的反转逻辑

    // 根据硬件 SEG 编号定位显存寄存器 (CH592 包含 RAM0, RAM1, RAM2)
    volatile uint32_t *ram;
    if (hw_seg < 8) {
        ram = &R32_LCD_RAM0;
    } else if (hw_seg < 16) {
        ram = &R32_LCD_RAM1; 
        hw_seg -= 8;
    } else {
        ram = &R32_LCD_RAM2; 
        hw_seg -= 16;
    }

    uint32_t mask = 1 << (hw_seg * 4 + com);
    if (on) *ram |= mask;
    else    *ram &= ~mask;
}

// 在指定区域绘制一位数字 (digit < 0 时消隐)
static void lcd_draw_digit(const lcd_pixel_t* mapping, int8_t digit) {
    uint8_t font = (digit >= 0 && digit <= 9) ? font7seg[digit] : 0x00;
    for (int i = 0; i < 7; i++) {
        lcd_set_pixel(mapping[i].com, mapping[i].seg, (font >> i) & 1);
    }
}


// ==========================================
// 外部调用接口
// ==========================================
void DRV_LCD_Init(void) {
    sys_safe_access_enable();
    
    // 1. 关闭对应引脚的数字输入功能，降低功耗并允许LCD复用
    uint32_t val = R32_PIN_CONFIG2;
    val |= (1<<4) | (1<<5) | (1<<12) | (1<<13) | (1<<14) | (1<<15); // PA4, PA5, PA12-PA15
    val |= (1<<16) | (1<<20) | (1<<22) | (1<<23);                   // PB0, PB4, PB6, PB7
    val |= (1<<25);                                                 // PB23 (SEG2)
    val |= (1<<26) | (1<<27);                                       // PB10, PB11
    val |= (1<<28) | (1<<29) | (1<<30) | (1<<31);                   // PB12-PB15 (COM0-COM3)
    R32_PIN_CONFIG2 = val;

    // 2. 关闭两线调试接口，释放 PB14/PB15 给 COM2/COM3
    R16_PIN_ALTERNATE |= (1 << 13); // RB_DEBUG_EN = 1
    
    sys_safe_access_disable();

    // 3. 清空显存
    R32_LCD_RAM0 = 0;
    R32_LCD_RAM1 = 0;
    R32_LCD_RAM2 = 0;

    // 4. 配置LCD_CMD寄存器
    // SEG开启: HW SEG 0,1,2, 4,5,6,7, 8,9, 14,15, 16,17
    // [27:24] SEG16-19 -> 0x03 (16,17)
    // [23:16] SEG8-15  -> 0xC3 (8,9,14,15)
    // [15:8]  SEG0-7   -> 0xF7 (0,1,2,4,5,6,7)
    // VLCD_SEL(7) = 0 (0=vdd 1=2.5v)
    // SCAN_CLK_SEL(6:5) = 10b (64Hz)
    // LCD_DUTY(4:3) = 10b (1/4 Duty)
    // LCD_BIAS(2) = 0 (0=1/2 Bias；1=1/3 Bias)
    // LCD_ON(1) = 1, SYS_EN(0) = 1
    R32_LCD_CMD = (0x03 << 24) | (0xC3 << 16) | (0xF7 << 8) | (0 << 7) | (2 << 5) | (2 << 3) | (0 << 2) | 3;
}


void DRV_LCD_Refresh(const LCD_DisplayData *data) {
    // 每次刷新前先清空全局显存，防止上次显示的残影残留
    R32_LCD_RAM0 = 0;
    R32_LCD_RAM1 = 0;
    R32_LCD_RAM2 = 0;

    // ----------------- 1. 刷新温度 -----------------
    float temp = data->temperature;
    if (temp > 199.9f) temp = 199.9f;
    if (temp < -199.9f) temp = -199.9f;

    // 负号 (强制要求显示或者真实温度为负时点亮)
    BOOL is_negative = (temp < 0.0f) || data->show_minus;
    lcd_set_pixel(2, 1, is_negative);
    
    if (temp < 0.0f) temp = -temp;

    int t = (int)(temp * 10.0f + 0.5f);
    int d1 = t / 1000;
    int d2 = (t / 100) % 10;
    int d3 = (t / 10) % 10;
    int d4 = t % 10;

    // 百位数字"1"，上下两段由 11 控制
    lcd_set_pixel(1, 1, d1 > 0);
    
    // 绘制数字，若百位和十位都为0，则十位消隐
    lcd_draw_digit(temp_d2, (d1 == 0 && d2 == 0) ? -1 : d2);
    lcd_draw_digit(temp_d3, d3);
    lcd_draw_digit(temp_d4, d4);
    
    // 摄氏度与小数点符号
    if (data->is_celsius) {
        lcd_set_pixel(1, 13, 1); // 摄氏度符号常亮 (假设原逻辑)
    } else {
        // 如果有华氏度符号，在这里配置其对应的 com 和 seg
    }
    lcd_set_pixel(0, 5, 1);      // 小数点常亮

    // ----------------- 2. 刷新湿度 -----------------
    float hum = data->humidity;
    if (hum > 99.0f) hum = 99.0f;
    if (hum < 0.0f) hum = 0.0f;

    int h = (int)(hum + 0.5f);
    int h_d1 = h / 10;
    int h_d2 = h % 10;

    // 十位若为0则消隐
    lcd_draw_digit(hum_d1, (h_d1 == 0) ? -1 : h_d1);
    lcd_draw_digit(hum_d2, h_d2);
    
    // % 符号常亮
    lcd_set_pixel(3, 10, 1);

    // 舒适度图标判断
    lcd_set_pixel(3, 6, h < 40);             // dry (干燥)
    lcd_set_pixel(0, 6, h >= 40 && h <= 60); // com (舒适)
    lcd_set_pixel(3, 8, h > 60);             // wet (潮湿)

    // ----------------- 3. 刷新电池图标 -----------------
    // 新代码中未包含电池图标映射，在此预留框架
    // 你需要查阅新屏幕的真值表，找到电池格数的坐标
    /*
    if (data->battery >= BAT_LOW)  lcd_set_pixel(?, ?, 1); // 电池外框/空电
    if (data->battery >= BAT_MED)  lcd_set_pixel(?, ?, 1); // 电池一半
    if (data->battery >= BAT_HIGH) lcd_set_pixel(?, ?, 1); // 电池满电
    */
}