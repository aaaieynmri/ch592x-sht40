#include "soft_i2c.h"
#include <CH59x_common.h>
#include <stdint.h>

// 全局计时变量（供外部读取）
uint16_t g_i2c_write_ticks = 0;
uint16_t g_i2c_read_ticks = 0;

/* ========== 引脚定义（PA6=SCL, PA7=SDA）========== */
#define SDA_PIN         7
#define SDA_BIT         (1 << (SDA_PIN % 8))
#define SDA_DIR_REG     R8_PA_DIR_0
#define SDA_DRV_REG     R8_PA_PD_DRV_0
#define SDA_CLR_REG     R8_PA_CLR_0
#define SDA_GET_REG     R8_PA_PIN_0

#define SCL_PIN         6
#define SCL_BIT         (1 << (SCL_PIN % 8))
#define SCL_DIR_REG     R8_PA_DIR_0
#define SCL_DRV_REG     R8_PA_PD_DRV_0
#define SCL_CLR_REG     R8_PA_CLR_0
#define SCL_GET_REG     R8_PA_PIN_0

/* ========== 宏定义：输出高 = 输入+上拉（开漏），输出低 = 推挽低 ========== */
#define SDA_HIGH()  do { SDA_DRV_REG &= ~SDA_BIT; SDA_DIR_REG &= ~SDA_BIT; } while(0)
#define SDA_LOW()   do { SDA_DIR_REG |=  SDA_BIT; SDA_DRV_REG |=  SDA_BIT; SDA_CLR_REG = SDA_BIT; } while(0)
#define SDA_READ()  (SDA_GET_REG & SDA_BIT)

#define SCL_HIGH()  do { SCL_DRV_REG &= ~SCL_BIT; SCL_DIR_REG &= ~SCL_BIT; } while(0)
#define SCL_LOW()   do { SCL_DIR_REG |=  SCL_BIT; SCL_DRV_REG |=  SCL_BIT; SCL_CLR_REG = SCL_BIT; } while(0)
#define SCL_READ()  (SCL_GET_REG & SCL_BIT)

/* ========== 微调延时：16MHz 下每条 __nop() 约 62.5ns，用若干条调整 ========== */
#define DELAY_HI()   /* 无延时 */
#define DELAY_LO()   /* 无延时 */

/* ========== 基础操作（全部放在 RAM 中加速）========== */
__attribute__((section(".ramfunc")))
static void i2c_start(void) {
    SDA_HIGH(); DELAY_HI();
    SCL_HIGH(); DELAY_HI();
    // 不检测时钟拉伸（SHT40 不会在传输中拉伸）
    SDA_LOW();  DELAY_LO();
    SCL_LOW();  DELAY_LO();
}

__attribute__((section(".ramfunc")))
static void i2c_stop(void) {
    SDA_LOW();  DELAY_LO();
    SCL_HIGH(); DELAY_HI();
    SDA_HIGH(); DELAY_HI();
    DELAY_HI(); DELAY_HI();   // 额外的停止保持时间
}

__attribute__((section(".ramfunc")))
static uint8_t i2c_write_byte(uint8_t dat) {
    uint8_t ack;
    for (int8_t x = 8; x; x--) {
        if (dat & 0x80) SDA_HIGH();
        else            SDA_LOW();
        SCL_HIGH(); DELAY_HI();
        dat <<= 1;
        SCL_LOW();  DELAY_LO();
    }
    // 释放 SDA 接收 ACK
    SDA_HIGH(); 
    SCL_HIGH(); DELAY_HI();
    ack = SDA_READ();          // 0 = ACK
    SCL_LOW();  DELAY_LO();
    return ack;
}

__attribute__((section(".ramfunc")))
static uint8_t i2c_read_byte(uint8_t send_ack) {
    uint8_t in = 0;
    SDA_HIGH();                // 释放总线，从机驱动
    for (int8_t x = 8; x; x--) {
        in <<= 1;
        SCL_HIGH(); DELAY_HI();
        if (SDA_READ()) in |= 1;
        SCL_LOW();  DELAY_LO();
    }
    if (send_ack) SDA_LOW();   // ACK
    else          SDA_HIGH();  // NACK
    SCL_HIGH(); DELAY_HI();
    SCL_LOW();  DELAY_LO();
    return in;
}

/* ========== 对外接口 ========== */
void soft_i2c_init(void) {
    // 使能内部上拉
    R8_PA_PU_0 |= (SDA_BIT | SCL_BIT);
    // 初始状态：释放总线
    SDA_HIGH();
    SCL_HIGH();
}

int soft_i2c_write(uint8_t dev_addr, const uint8_t *data, uint8_t len) {
    uint32_t start = R32_RTC_CNT_32K;
    i2c_start();
    if (i2c_write_byte((dev_addr << 1) | 0)) {  // 写方向
        i2c_stop();
        return -1;
    }
    for (uint8_t i = 0; i < len; i++) {
        if (i2c_write_byte(data[i])) {
            i2c_stop();
            return -1;
        }
    }
    i2c_stop();
    g_i2c_write_ticks = (uint16_t)(R32_RTC_CNT_32K - start);
    return 0;
}

int soft_i2c_read(uint8_t dev_addr, uint8_t *data, uint8_t len) {
    uint32_t start = R32_RTC_CNT_32K;
    i2c_start();
    if (i2c_write_byte((dev_addr << 1) | 1)) {  // 读方向
        i2c_stop();
        return -1;
    }
    for (uint8_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(i < len - 1);   // 最后一个字节 NACK
    }
    i2c_stop();
    g_i2c_read_ticks = (uint16_t)(R32_RTC_CNT_32K - start);
    return len;
}