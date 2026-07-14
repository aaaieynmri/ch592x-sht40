#include "drv_sht40.h"
#include "soft_i2c.h" // 引入新的 I2C 头文件

#define SHT40_ADDR    0x44

void DRV_SHT40_Init(void) {
    soft_i2c_init();
}

static uint8_t Checksum(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc <<= 1;
        }
    }
    return crc;
}

// 第一阶段：发送测量命令并立即返回，不阻塞
void DRV_SHT40_StartMeasure(void) {
    uint8_t cmd = 0xFD; // 最高精度模式
    soft_i2c_write(SHT40_ADDR, &cmd, 1);
}

// 第二阶段：由 TMOS 在约 10ms 后调用读取结果
SHT40_Result DRV_SHT40_GetResult(void) {
    SHT40_Result res = {0, 0, FALSE};
    uint8_t buf[6] = {0};

    // 使用新的批量读取接口
    if (soft_i2c_read(SHT40_ADDR, buf, 6) == 6) {
        // 校验与换算
        if (Checksum(&buf[0], 2) == buf[2] && Checksum(&buf[3], 2) == buf[5]) {
            uint16_t t_ticks = (buf[0] << 8) | buf[1];
            uint16_t h_ticks = (buf[3] << 8) | buf[4];
            res.temp = -45.0f + 175.0f * (float)t_ticks / 65535.0f;
            res.humi = -6.0f + 125.0f * (float)h_ticks / 65535.0f;
            res.valid = TRUE;
        }
    }

    return res;
}