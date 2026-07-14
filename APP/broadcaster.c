/********************************** COPYRIGHT *******************************
 * broadcaster.c - BTHome v2 温湿度计（周期性突发广播模式）
 * 描述：
 *   每 WAKEUP_PERIOD_MS 唤醒一次，采集温湿度及电池，然后在极短时间内
 *   连续发送 BURST_COUNT 个 BTHome 广播包，随后关闭广播并进入低功耗等待。
 *   关键参数均以宏定义，方便调节。
 *******************************************************************************/

#include "CONFIG.h"
#include "devinfoservice.h"
#include "broadcaster.h"
#include "drv_sht40.h"
#include "drv_lcd.h"
#include "drv_adc.h"
#include <math.h>  // 必须包含，用于 fabs() 浮点数绝对值计算


/* RF 32MHz晶振校准策略
 *
 * 外部LSE作为RTC，不需要LSI校准
 * RF校准：
 * 1. 温度变化超过15.5℃
 * 2. 最大4周一次
 */

#define RF_TEMP_THRESHOLD      15.5f
// 20秒一次唤醒
// 4周 = 120960次
#define RF_CALIB_PERIOD        120960UL

static float last_rf_calib_temp = 0.0f;
static uint8_t rf_calib_initialized = FALSE;

static uint32_t rf_calib_counter = 0;

/* --- 门限设置 --- */
#define REPORT_TEMP_DIFF 0.5f     // 温度变化 0.5 度上报
#define REPORT_HUMI_DIFF 1.0f     // 湿度变化 1% 上报
#define FORCE_REPORT_INTERVAL 60  // 保底上报：每 60 次采集(20分钟)强制上报一次
#define ADC_SAMPLE_INTERVAL     4320    // 约24小时测一次 (20s * 4320)adc

/* --- 状态记录 --- */
static float last_report_temp = -99.0f;  // 上次上报的温度
static float last_report_humi = -99.0f;  // 上次上报的湿度
static uint16_t force_report_cnt = 0;    // 保底计数器
static uint8_t packet_id = 0;            // BTHome 数据包序列号
static uint16_t cached_vbat_mv = 3000;  // 缓存电量，初始假定 3000mV
static uint16_t adc_run_count = 0;      // ADC 计数器
// 定义时间常量 (注意 TMOS 的单位是 0.625ms!)
#define ADV_INTERVAL 244     // 间隔
#define BURST_DURATION (244*10)  // 一组
#define WAKEUP_PERIOD 32000  // 20秒醒来一次 (16000 * 0.625 = 10000ms)

// 事件位重定义
#define SBP_START_DEVICE_EVT 0x0001
#define SBP_WAKEUP_EVT 0x0002
#define SBP_DATA_READY_EVT 0x0004
#define SBP_STOP_ADV_EVT 0x0008  // 替换掉原来的 NEXT_BURST_EVT
/* BTHome v2 广告数据模板 (数据部分后续动态更新) */
static uint8_t advertData[] = {
    0x02, GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x0F, 0x16,        // Service Data (16-bit UUID), 长度 15 (新增电压)
    0xD2, 0xFC,        // UUID = 0xFCD2
    0x44,              // BTHome device info: v2, 未加密, 定期
    0x01, 0x00,        // 电池电量 [9] (占位)
    0x02, 0x00, 0x00,  // 温度 [11,12] (占位)
    0x03, 0x00, 0x00,  // 湿度 [14,15] (占位)
    0x0C, 0x00, 0x00   // 电池电压 [16,17,18] (占位: ID + 2字节值)
};

static uint8_t Broadcaster_TaskID;
static uint8_t burst_count;  // 已发送的突发包计数

/* 函数声明 */
static void Broadcaster_ProcessTMOSMsg (tmos_event_hdr_t *pMsg);
static void Broadcaster_StateNotificationCB (gapRole_States_t newState);

/* GAP 角色回调 */
static gapRolesBroadcasterCBs_t Broadcaster_CBs = {
    Broadcaster_StateNotificationCB,  // 状态改变回调
    NULL                              // 第二个回调（通常不用）
};

__HIGH_CODE
void BTHome_UpdateData (float temp, float humi, uint16_t vbat_mv) {
    // 1. 电池百分比计算
    uint8_t bat_pct;

    // 根据 CR2032 放电曲线进行分段计算
    if (vbat_mv >= 3000) {
        bat_pct = 100; // 3.0V以上视为满电
    } 
    else if (vbat_mv >= 2900) {
        // 2.9V - 3.0V 对应 80% - 100% (这个区间电量最耐用)
        bat_pct = 80 + (vbat_mv - 2900) / 5; 
    } 
    else if (vbat_mv >= 2800) {
        // 2.8V - 2.9V 对应 50% - 80%
        bat_pct = 50 + (vbat_mv - 2800) * 3 / 10; 
    } 
    else if (vbat_mv >= 2700) {
        // 2.7V - 2.8V 对应 20% - 50%
        bat_pct = 20 + (vbat_mv - 2700) * 3 / 10;
    } 
    else if (vbat_mv >= 2500) {
        // 2.5V - 2.7V 对应 0% - 20% (进入红色警戒区)
        bat_pct = (vbat_mv - 2500) / 10;
    } 
    else {
        bat_pct = 0;
    }

    advertData[9] = bat_pct;
    // 新增：填充电池电压（ID 0x0C 已在数组中固定）
    advertData[17] = (uint8_t)(vbat_mv & 0xFF);
    advertData[18] = (uint8_t)((vbat_mv >> 8) & 0xFF);

    // 2. 温度转换 (sint16, 小端序)
    int16_t temp_100 = (int16_t)(temp * 100);
    advertData[11] = (uint8_t)(temp_100 & 0xFF);
    advertData[12] = (uint8_t)((temp_100 >> 8) & 0xFF);

    // 3. 湿度转换 (uint16, 小端序)
    uint16_t humi_100 = (uint16_t)(humi * 100);
    advertData[14] = (uint8_t)(humi_100 & 0xFF);
    advertData[15] = (uint8_t)((humi_100 >> 8) & 0xFF);

    // 4. 更新协议栈广播内容
    GAPRole_SetParameter (GAPROLE_ADVERT_DATA, sizeof (advertData), advertData);
}

/*********************************************************************
 * 广播初始化：配置参数但不开启，启动角色
 *********************************************************************/
void Broadcaster_Init (void) {
    Broadcaster_TaskID = TMOS_ProcessEventRegister (Broadcaster_ProcessEvent);

    // 广播初始状态：不开启
    uint8_t adv_enable = FALSE;
    uint8_t adv_type = GAP_ADTYPE_ADV_NONCONN_IND;
    GAPRole_SetParameter (GAPROLE_ADVERT_ENABLED, sizeof (uint8_t), &adv_enable);
    GAPRole_SetParameter (GAPROLE_ADV_EVENT_TYPE, sizeof (uint8_t), &adv_type);
    GAPRole_SetParameter (GAPROLE_ADVERT_DATA, sizeof (advertData), advertData);

    uint16_t adv_int = ADV_INTERVAL;
    GAP_SetParamValue (TGAP_DISC_ADV_INT_MIN, adv_int);
    GAP_SetParamValue (TGAP_DISC_ADV_INT_MAX, adv_int);

    // 延时启动广播角色
    tmos_start_task (Broadcaster_TaskID, SBP_START_DEVICE_EVT, 160);
}

/*********************************************************************
 * 事件处理：控制整个采集 -> 突发 -> 休眠流程
 *********************************************************************/
uint16_t Broadcaster_ProcessEvent (uint8_t task_id, uint16_t events) {
    // 系统消息处理
    if (events & SYS_EVENT_MSG) {
        uint8_t *pMsg;
        if ((pMsg = tmos_msg_receive (Broadcaster_TaskID)) != NULL) {
            Broadcaster_ProcessTMOSMsg ((tmos_event_hdr_t *)pMsg);
            tmos_msg_deallocate (pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }

    // 1. 设备启动
    if (events & SBP_START_DEVICE_EVT) {
        GAPRole_BroadcasterStartDevice (&Broadcaster_CBs);
        tmos_set_event (Broadcaster_TaskID, SBP_WAKEUP_EVT);
        return (events ^ SBP_START_DEVICE_EVT);
    }

    // --- 步骤 1: 10秒周期到达，开始测量 ---
    if (events & SBP_WAKEUP_EVT) {
        DRV_SHT40_StartMeasure();
        // tmos_start_task (Broadcaster_TaskID, SBP_DATA_READY_EVT, 3);  //  3个单位 = 1.875ms,
        tmos_start_task (Broadcaster_TaskID, SBP_DATA_READY_EVT, 14);  //  14个单位 = 8.75ms,
        return (events ^ SBP_WAKEUP_EVT);
    }

    // --- 步骤 2: 拿到数据，逻辑判定 ---
    if (events & SBP_DATA_READY_EVT) {
        SHT40_Result res = DRV_SHT40_GetResult();
        
        // 2. 【核心修改】判定是否需要采集 ADC
        // 初始运行或达到计数值时采样
        if (adc_run_count == 0) {
            cached_vbat_mv = DRV_ADC_GetVbat();
            // PRINT("ADC Sampled: %d mV\n", cached_vbat_mv);
        }
        
        // 计数器累加
        adc_run_count++;
        if (adc_run_count >= ADC_SAMPLE_INTERVAL) {
            adc_run_count = 0; // 下次唤醒时采样
        }

        // uint16_t vbat = DRV_ADC_GetVbat();

        force_report_cnt++;

        if (res.valid) {

            /* ================= RF 32MHz 校准 ================= */
            if(rf_calib_initialized == FALSE)
            {
                last_rf_calib_temp = res.temp;
                rf_calib_initialized = TRUE;
            }
            else
            {
                rf_calib_counter++;

                BOOL need_rf_calib = FALSE;


                // 条件1：温度变化超过15℃
                if(fabs(res.temp - last_rf_calib_temp) >= RF_TEMP_THRESHOLD)
                {
                    need_rf_calib = TRUE;
                }


                // 条件2：最长4周
                if(rf_calib_counter >= RF_CALIB_PERIOD)
                {
                    need_rf_calib = TRUE;
                }


                if(need_rf_calib)
                {
                    BLE_RegInit();

                    last_rf_calib_temp = res.temp;
                    rf_calib_counter = 0;
                }
            }

            LCD_DisplayData disp;
            // 1. 显式清零整个结构体，确保所有未提到的成员都是 0 (FALSE)
            memset (&disp, 0, sizeof (LCD_DisplayData));

            // 2. 填充数据
            disp.temperature = res.temp;
            disp.humidity = res.humi;
            disp.is_celsius = TRUE;

            // 3. 只有真正需要时才手动控负号（比如你想做低温报警闪烁之类的）
            // 其实 lcd.c 内部已经判断了 temp < 0，所以这里保持 0 即可
            disp.show_minus = FALSE;

            if (cached_vbat_mv  > 2900)
                disp.battery = BAT_HIGH;
            else if (cached_vbat_mv  > 2600)
                disp.battery = BAT_MED;
            else
                disp.battery = BAT_LOW;

            DRV_LCD_Refresh (&disp);

            // --- 核心判定逻辑 ---
            BOOL should_report = FALSE;

            // 条件 A: 温度变化门限
            if (fabs (res.temp - last_report_temp) >= REPORT_TEMP_DIFF)
                should_report = TRUE;
            // 条件 B: 湿度变化门限
            if (fabs (res.humi - last_report_humi) >= REPORT_HUMI_DIFF)
                should_report = TRUE;
            // 条件 C: 保底时间到了
            if (force_report_cnt >= FORCE_REPORT_INTERVAL)
                should_report = TRUE;

            if (should_report) {
                // 更新记录值
                last_report_temp = res.temp;
                last_report_humi = res.humi;
                force_report_cnt = 0;

                // 拼包并开启连喷
                BTHome_UpdateData (res.temp, res.humi, cached_vbat_mv);
                uint8_t adv_en = TRUE;
                GAPRole_SetParameter (GAPROLE_ADVERT_ENABLED, sizeof (uint8_t), &adv_en);

                // 开启连喷持续时间（2.9秒）
                tmos_start_task (Broadcaster_TaskID, SBP_STOP_ADV_EVT, BURST_DURATION);

                // 打印一下提示（调试用）
                // PRINT("Report Triggered: T:%.2f H:%.2f\n", res.temp, res.humi);
            } else {
                // 不需要上报，直接进入下一次 X 秒大循环
                tmos_start_task (Broadcaster_TaskID, SBP_WAKEUP_EVT, WAKEUP_PERIOD);
            }
        } else {
            // 采集失败处理：1秒后重试
            tmos_start_task (Broadcaster_TaskID, SBP_WAKEUP_EVT, 1600);
        }
        return (events ^ SBP_DATA_READY_EVT);
    }

    // --- 步骤 3: 连喷结束，停止广播并深睡 ---
    if (events & SBP_STOP_ADV_EVT) {
        uint8_t adv_en = FALSE;
        GAPRole_SetParameter (GAPROLE_ADVERT_ENABLED, sizeof (uint8_t), &adv_en);

        // 连喷完了，定下一个 10 秒后的唤醒
        tmos_start_task (Broadcaster_TaskID, SBP_WAKEUP_EVT, WAKEUP_PERIOD);
        return (events ^ SBP_STOP_ADV_EVT);
    }

    return 0;
}

/*********************************************************************
 * 协议栈消息处理（未使用）
 *********************************************************************/
static void Broadcaster_ProcessTMOSMsg (tmos_event_hdr_t *pMsg) {
    (void)pMsg;
}

/*********************************************************************
 * 广播状态回调（仅用于调试打印）
 *********************************************************************/
static void Broadcaster_StateNotificationCB (gapRole_States_t newState) {
    switch (newState) {
    case GAPROLE_STARTED: PRINT ("..Started\n"); break;
    case GAPROLE_ADVERTISING: PRINT ("..Advertising\n"); break;
    case GAPROLE_WAITING: PRINT ("..Waiting\n"); break;
    case GAPROLE_ERROR: PRINT ("..Error\n"); break;
    default: break;
    }
}