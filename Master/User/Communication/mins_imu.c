#include "mins_imu.h"

#include <string.h>

#define ERROR_K                         0.973f  //转动误差系数

#define MINS_IMU_YAW_OFFSET_DEG         2.84f   //上电固定飘移角

#define MINS_IMU_FRAME_HEAD             0x77U   //帧头
#define MINS_IMU_ADDR_CODE              0x00U   //地址码（不知道地址码就在上位机串口助手看一眼，每一帧的第三位是地址码）
#define MINS_IMU_FRAME_EULER_LENGTH     0x0DU   //回复欧拉角数据帧长度
#define MINS_IMU_FRAME_EULER_SIZE       (MINS_IMU_FRAME_EULER_LENGTH + 1U)
#define MINS_IMU_RESPONSE_EULER         0x84U   //读取三轴欧拉角 接收
#define MINS_IMU_COMMAND_EULER          0x04U   //读取三轴欧拉角 发送
#define MINS_IMU_COMMAND_OUTPUT_TYPE    0x56U   //设置自动输出数据模式 发送/接收
#define MINS_IMU_COMMAND_OUTPUT_RATE    0x0CU   //设置自动输出频率 发送

mins_imu_t g_imu;

static uint8_t Mins_Imu_RxFrame[MINS_IMU_FRAME_EULER_SIZE];
static uint16_t Mins_Imu_RxFrameLength;

/**
 * @brief 帧校验函数
 * 
 * @param frame         接收到的数据帧
 * @param len           数据帧长度（几个字节）
 * @return uint8_t      如果数据合规，返回校验和
 */
static uint8_t Mins_Imu_CheckFrame(const uint8_t *frame, uint16_t len)
{
    uint16_t index;
    uint8_t checksum;

    if ((len < 5U) || (frame[0] != MINS_IMU_FRAME_HEAD)         //数据帧长度不可能小于5，帧头必须是0x77
        || (len != (uint16_t)(frame[1] + 1U)))
    {
        return 0U;
    }

    checksum = 0U;
    for (index = 1U; index < (len - 1U); index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return checksum == frame[len - 1U];
}

/**
 * @brief 数据帧里的欧拉角数据转化成浮点数
 * 
 * @param bcd 
 * @return float 
 */
static float Mins_Imu_DecodeAngle(const uint8_t *bcd)
{
    float angle;

    angle = (float)((bcd[0] & 0x0FU) * 100U);
    angle += (float)(((bcd[1] >> 4U) & 0x0FU) * 10U);
    angle += (float)(bcd[1] & 0x0FU);
    angle += (float)((bcd[2] >> 4U) & 0x0FU) * 0.1f;
    angle += (float)(bcd[2] & 0x0FU) * 0.01f;
    return ((bcd[0] & 0x80U) != 0U) ? -angle : angle;
}

/**
 * @brief YAW角0°时容易跳变解决
 * 
 * @param yaw_delta 
 * @return float 
 */
static float Mins_Imu_WrapYawDelta(float yaw_delta)
{
    while (yaw_delta >= 180.0f)
    {
        yaw_delta -= 360.0f;
    }
    while (yaw_delta < -180.0f)
    {
        yaw_delta += 360.0f;
    }
    return yaw_delta;
}

static void Mins_Imu_ProcessEulerFrame(const uint8_t *frame)
{
    float raw_yaw;

    g_imu.euler.pitch = Mins_Imu_DecodeAngle(&frame[4U]);
    g_imu.euler.roll = Mins_Imu_DecodeAngle(&frame[7U]);
    raw_yaw = Mins_Imu_DecodeAngle(&frame[10U]);

    if (g_imu.yaw_last_valid == 0U)
    {
        g_imu.yaw_last_raw = raw_yaw;
        g_imu.yaw_total = 0.0f;
        g_imu.yaw_last_valid = 1U;
    }
    else
    {
        g_imu.yaw_total -= Mins_Imu_WrapYawDelta(raw_yaw - g_imu.yaw_last_raw);
        g_imu.yaw_last_raw = raw_yaw;
    }
    g_imu.euler.yaw = g_imu.yaw_total * ERROR_K;
    g_imu.update_count++;
}

static void Mins_Imu_ResyncRxFrame(void)
{
    uint16_t start;

    for (start = 1U; start < Mins_Imu_RxFrameLength; start++)
    {
        if ((Mins_Imu_RxFrame[start] == MINS_IMU_FRAME_HEAD)
            && ((start + 1U == Mins_Imu_RxFrameLength)
                || (Mins_Imu_RxFrame[start + 1U] == MINS_IMU_FRAME_EULER_LENGTH)))
        {
            Mins_Imu_RxFrameLength = (uint16_t)(Mins_Imu_RxFrameLength - start);
            memmove(Mins_Imu_RxFrame, &Mins_Imu_RxFrame[start], Mins_Imu_RxFrameLength);
            return;
        }
    }
    Mins_Imu_RxFrameLength = 0U;
}

/**
 * @brief 回包数据解析
 * 
 * @param data 
 * @param len 
 */
void Mins_Imu_OnRx(const uint8_t *data, uint16_t len)
{
    uint16_t index;

    if (data == NULL)
    {
        return;
    }

    for (index = 0U; index < len; index++)
    {
        uint8_t byte = data[index];

        if (Mins_Imu_RxFrameLength == 0U)
        {
            if (byte == MINS_IMU_FRAME_HEAD)
            {
                Mins_Imu_RxFrame[0] = byte;
                Mins_Imu_RxFrameLength = 1U;
            }
            continue;
        }

        if (Mins_Imu_RxFrameLength == 1U)
        {
            if (byte == MINS_IMU_FRAME_EULER_LENGTH)
            {
                Mins_Imu_RxFrame[Mins_Imu_RxFrameLength++] = byte;
            }
            else if (byte == MINS_IMU_FRAME_HEAD)
            {
                Mins_Imu_RxFrame[0] = byte;
            }
            else
            {
                Mins_Imu_RxFrameLength = 0U;
            }
            continue;
        }

        Mins_Imu_RxFrame[Mins_Imu_RxFrameLength++] = byte;
        if (Mins_Imu_RxFrameLength == MINS_IMU_FRAME_EULER_SIZE)
        {
            if ((Mins_Imu_RxFrame[3U] == MINS_IMU_RESPONSE_EULER)
                && Mins_Imu_CheckFrame(Mins_Imu_RxFrame, MINS_IMU_FRAME_EULER_SIZE))
            {
                Mins_Imu_ProcessEulerFrame(Mins_Imu_RxFrame);
                Mins_Imu_RxFrameLength = 0U;
            }
            else
            {
                Mins_Imu_ResyncRxFrame();
            }
        }
    }
}
