#include "dt35.h"
#include <string.h>
#include <stddef.h>

/* DT35 全局数据 */
dt35_data_t dt35_data = {0};

/* DT35 协议配置 */
#define DT35_FRAME_HEAD_0     0xAAU
#define DT35_FRAME_HEAD_1     0x55U
#define DT35_FRAME_SIZE       11U

static uint8_t rx_frame[DT35_FRAME_SIZE];
static uint8_t rx_count = 0U;

/**
 * @brief DT35 数据解析
 */
void DT35_RxBytes(const uint8_t *data, uint16_t size)
{
    if ((data == NULL) || (size == 0U))
    {
        return;
    }

    for (uint16_t i = 0U; i < size; i++)
    {
        uint8_t byte = data[i];

        /* 等待第一个帧头 */
        if (rx_count == 0U)
        {
            if (byte == DT35_FRAME_HEAD_0)
            {
                rx_frame[rx_count++] = byte;
            }
            continue;
        }

        /* 等待第二个帧头 */
        if (rx_count == 1U)
        {
            if (byte == DT35_FRAME_HEAD_1)
            {
                rx_frame[rx_count++] = byte;
            }
            else if (byte == DT35_FRAME_HEAD_0)
            {
                rx_frame[0] = byte;
            }
            else
            {
                rx_count = 0U;
            }
            continue;
        }

        rx_frame[rx_count++] = byte;

        /* 接收完整帧 */
        if (rx_count == DT35_FRAME_SIZE)
        {
            rx_count = 0U;

            /* 校验数据：累加前 10 个字节，取低 8 位 */
            uint32_t sum = 0U;

            for (uint8_t j = 0U; j < (DT35_FRAME_SIZE - 1U); j++)
            {
                sum += rx_frame[j];
            }

            if ((uint8_t)(sum & 0xFFU) != rx_frame[DT35_FRAME_SIZE - 1U])
            {
                continue;
            }

            /* 解析浮点数 */
            memcpy(&dt35_data.front_distance, &rx_frame[2], sizeof(float));
            memcpy(&dt35_data.left_distance, &rx_frame[6], sizeof(float));
        }
    }
}