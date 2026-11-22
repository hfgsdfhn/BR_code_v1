#include "dt35.h"
#include <string.h>
#include "main.h"

#define DT35_FRAME_SIZE 11U
#define DT35_FRAME_HEAD_0 0xAAU
#define DT35_FRAME_HEAD_1 0x55U

typedef char dt35_float_must_be_32_bits[(sizeof(float) == 4U) ? 1 : -1];

static uint8_t rx_frame[DT35_FRAME_SIZE];
static uint8_t rx_length;
static dt35_t latest_data;
static uint8_t latest_valid;

static uint8_t DT35_Checksum(const uint8_t *frame)
{
    uint16_t sum = 0U;
    for (uint32_t index = 0U; index < DT35_FRAME_SIZE - 1U; index++)
        sum += frame[index];
    return (uint8_t)sum;
}

static float DT35_ReadFloatLE(const uint8_t *data)
{
    uint32_t bits = (uint32_t)data[0] |
                    ((uint32_t)data[1] << 8U) |
                    ((uint32_t)data[2] << 16U) |
                    ((uint32_t)data[3] << 24U);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void DT35_Resync(void)
{
    for (uint8_t index = 1U; index + 1U < DT35_FRAME_SIZE; index++)
    {
        if ((rx_frame[index] == DT35_FRAME_HEAD_0) &&
            (rx_frame[index + 1U] == DT35_FRAME_HEAD_1))
        {
            rx_length = (uint8_t)(DT35_FRAME_SIZE - index);
            memmove(rx_frame, &rx_frame[index], rx_length);
            return;
        }
    }

    if (rx_frame[DT35_FRAME_SIZE - 1U] == DT35_FRAME_HEAD_0)
    {
        rx_frame[0] = DT35_FRAME_HEAD_0;
        rx_length = 1U;
    }
    else
    {
        rx_length = 0U;
    }
}

void DT35_RxBytes(const uint8_t *data, uint16_t size)
{
    if ((data == NULL) || (size == 0U))
        return;

    for (uint16_t index = 0U; index < size; index++)
    {
        uint8_t byte = data[index];

        if (rx_length == 0U)
        {
            if (byte == DT35_FRAME_HEAD_0)
                rx_frame[rx_length++] = byte;
            continue;
        }

        if ((rx_length == 1U) && (byte != DT35_FRAME_HEAD_1))
        {
            if (byte == DT35_FRAME_HEAD_0)
                rx_frame[0] = byte;
            else
                rx_length = 0U;
            continue;
        }

        rx_frame[rx_length++] = byte;
        if (rx_length == DT35_FRAME_SIZE)
        {
            if (rx_frame[DT35_FRAME_SIZE - 1U] == DT35_Checksum(rx_frame))
            {
                dt35_t parsed = {
                    .dt35_x = DT35_ReadFloatLE(&rx_frame[2]),
                    .dt35_y = DT35_ReadFloatLE(&rx_frame[6])
                };
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                latest_data = parsed;
                latest_valid = 1U;
                __set_PRIMASK(primask);
                rx_length = 0U;
            }
            else
            {
                DT35_Resync();
            }
        }
    }
}

uint8_t DT35_GetLatest(dt35_t *data)
{
    uint32_t primask;
    uint8_t valid;

    if (data == NULL)
        return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    valid = latest_valid;
    if (valid != 0U)
        *data = latest_data;
    __set_PRIMASK(primask);
    return valid;
}