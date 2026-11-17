#include "status_task.h"
#include "bsp_can.h"
#include "my_main.h"
#include "arm_task.h"

volatile status_snapshot_t g_status_snapshot;

volatile uint8_t gpio_pb6 = 0U;
volatile uint8_t gpio_pb7 = 0U;
volatile uint8_t gpio_pd7 = 0U;
volatile uint8_t gpio_pd6 = 0U;
volatile uint8_t gpio_pd1 = 0U;
volatile uint8_t gpio_pd3 = 0U;
volatile uint8_t gpio_pd4 = 0U;
volatile uint8_t gpio_pd5 = 0U;

void Status_InitGpio(void)
{
    gpio_pb6 = 0U;
    gpio_pb7 = 0U;
    gpio_pd7 = 0U;
    gpio_pd6 = 0U;
    gpio_pd1 = 0U;
    gpio_pd3 = 0U;
    gpio_pd4 = 0U;
    gpio_pd5 = 0U;
}

void Status_ApplyGpio(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, gpio_pb6 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, gpio_pb7 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, gpio_pd7 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, gpio_pd6 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, gpio_pd1 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, gpio_pd3 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, gpio_pd4 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, gpio_pd5 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool Status_GetSnapshot(status_snapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
        return false;

    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = g_status_snapshot;
    __set_PRIMASK(primask);
    return true;
}

void Status_UpdateSnapshot(void)
{
    static chassis_speed_t planned_speed = {0.0f, 0.0f, 0.0f};
    status_snapshot_t next = {0};

    (void)Remote_GetState(&next.remote);
    {
        static uint8_t switch_raw_prev;
        static uint8_t switch_stable;
        static uint8_t switch_count;
        static bool switch_initialized;
        uint8_t switch_raw = next.remote.switch_state;

        if (!switch_initialized)
        {
            switch_raw_prev = switch_raw;
            switch_stable = switch_raw;
            switch_initialized = true;
        }
        else if (switch_raw != switch_raw_prev)
        {
            switch_raw_prev = switch_raw;
            switch_count = 0U;
        }
        else if (switch_count < 3U)
        {
            switch_count++;
            if (switch_count >= 3U)
                switch_stable = switch_raw;
        }
        next.remote.switch_state = switch_stable;
    }

    joy_to_mps(&next.remote, &planned_speed);
    next.target_speed = planned_speed;
    next.control = (next.remote.online != 0U) ? STATUS_CONTROL_REMOTE : STATUS_CONTROL_OFFLINE;

    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        next.sequence = g_status_snapshot.sequence + 1U;
        g_status_snapshot = next;
        __set_PRIMASK(primask);
    }
}

void Status_Task(void *argument)
{
    uint32_t tick;
    static bool can1_bus_off_prev;
    static bool can2_bus_off_prev;
    static bool can3_bus_off_prev;

    (void)argument;
    Status_InitGpio();
    tick = osKernelGetTickCount();

    for (;;)
    {
        bool can1_bus_off = ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U);
        bool can2_bus_off = ((hfdcan2.Instance->PSR & FDCAN_PSR_BO) != 0U);
        bool can3_bus_off = ((hfdcan3.Instance->PSR & FDCAN_PSR_BO) != 0U);

        if (can1_bus_off && !can1_bus_off_prev) can1_diag.bus_off_count++;
        if (can2_bus_off && !can2_bus_off_prev) can2_diag.bus_off_count++;
        if (can3_bus_off && !can3_bus_off_prev) can3_diag.bus_off_count++;
        can1_bus_off_prev = can1_bus_off;
        can2_bus_off_prev = can2_bus_off;
        can3_bus_off_prev = can3_bus_off;

        BspCan_Service();
        Status_UpdateSnapshot();
        Status_ApplyGpio();
        tick += 2U;
        osDelayUntil(tick);
    }
}
