
#include "ethernet.h"
#include "spi.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdint.h>

/* W5500 硬件配置 */
#define W5500_CS_LOW()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define W5500_CS_HIGH()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

#define W5500_MAC0           0x02U
#define W5500_MAC1           0x08U
#define W5500_MAC2           0xDCU
#define W5500_MAC3           0x12U
#define W5500_MAC4           0x34U
#define W5500_MAC5           0x56U

#define W5500_UDP_SOCKET     0U
#define W5500_UDP_PORT       5000U

static uint8_t s_udp_receive_buffer[2048];
static osThreadId_t s_udp_task_handle = NULL;

static void W5500_UdpTask(void *argument);

/* SPI 片选控制 */
static void SPI_CS_Select(void)
{
    W5500_CS_LOW();
}

static void SPI_CS_Deselect(void)
{
    W5500_CS_HIGH();
}

/* W5500 临界区控制 */
static uint32_t s_wizchip_primask;
static uint32_t s_wizchip_critical_depth;

static void SPI_CrisEnter(void)
{
    if (s_wizchip_critical_depth == 0U)
    {
        s_wizchip_primask = __get_PRIMASK();
    }

    __disable_irq();
    s_wizchip_critical_depth++;
}

static void SPI_CrisExit(void)
{
    if (s_wizchip_critical_depth == 0U)
    {
        return;
    }

    s_wizchip_critical_depth--;

    if (s_wizchip_critical_depth == 0U)
    {
        if (s_wizchip_primask == 0U)
        {
            __enable_irq();
        }
    }
}

/* SPI 单字节读取 */
static uint8_t SPI_ReadByte(void)
{
    uint8_t rx_data = 0U;
    uint8_t tx_data = 0xFFU;

    if (HAL_SPI_TransmitReceive(&hspi6, &tx_data, &rx_data, 1U, 100U) != HAL_OK)
    {
        return 0U;
    }

    return rx_data;
}

/* SPI 单字节写入 */
static void SPI_WriteByte(uint8_t wb)
{
    uint8_t rx_data = 0U;

    (void)HAL_SPI_TransmitReceive(&hspi6, &wb, &rx_data, 1U, 100U);
}

/* SPI 批量读取 */
static void SPI_BusRead(uint8_t *pBuf, uint16_t len)
{
    uint8_t tx_data = 0xFFU;

    for (uint16_t i = 0U; i < len; i++)
    {
        if (HAL_SPI_TransmitReceive(&hspi6, &tx_data, &pBuf[i], 1U, 100U) != HAL_OK)
        {
            pBuf[i] = 0U;
        }
    }
}

/* SPI 批量写入 */
static void SPI_BusWrite(uint8_t *pBuf, uint16_t len)
{
    uint8_t rx_data[32];

    while (len > 0U)
    {
        uint16_t chunk = (len > sizeof(rx_data)) ? sizeof(rx_data) : len;

        if (HAL_SPI_TransmitReceive(&hspi6, pBuf, rx_data, chunk, 100U) != HAL_OK)
        {
            return;
        }

        pBuf += chunk;
        len = (uint16_t)(len - chunk);
    }
}

/* W5500 初始化 */
void W5500_Port_Init(void)
{
    wiz_NetInfo net_info = {0};

    W5500_CS_HIGH();
    HAL_Delay(50U);

    /* 注册 W5500 底层驱动 */
    reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);
    reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);
    reg_wizchip_spi_cbfunc(SPI_ReadByte, SPI_WriteByte);
    reg_wizchip_spiburst_cbfunc(SPI_BusRead, SPI_BusWrite);

    /* 配置 Socket 收发缓冲区，每个 2 KB */
    uint8_t tx_size[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    uint8_t rx_size[8] = {2, 2, 2, 2, 2, 2, 2, 2};

    if (wizchip_init(tx_size, rx_size) != 0)
    {
        return;
    }

    /* 配置 MAC 地址 */
    net_info.mac[0] = W5500_MAC0;
    net_info.mac[1] = W5500_MAC1;
    net_info.mac[2] = W5500_MAC2;
    net_info.mac[3] = W5500_MAC3;
    net_info.mac[4] = W5500_MAC4;
    net_info.mac[5] = W5500_MAC5;

    /* 配置静态 IP */
    net_info.ip[0] = 192U;
    net_info.ip[1] = 168U;
    net_info.ip[2] = 1U;
    net_info.ip[3] = 100U;

    /* 配置子网掩码 */
    net_info.sn[0] = 255U;
    net_info.sn[1] = 255U;
    net_info.sn[2] = 255U;
    net_info.sn[3] = 0U;

    /* 配置网关 */
    net_info.gw[0] = 192U;
    net_info.gw[1] = 168U;
    net_info.gw[2] = 1U;
    net_info.gw[3] = 1U;

    /* 配置 DNS */
    net_info.dns[0] = 192U;
    net_info.dns[1] = 168U;
    net_info.dns[2] = 1U;
    net_info.dns[3] = 1U;

    net_info.dhcp = NETINFO_STATIC;

    wizchip_setnetinfo(&net_info);

}

/* 创建 UDP 通信任务 */
void W5500_UdpInit(void)
{
    const osThreadAttr_t udp_task_attributes = {
        .name = "W5500_Udp_Task",
        .stack_size = 2048U,
        .priority = osPriorityNormal
    };

    if (s_udp_task_handle == NULL)
    {
        s_udp_task_handle = osThreadNew(
            W5500_UdpTask,
            NULL,
            &udp_task_attributes
        );
    }
}

/* UDP 回环任务 */
static void W5500_UdpTask(void *argument)
{
    (void)argument;

    uint8_t socket_status;
    uint8_t remote_ip[4];
    uint16_t remote_port;

    for (;;)
    {
        socket_status = getSn_SR(W5500_UDP_SOCKET);

        if (socket_status != SOCK_UDP)
        {
            if ((socket_status != SOCK_CLOSED) &&
                (close(W5500_UDP_SOCKET) < 0))
            {
                osDelay(100U);
                continue;
            }

            if (socket(
                W5500_UDP_SOCKET,
                Sn_MR_UDP,
                W5500_UDP_PORT,
                SF_IO_NONBLOCK
            ) < 0)
            {
                osDelay(100U);
                continue;
            }
        }

        /* 接收 UDP 数据 */
        int32_t received = recvfrom(
            W5500_UDP_SOCKET,
            s_udp_receive_buffer,
            sizeof(s_udp_receive_buffer),
            remote_ip,
            &remote_port
        );

        /* 收到数据后原样返回 */
        if (received > 0)
        {
            sendto(
                W5500_UDP_SOCKET,
                s_udp_receive_buffer,
                (uint16_t)received,
                remote_ip,
                remote_port
            );

        }

        osDelay(2U);
    }
}

