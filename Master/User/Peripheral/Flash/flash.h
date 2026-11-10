/*
 * @file flash.h
 * @brief 声明 W25Qxx 外部 闪存 的基本操作。
 */

/* 外部 SPI 闪存 命令、引脚和公共接口声明。 */
#ifndef USER_FLASH_H
#define USER_FLASH_H /* 外部 闪存 头文件保护宏。 */

#include <stdbool.h>
#include <stdint.h>

#define FLASH_CAPACITY_BYTES 0x1000000UL
#define FLASH_PAGE_SIZE 256U
#define FLASH_IO_TIMEOUT_MS 100U
#define FLASH_PROGRAM_TIMEOUT_MS 20U
#define FLASH_ERASE_TIMEOUT_MS 3000U
#define FLASH_MOTOR_ANGLE_BASE 0x1F0000UL
#define FLASH_MOTOR_ANGLE_SECTOR 0x1000UL
#define FLASH_MOTOR_ANGLE_MAGIC 0x52534147UL

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    float angle;
    uint32_t crc;
} motor_angle_record_t;

#define FLASH_CMD_WRITE_ENABLE 0x06U /* W25Qxx 允许下一次写入或擦除操作的指令码。 */
#define FLASH_CMD_READ_STATUS 0x05U  /* W25Qxx 读取状态寄存器 1 的指令码。 */
#define FLASH_CMD_READ_DATA 0x03U    /* W25Qxx 按 24 位地址读取数据的指令码。 */
#define FLASH_CMD_PAGE_PROGRAM 0x02U /* W25Qxx 对指定页执行数据编程的指令码。 */
#define FLASH_CMD_SECTOR_ERASE 0x20U /* W25Qxx 擦除指定 4 KB 扇区的指令码。 */
#define FLASH_CMD_DEVICE_ID 0x90U    /* W25Qxx 读取制造商和器件标识的指令码。 */

/* 功能：读取 W25Qxx 芯片的制造商和器件 ID。 */
uint16_t Flash_ReadId(void);

/* 读、写、擦除操作在参数无效、SPI 出错或超时时返回 false。 */
bool Flash_Read(uint32_t address /* 闪存 中待读取数据的起始地址。 */, uint8_t *data /* 用于保存读取结果的缓冲区。 */,
                uint16_t size /* 本次读取的字节数。 */);

/* 功能：向一页内写入数据。 */
bool Flash_WritePage(uint32_t address /* 闪存 页内待写入数据的起始地址。 */,
                     const uint8_t *data /* 待写入 闪存 的数据。 */, uint16_t size /* 本次页编程的字节数。 */);

/* 功能：擦除地址所在的 4 KB 扇区。 */
bool Flash_EraseSector(uint32_t address /* 待擦除扇区内的任意地址。 */);

/* 从指定 闪存 槽位读取电机连续角度。 */
bool Flash_LoadMotorAngle(uint8_t slot /* 用于读取角度的 闪存 槽位编号。 */,
                          float *angle /* 用于保存读取到的电机角度，单位：弧度。 */);
/* 将电机连续角度保存到指定 闪存 槽位。 */
bool Flash_SaveMotorAngle(uint8_t slot /* 用于保存角度的 闪存 槽位编号。 */,
                          float angle /* 待保存的电机角度，单位：弧度。 */);

#endif
