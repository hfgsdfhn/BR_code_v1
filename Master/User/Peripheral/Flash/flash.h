/*
 * @file flash.h
 * @brief flash模块的用户接口与实现。
 */

#ifndef USER_FLASH_H
#define USER_FLASH_H

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
    uint32_t magic; /* 成员变量。 */
    uint32_t sequence; /* 成员变量。 */
    float angle; /* 成员变量。 */
    uint32_t crc; /* 成员变量。 */
} motor_angle_record_t;

#define FLASH_CMD_WRITE_ENABLE 0x06U
#define FLASH_CMD_READ_STATUS 0x05U
#define FLASH_CMD_READ_DATA 0x03U
#define FLASH_CMD_PAGE_PROGRAM 0x02U
#define FLASH_CMD_SECTOR_ERASE 0x20U
#define FLASH_CMD_DEVICE_ID 0x90U


/* 功能：读取设备参数。 */
uint16_t Flash_ReadId(void);


/* 功能：读取设备参数。 */
bool Flash_Read(uint32_t address                      , uint8_t *data                    , /* 数据缓冲区或数据帧。 */
                uint16_t size                );


/* 功能：写入设备参数。 */
bool Flash_WritePage(uint32_t address                       , /* 成员变量。 */
                     const uint8_t *data                  , uint16_t size                 );


/* 功能：执行 Flash_EraseSector 模块操作并维护相关状态。 */
bool Flash_EraseSector(uint32_t address                   );


/* 功能：执行 Flash_LoadMotorAngle 模块操作并维护相关状态。 */
bool Flash_LoadMotorAngle(uint8_t slot                       , /* 成员变量。 */
                          float *angle                          );

/* 功能：执行 Flash_SaveMotorAngle 模块操作并维护相关状态。 */
bool Flash_SaveMotorAngle(uint8_t slot                       , /* 成员变量。 */
                          float angle                      );

#endif
