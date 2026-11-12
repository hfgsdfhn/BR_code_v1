





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
    uint32_t magic;
    uint32_t sequence;
    float angle;
    uint32_t crc;
} motor_angle_record_t;

#define FLASH_CMD_WRITE_ENABLE 0x06U
#define FLASH_CMD_READ_STATUS 0x05U
#define FLASH_CMD_READ_DATA 0x03U
#define FLASH_CMD_PAGE_PROGRAM 0x02U
#define FLASH_CMD_SECTOR_ERASE 0x20U
#define FLASH_CMD_DEVICE_ID 0x90U


uint16_t Flash_ReadId(void);


bool Flash_Read(uint32_t address                      , uint8_t *data                    ,
                uint16_t size                );


bool Flash_WritePage(uint32_t address                       ,
                     const uint8_t *data                  , uint16_t size                 );


bool Flash_EraseSector(uint32_t address                   );


bool Flash_LoadMotorAngle(uint8_t slot                       ,
                          float *angle                          );

bool Flash_SaveMotorAngle(uint8_t slot                       ,
                          float angle                      );

#endif
