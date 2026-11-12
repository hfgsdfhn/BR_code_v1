/*
 * @file flash.c
 * @brief flash模块的用户接口与实现。
 */

#include "flash.h"
#include "main.h"
#include "spi.h"
#include <math.h>
#include <stddef.h>
#include <string.h>


/* 功能：执行 angle_crc 模块操作并维护相关状态。 */
static uint32_t angle_crc(const motor_angle_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t crc = 2166136261UL;
    uint32_t i;
    for (i = 0U; i < offsetof(motor_angle_record_t, crc); ++i)
    {
        crc ^= bytes[i];
        crc *= 16777619UL;
    }
    return crc;
}


/* 功能：执行 angle_record_valid 模块操作并维护相关状态。 */
static bool angle_record_valid(const motor_angle_record_t *record)
{
    return record->magic == FLASH_MOTOR_ANGLE_MAGIC && isfinite(record->angle) && record->crc == angle_crc(record);
}


/* 功能：执行 Flash_Select 模块操作并维护相关状态。 */
static void Flash_Select(GPIO_PinState state)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, state);
}


/* 功能：读取设备参数。 */
static bool Flash_ReadStatus(uint8_t *status)
{
    uint8_t command = FLASH_CMD_READ_STATUS;
    bool ok;
    Flash_Select(GPIO_PIN_RESET);
    ok = HAL_SPI_Transmit(&hspi1, &command, 1U, FLASH_IO_TIMEOUT_MS) == HAL_OK &&
         HAL_SPI_Receive(&hspi1, status, 1U, FLASH_IO_TIMEOUT_MS) == HAL_OK;
    Flash_Select(GPIO_PIN_SET);
    return ok && *status != 0xFFU;
}


/* 功能：执行 Flash_Wait 模块操作并维护相关状态。 */
static bool Flash_Wait(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t status = 0U;
    do
    {
        if (!Flash_ReadStatus(&status))
        {
            return false;
        }
        if ((status & 1U) == 0U)
        {
            return true;
        }
    } while ((uint32_t)(HAL_GetTick() - start) < timeout_ms);
    return false;
}


/* 功能：写入设备参数。 */
static bool Flash_WriteEnable(void)
{
    uint8_t command = FLASH_CMD_WRITE_ENABLE;
    uint8_t status = 0U;
    bool ok;
    Flash_Select(GPIO_PIN_RESET);
    ok = HAL_SPI_Transmit(&hspi1, &command, 1U, FLASH_IO_TIMEOUT_MS) == HAL_OK;
    Flash_Select(GPIO_PIN_SET);
    return ok && Flash_ReadStatus(&status) && (status & 2U) != 0U;
}


/* 功能：发送数据或控制指令。 */
static bool Flash_SendAddress(uint8_t command, uint32_t address)
{
    uint8_t header[4] = {command, (uint8_t)(address >> 16U), (uint8_t)(address >> 8U), (uint8_t)address};
/* 功能：执行 HAL_SPI_Transmit 模块操作并维护相关状态。 */
    return HAL_SPI_Transmit(&hspi1, header, sizeof(header), FLASH_IO_TIMEOUT_MS) == HAL_OK;
}


/* 功能：读取设备参数。 */
uint16_t Flash_ReadId(void)
{
    uint8_t command[4] = {FLASH_CMD_DEVICE_ID, 0U, 0U, 0U};
    uint8_t id[2] = {0U, 0U};
    bool ok;
    Flash_Select(GPIO_PIN_RESET);
    ok = HAL_SPI_Transmit(&hspi1, command, sizeof(command), FLASH_IO_TIMEOUT_MS) == HAL_OK &&
         HAL_SPI_Receive(&hspi1, id, sizeof(id), FLASH_IO_TIMEOUT_MS) == HAL_OK;
    Flash_Select(GPIO_PIN_SET);
    if (!ok || id[0] == 0U || id[0] == 0xFFU || id[1] == 0U || id[1] == 0xFFU)
    {
        return 0U;
    }
    return ((uint16_t)id[0] << 8U) | id[1];
}


/* 功能：读取设备参数。 */
bool Flash_Read(uint32_t address, uint8_t *data, uint16_t size)
{
    bool ok;
    if (data == NULL || size == 0U || address >= FLASH_CAPACITY_BYTES || size > FLASH_CAPACITY_BYTES - address)
    {
        return false;
    }
    if (!Flash_Wait(FLASH_ERASE_TIMEOUT_MS))
    {
        return false;
    }
    Flash_Select(GPIO_PIN_RESET);
    ok = Flash_SendAddress(FLASH_CMD_READ_DATA, address) &&
         HAL_SPI_Receive(&hspi1, data, size, FLASH_IO_TIMEOUT_MS) == HAL_OK;
    Flash_Select(GPIO_PIN_SET);
    return ok;
}


/* 功能：写入设备参数。 */
bool Flash_WritePage(uint32_t address, const uint8_t *data, uint16_t size)
{
    bool ok;
    if (data == NULL || size == 0U || address >= FLASH_CAPACITY_BYTES || size > FLASH_CAPACITY_BYTES - address ||
        size > FLASH_PAGE_SIZE - (address % FLASH_PAGE_SIZE))
    {
        return false;
    }
    if (!Flash_Wait(FLASH_ERASE_TIMEOUT_MS) || !Flash_WriteEnable())
    {
        return false;
    }
    Flash_Select(GPIO_PIN_RESET);
    ok = Flash_SendAddress(FLASH_CMD_PAGE_PROGRAM, address) &&
         HAL_SPI_Transmit(&hspi1, (uint8_t *)data, size, FLASH_IO_TIMEOUT_MS) == HAL_OK;
    Flash_Select(GPIO_PIN_SET);
    return ok && Flash_Wait(FLASH_PROGRAM_TIMEOUT_MS);
}


/* 功能：执行 Flash_EraseSector 模块操作并维护相关状态。 */
bool Flash_EraseSector(uint32_t address)
{
    bool ok;
    if (address >= FLASH_CAPACITY_BYTES)
    {
        return false;
    }
    if (!Flash_Wait(FLASH_ERASE_TIMEOUT_MS) || !Flash_WriteEnable())
    {
        return false;
    }
    Flash_Select(GPIO_PIN_RESET);
    ok = Flash_SendAddress(FLASH_CMD_SECTOR_ERASE, address);
    Flash_Select(GPIO_PIN_SET);
    return ok && Flash_Wait(FLASH_ERASE_TIMEOUT_MS);
}


/* 功能：执行 angle_record_newer 模块操作并维护相关状态。 */
static bool angle_record_newer(const motor_angle_record_t *a, const motor_angle_record_t *b)
{
    uint32_t delta = a->sequence - b->sequence;
    return delta != 0U && delta < 0x80000000UL;
}


/* 功能：执行 Flash_LoadMotorAngle 模块操作并维护相关状态。 */
bool Flash_LoadMotorAngle(uint8_t slot, float *angle)
{
    motor_angle_record_t a = {0}, b = {0};
    uint32_t base;
    bool valid_a, valid_b;
    if (slot > 1U || angle == NULL)
    {
        return false;
    }
    base = FLASH_MOTOR_ANGLE_BASE + (uint32_t)slot * 2U * FLASH_MOTOR_ANGLE_SECTOR;
    valid_a = Flash_Read(base, (uint8_t *)&a, sizeof(a)) && angle_record_valid(&a);
    valid_b = Flash_Read(base + FLASH_MOTOR_ANGLE_SECTOR, (uint8_t *)&b, sizeof(b)) && angle_record_valid(&b);
    if (!valid_a && !valid_b)
    {
        return false;
    }
    *angle = (!valid_b || (valid_a && !angle_record_newer(&b, &a))) ? a.angle : b.angle;
    return true;
}


/* 功能：执行 Flash_SaveMotorAngle 模块操作并维护相关状态。 */
bool Flash_SaveMotorAngle(uint8_t slot, float angle)
{
    motor_angle_record_t a = {0}, b = {0}, record = {0}, verify = {0};
    uint32_t base, address;
    bool valid_a, valid_b;
    if (slot > 1U || !isfinite(angle))
    {
        return false;
    }
    base = FLASH_MOTOR_ANGLE_BASE + (uint32_t)slot * 2U * FLASH_MOTOR_ANGLE_SECTOR;

    if (!Flash_Read(base, (uint8_t *)&a, sizeof(a)) ||
        !Flash_Read(base + FLASH_MOTOR_ANGLE_SECTOR, (uint8_t *)&b, sizeof(b)))
    {
        return false;
    }
    valid_a = angle_record_valid(&a);
    valid_b = angle_record_valid(&b);
    record.magic = FLASH_MOTOR_ANGLE_MAGIC;
    record.angle = angle;
    if (valid_b && (!valid_a || angle_record_newer(&b, &a)))
    {
        record.sequence = b.sequence + 1U;
        address = base;
    }
    else
    {
        record.sequence = valid_a ? a.sequence + 1U : 1U;
        address = base + FLASH_MOTOR_ANGLE_SECTOR;
    }
    record.crc = angle_crc(&record);
/* 功能：执行 Flash_EraseSector 模块操作并维护相关状态。 */
    return Flash_EraseSector(address) && Flash_WritePage(address, (const uint8_t *)&record, sizeof(record)) &&
           Flash_Read(address, (uint8_t *)&verify, sizeof(verify)) && memcmp(&record, &verify, sizeof(record)) == 0;
}