/**
 * @file    settings_flash.c
 * @brief   Settings persistence in flash sector 11 (reserved in the linker script).
 */
#include "settings.h"
#include "stm32f4xx_hal.h"

#include <string.h>

#define SETTINGS_FLASH_ADDRESS  0x080E0000u
#define SETTINGS_FLASH_SECTOR   FLASH_SECTOR_11

bool settings_save(const settings_t *s)
{
    const uint32_t *words = (const uint32_t *)s;
    const uint32_t num_words = sizeof(*s) / sizeof(uint32_t);
    bool success = true;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
        .Sector = SETTINGS_FLASH_SECTOR,
        .NbSectors = 1,
    };
    uint32_t sector_error = 0;

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        success = false;
    }

    // Magic (word 0) last, so an interrupted save is never seen as valid
    for (uint32_t i = 1; success && i < num_words; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SETTINGS_FLASH_ADDRESS + i * 4, words[i]) != HAL_OK)
        {
            success = false;
        }
    }
    if (success &&
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SETTINGS_FLASH_ADDRESS, words[0]) != HAL_OK)
    {
        success = false;
    }

    if (success && memcmp((const void *)SETTINGS_FLASH_ADDRESS, s, sizeof(*s)) != 0)
    {
        success = false;
    }

    HAL_FLASH_Lock();
    return success;
}

bool settings_load(settings_t *out)
{
    memcpy(out, (const void *)SETTINGS_FLASH_ADDRESS, sizeof(*out));
    return settings_valid(out);
}
