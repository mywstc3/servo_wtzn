#include "sts_eeprom.h"
#include "sts_mem_map.h"

#include "gd32f1x0_fmc.h"
#include <string.h>

#define STS_FLASH_EEPROM_ADDR    0x0800FC00U
#define STS_EEPROM_MAGIC0        0x55U
#define STS_EEPROM_MAGIC1        0xAAU
#define STS_EEPROM_DATA_SIZE     64U
#define STS_EPROM_SAVE_LEN       ((uint8_t)(STS_ADDR_EPROM_END - STS_ADDR_EPROM_BEGIN + 1U))

static uint8_t s_eeprom_buf[STS_EEPROM_DATA_SIZE];

uint8_t sts_eeprom_is_eprom_addr(uint8_t addr)
{
    return (uint8_t)((addr >= STS_ADDR_EPROM_BEGIN && addr <= STS_ADDR_EPROM_END)
        ? 1U : 0U);
}

void sts_eeprom_load(uint8_t *mem)
{
    uint32_t i;
    const uint32_t *src = (const uint32_t *)STS_FLASH_EEPROM_ADDR;
    uint32_t *dst = (uint32_t *)s_eeprom_buf;

    for (i = 0U; i < (STS_EEPROM_DATA_SIZE / 4U); i++) {
        dst[i] = src[i];
    }

    if (s_eeprom_buf[0] != STS_EEPROM_MAGIC0
        || s_eeprom_buf[1] != STS_EEPROM_MAGIC1) {
        return;
    }

    for (i = 0U; i < STS_EPROM_SAVE_LEN; i++) {
        mem[(uint8_t)(STS_ADDR_EPROM_BEGIN + i)] = s_eeprom_buf[i + 2U];
    }
}

void sts_eeprom_save(const uint8_t *mem)
{
    uint32_t i;
    uint32_t addr = STS_FLASH_EEPROM_ADDR;

    s_eeprom_buf[0] = STS_EEPROM_MAGIC0;
    s_eeprom_buf[1] = STS_EEPROM_MAGIC1;
    for (i = 0U; i < STS_EPROM_SAVE_LEN; i++) {
        s_eeprom_buf[i + 2U] = mem[(uint8_t)(STS_ADDR_EPROM_BEGIN + i)];
    }

    fmc_unlock();
    fmc_page_erase(STS_FLASH_EEPROM_ADDR);
    for (i = 0U; i < (STS_EEPROM_DATA_SIZE / 4U); i++) {
        fmc_word_program(addr, ((const uint32_t *)s_eeprom_buf)[i]);
        addr += 4U;
    }
    fmc_lock();
}
