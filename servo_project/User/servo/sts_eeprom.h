#ifndef __STS_EEPROM_H__
#define __STS_EEPROM_H__

#include <stdint.h>

void sts_eeprom_load(uint8_t *mem);
void sts_eeprom_save(const uint8_t *mem);
uint8_t sts_eeprom_is_eprom_addr(uint8_t addr);

#endif
