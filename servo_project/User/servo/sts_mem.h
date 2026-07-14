#ifndef __STS_MEM_H__
#define __STS_MEM_H__

#include <stdint.h>

void sts_mem_init(void);
void sts_mem_poll(void);
uint8_t sts_mem_read(uint8_t addr, uint8_t *out, uint8_t len);
uint8_t sts_mem_write(uint8_t addr, const uint8_t *data, uint8_t len);
void sts_mem_refresh_feedback(void);

uint8_t sts_mem_get_error(void);
uint8_t sts_mem_get_servo_id(void);
uint8_t sts_mem_control_active(void);
uint16_t sts_mem_get_min_start_force(void);
void sts_mem_set_magnet_ok(uint8_t ok);
void sts_mem_calibrate_midpoint(uint16_t target_raw);
void sts_mem_reset_offset(void);

#endif
