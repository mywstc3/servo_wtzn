#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>

#define ENCODER_UPDATE_HZ    100U   /* 位置采样 100 Hz（1 kHz 控制环每 10 次采一次） */

void encoder_init(void);
uint8_t encoder_update(void);   /* 1=读成功，0=失败 */

#endif
