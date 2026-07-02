#ifndef __DATA_SEND_H
#define __DATA_SEND_H

#include <stdint.h>
#define JF_MAX_CH   8
#define JF_N(...)       (sizeof((float[]){__VA_ARGS__}) / sizeof(float))
#define JF_SEND(...)    justfloat_dma_send((float[]){__VA_ARGS__}, JF_N(__VA_ARGS__))

void justfloat_dma_send(const float *ch, uint8_t ch_num);
uint8_t justfloat_dma_busy(void);

#endif
