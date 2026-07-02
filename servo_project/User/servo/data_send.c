#include "data_send.h"
#include "bsp/bap_uart.h"
#include <stddef.h>

static const uint8_t JF_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};
static uint8_t jf_tx_buf[JF_MAX_CH * 4 + 4];

static void jf_copy4(uint8_t *dst, const uint8_t *src)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
}

uint8_t justfloat_dma_busy(void)
{
    return bap_uart_tx_dma_busy() ? 1U : 0U;
}

void justfloat_dma_send(const float *ch, uint8_t ch_num)
{
    uint16_t n = 0U;

    if (bap_uart_tx_dma_busy() || ch == NULL || ch_num == 0U || ch_num > JF_MAX_CH) {
        return;
    }

    for (uint8_t i = 0U; i < ch_num; i++) {
        jf_copy4(&jf_tx_buf[n], (const uint8_t *)&ch[i]);
        n += 4U;
    }
    jf_copy4(&jf_tx_buf[n], JF_TAIL);
    n += 4U;

    (void)bap_uart_tx_dma_start(jf_tx_buf, n);
}
