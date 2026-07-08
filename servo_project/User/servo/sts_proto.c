#include "sts_proto.h"
#include "uart.h"
#include "sts_mem.h"
#include "sts_mem_map.h"
#include <stddef.h>

sts_frame_t g_sts_tx_frame[STS_TX_FRAME_CNT] = {0};
sts_frame_t g_sts_rx_frame[STS_RX_FRAME_CNT] = {0};
volatile uint8_t g_sts_rx_frame_head = 0U;
volatile uint8_t g_sts_rx_frame_tail = 0U;
volatile uint8_t g_sts_rx_frame_count = 0U;
sts_frame_t g_sts_rx_assemble = {0};

static uint8_t s_cs_buf[2U + STS_FRAME_DATA_MAX];
static uint8_t s_read_buf[STS_FRAME_DATA_MAX];
static uint8_t s_read_tx[STS_FRAME_DATA_MAX + 6U];

static void sts_frame_reset(sts_frame_t *frame)
{
    frame->frame_state = sts_frame_state_header1;
    frame->id = 0U;
    frame->length = 0U;
    frame->data_idx = 0U;
    frame->checksum_ok = 0U;
}

static uint8_t sts_checksum(const uint8_t *data, uint8_t len)
{
    uint16_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < len; i++) {
        sum = (uint16_t)(sum + data[i]);
    }
    return (uint8_t)(~sum);
}

static uint8_t sts_frame_verify(sts_frame_t *frame)
{
    uint8_t payload_len;
    uint8_t rx_cs;
    uint8_t calc_cs;

    if (frame->length < 2U) {
        return 0U;
    }
    payload_len = (uint8_t)(frame->length - 1U);
    if (payload_len >= STS_FRAME_DATA_MAX) {
        return 0U;
    }

    rx_cs = frame->data[payload_len];
    s_cs_buf[0] = frame->id;
    s_cs_buf[1] = frame->length;
    if (payload_len > 0U) {
        uint8_t i;
        for (i = 0U; i < payload_len; i++) {
            s_cs_buf[2U + i] = frame->data[i];
        }
    }
    calc_cs = sts_checksum(s_cs_buf, (uint8_t)(2U + payload_len));
    return (uint8_t)(calc_cs == rx_cs);
}

static uint8_t sts_rx_frame_ring_push(const sts_frame_t *frame)
{
    uint8_t slot;

    if (frame == NULL || g_sts_rx_frame_count >= STS_RX_FRAME_CNT) {
        return 0U;
    }

    slot = g_sts_rx_frame_head;
    g_sts_rx_frame[slot] = *frame;
    g_sts_rx_frame[slot].frame_state = sts_frame_state_ready;
    g_sts_rx_frame_head = (uint8_t)((g_sts_rx_frame_head + 1U) % STS_RX_FRAME_CNT);
    g_sts_rx_frame_count++;
    return 1U;
}

static void sts_rx_assemble_finish(void)
{
    g_sts_rx_assemble.checksum_ok = sts_frame_verify(&g_sts_rx_assemble);
    if (g_sts_rx_assemble.checksum_ok != 0U) {
        g_sts_rx_assemble.frame_state = sts_frame_state_ready;
        (void)sts_rx_frame_ring_push(&g_sts_rx_assemble);
    }
    sts_frame_reset(&g_sts_rx_assemble);
}

void sts_proto_init(void)
{
    uint8_t i;

    g_sts_rx_frame_head = 0U;
    g_sts_rx_frame_tail = 0U;
    g_sts_rx_frame_count = 0U;
    sts_frame_reset(&g_sts_rx_assemble);

    for (i = 0U; i < STS_RX_FRAME_CNT; i++) {
        g_sts_rx_frame[i].frame_state = sts_frame_state_idle;
        g_sts_tx_frame[i].frame_state = sts_frame_state_idle;
    }
    sts_mem_init();
}

void sts_proto_rx_frame_group(uint8_t byte)
{
    sts_frame_t *rx = &g_sts_rx_assemble;

    switch (rx->frame_state) {
    case sts_frame_state_header1:
        if (byte == STS_HEADER_0) {
            rx->frame_state = sts_frame_state_header2;
        }
        break;

    case sts_frame_state_header2:
        if (byte == STS_HEADER_1) {
            rx->frame_state = sts_frame_state_id;
        } else {
            rx->frame_state = sts_frame_state_header1;
        }
        break;

    case sts_frame_state_id:
        rx->id = byte;
        rx->frame_state = sts_frame_state_length;
        break;

    case sts_frame_state_length:
        rx->length = byte;
        rx->data_idx = 0U;
        if (byte < 2U || byte >= STS_FRAME_DATA_MAX) {
            sts_frame_reset(rx);
        } else {
            rx->frame_state = sts_frame_state_data;
        }
        break;

    case sts_frame_state_data:
        rx->data[rx->data_idx++] = byte;
        if (rx->data_idx >= rx->length) {
            sts_rx_assemble_finish();
        }
        break;

    case sts_frame_state_ready:
    case sts_frame_state_idle:
    default:
        sts_frame_reset(rx);
        if (byte == STS_HEADER_0) {
            rx->frame_state = sts_frame_state_header2;
        }
        break;
    }
}

uint8_t sts_proto_rx_frame_available(void)
{
    return g_sts_rx_frame_count;
}

uint8_t sts_proto_rx_frame_pop(sts_frame_t *out)
{
    uint8_t slot;

    if (out == NULL || g_sts_rx_frame_count == 0U) {
        return 0U;
    }

    slot = g_sts_rx_frame_tail;
    *out = g_sts_rx_frame[slot];
    g_sts_rx_frame[slot].frame_state = sts_frame_state_idle;
    g_sts_rx_frame_tail = (uint8_t)((g_sts_rx_frame_tail + 1U) % STS_RX_FRAME_CNT);
    g_sts_rx_frame_count--;
    return 1U;
}

static void sts_proto_process_one_frame(const sts_frame_t *frame)
{
    uint8_t inst;
    uint8_t err = 0U;
    uint8_t need_rsp;
    uint8_t id_rsp;

    if (frame == NULL || frame->checksum_ok == 0U || frame->length < 2U) {
        return;
    }
    if (frame->id != sts_mem_get_servo_id() && frame->id != STS_ID_BROADCAST) {
        return;
    }

    inst = frame->data[0];
    need_rsp = (uint8_t)((frame->id != STS_ID_BROADCAST) || (inst == STS_INST_PING));
    id_rsp = (frame->id == STS_ID_BROADCAST) ? sts_mem_get_servo_id() : frame->id;

    if (inst == STS_INST_PING) {
        if (need_rsp != 0U) {
            uint8_t tx_data[6];
            uint8_t plen = 2U;
            tx_data[0] = STS_HEADER_0;
            tx_data[1] = STS_HEADER_1;
            tx_data[2] = id_rsp;
            tx_data[3] = plen;
            tx_data[4] = sts_mem_get_error();
            tx_data[5] = sts_checksum(&tx_data[2], (uint8_t)(1U + plen));
            uart_comm_tx(tx_data, 6U);
        }
        return;
    }

    if (inst == STS_INST_READ) {
        uint8_t addr;
        uint8_t rlen;
        uint8_t plen;
        uint8_t i;

        if (frame->length < 4U) {
            return;
        }
        addr = frame->data[1];
        rlen = frame->data[2];
        if (rlen == 0U || rlen > STS_FRAME_DATA_MAX) {
            return;
        }
        if (sts_mem_read(addr, s_read_buf, rlen) != rlen) {
            return;
        }
        err = sts_mem_get_error();
        if (need_rsp == 0U) {
            return;
        }
        plen = (uint8_t)(2U + rlen);
        s_read_tx[0] = STS_HEADER_0;
        s_read_tx[1] = STS_HEADER_1;
        s_read_tx[2] = id_rsp;
        s_read_tx[3] = plen;
        s_read_tx[4] = err;
        for (i = 0U; i < rlen; i++) {
            s_read_tx[(uint8_t)(5U + i)] = s_read_buf[i];
        }
        s_read_tx[(uint8_t)(5U + rlen)] = sts_checksum(&s_read_tx[2], (uint8_t)(1U + plen));
        uart_comm_tx(s_read_tx, (uint8_t)(6U + rlen));
        return;
    }

    if (inst == STS_INST_WRITE) {
        uint8_t addr;
        uint8_t wlen;
        if (frame->length < 3U) {
            return;
        }
        addr = frame->data[1];
        wlen = (uint8_t)(frame->length - 3U);
        if (wlen == 0U) {
            return;
        }
        (void)sts_mem_write(addr, &frame->data[2], wlen);
        err = sts_mem_get_error();
        if (need_rsp != 0U) {
            uint8_t tx_data[6];
            uint8_t plen = 2U;
            tx_data[0] = STS_HEADER_0;
            tx_data[1] = STS_HEADER_1;
            tx_data[2] = id_rsp;
            tx_data[3] = plen;
            tx_data[4] = err;
            tx_data[5] = sts_checksum(&tx_data[2], (uint8_t)(1U + plen));
            uart_comm_tx(tx_data, 6U);
        }
    }
}

static void sts_proto_process_frames(void)
{
    sts_frame_t frame;

    while (sts_proto_rx_frame_pop(&frame) != 0U) {
        sts_proto_process_one_frame(&frame);
    }
}

void sts_proto_poll(void)
{
    uint8_t byte;

    while (uart_rx_pop(&byte) != 0U) {
        sts_proto_rx_frame_group(byte);
    }
    sts_proto_process_frames();
}
