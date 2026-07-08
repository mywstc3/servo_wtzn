#ifndef __STS_PROTO_H__
#define __STS_PROTO_H__

#include <stdint.h>

#define STS_HEADER_0            0xFFU
#define STS_HEADER_1            0xFFU

#define STS_RX_FRAME_CNT        16U
#define STS_TX_FRAME_CNT        16U
#define STS_FRAME_DATA_MAX      128U

#define STS_INST_PING           0x01U
#define STS_INST_READ           0x02U
#define STS_INST_WRITE          0x03U

typedef enum {
    sts_frame_state_idle = 0,
    sts_frame_state_header1,
    sts_frame_state_header2,
    sts_frame_state_id,
    sts_frame_state_length,
    sts_frame_state_data,
    sts_frame_state_discard,
    sts_frame_state_ready,
} sts_frame_state_t;

typedef struct {
    sts_frame_state_t frame_state;
    uint8_t id;
    uint8_t length;
    uint8_t data_idx;
    uint8_t checksum_ok;
    uint8_t data[STS_FRAME_DATA_MAX];
} sts_frame_t;

/* 已完成帧环形队列（Keil Watch: head/tail/count + slots[]） */
extern sts_frame_t g_sts_rx_frame[STS_RX_FRAME_CNT];
extern volatile uint8_t g_sts_rx_frame_head;
extern volatile uint8_t g_sts_rx_frame_tail;
extern volatile uint8_t g_sts_rx_frame_count;

/* 正在组帧的缓冲（字节状态机写入此处，完成后 push 进环） */
extern sts_frame_t g_sts_rx_assemble;

extern sts_frame_t g_sts_tx_frame[STS_RX_FRAME_CNT];

void sts_proto_init(void);
void sts_proto_poll(void);
void sts_proto_rx_frame_group(uint8_t byte);
uint8_t sts_proto_rx_frame_available(void);
uint8_t sts_proto_rx_frame_pop(sts_frame_t *out);

#endif
