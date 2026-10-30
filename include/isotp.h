/*
 * isotp.h - ISO 15765-2 (ISO-TP) transport layer for classic CAN.
 *
 * - One link = one pair of CAN identifiers (normal addressing), full duplex:
 *   a link can send and receive a message at the same time.
 * - No dynamic memory, no global state, no OS dependency. The caller owns
 *   the buffers and supplies the time in microseconds.
 * - Messages up to 4095 bytes (12-bit first-frame length).
 * - Supports block size, STmin (ms and 100 us ranges), FC.WAIT, FC.OVFLW,
 *   N_Bs / N_Cr timeouts, sequence-number checking and frame padding.
 *
 * Integration:
 *   1. isotp_init() once.
 *   2. Call isotp_on_frame() for every received CAN frame whose ID equals
 *      cfg.rx_id (from the RX interrupt or a queue).
 *   3. Call isotp_poll() periodically (e.g. every 1 ms, or when a TX mailbox
 *      frees up): it sends due consecutive frames and checks timeouts.
 *   4. isotp_send() starts a transmission; isotp_rx_take() collects a
 *      completed reception.
 *
 * Not thread-safe: call all functions from one context, or guard the link.
 */
#ifndef ISOTP_H
#define ISOTP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ISOTP_MAX_LEN 4095u
#define ISOTP_NO_PADDING (-1)

/* Return 0 if the frame was queued, non-zero if the TX mailbox is full; the
 * link will retry on the next isotp_poll(). len is 1..8. */
typedef int (*isotp_send_fn)(void *user, uint32_t can_id, const uint8_t *data, uint8_t len);

typedef enum {
    ISOTP_IDLE = 0,
    ISOTP_BUSY,
    ISOTP_DONE,
    ISOTP_ERR_TIMEOUT_BS,  /* sender: no flow control within N_Bs */
    ISOTP_ERR_TIMEOUT_CR,  /* receiver: no consecutive frame within N_Cr */
    ISOTP_ERR_WRONG_SN,    /* receiver: consecutive frame out of sequence */
    ISOTP_ERR_OVERFLOW,    /* receiver buffer too small / peer reported overflow */
    ISOTP_ERR_WFT_OVRN,    /* sender: too many FC.WAIT in a row */
    ISOTP_ERR_INVALID,     /* bad arguments */
} isotp_status_t;

typedef struct {
    uint32_t tx_id;          /* identifier this node transmits on */
    uint32_t rx_id;          /* identifier this node listens to (informational) */
    isotp_send_fn send;
    void *user;

    uint8_t *rx_buf;         /* reassembly buffer */
    uint16_t rx_cap;

    uint8_t block_size;      /* BS we ask the peer for (0 = no limit) */
    uint8_t st_min;          /* STmin we ask the peer for, raw ISO encoding */
    int16_t padding;         /* fill byte for short frames, or ISOTP_NO_PADDING */
    uint16_t n_bs_ms;        /* sender timeout waiting for FC (default 1000) */
    uint16_t n_cr_ms;        /* receiver timeout waiting for CF (default 1000) */
    uint8_t wft_max;         /* max consecutive FC.WAIT accepted (default 8) */
} isotp_config_t;

typedef struct {
    isotp_config_t cfg;

    /* transmitter */
    const uint8_t *tx_data;
    uint16_t tx_len, tx_off;
    uint8_t tx_state, tx_sn, tx_bs, tx_bs_left, tx_wft;
    uint32_t tx_st_us;       /* peer-requested separation time */
    uint32_t tx_due_us;      /* earliest time for next CF */
    uint32_t tx_last_cf_us;  /* when the previous CF was queued */
    uint32_t tx_deadline_us; /* N_Bs */
    isotp_status_t tx_status;

    /* receiver */
    uint16_t rx_len, rx_off;
    uint8_t rx_state, rx_sn, rx_bs_count;
    uint8_t rx_fc_pending;   /* FC that could not be sent yet: 0 none, else FS+1 */
    uint32_t rx_deadline_us; /* N_Cr */
    isotp_status_t rx_status;
} isotp_link_t;

void isotp_init(isotp_link_t *l, const isotp_config_t *cfg);

/* Starts sending len bytes. data must stay valid until the status is no
 * longer ISOTP_BUSY. Returns ISOTP_BUSY, ISOTP_DONE (single frame sent),
 * or an error. */
isotp_status_t isotp_send(isotp_link_t *l, const uint8_t *data, uint16_t len, uint32_t now_us);

void isotp_on_frame(isotp_link_t *l, const uint8_t *data, uint8_t len, uint32_t now_us);
void isotp_poll(isotp_link_t *l, uint32_t now_us);

isotp_status_t isotp_tx_status(const isotp_link_t *l);
isotp_status_t isotp_rx_status(const isotp_link_t *l);

/* If a message has been received, returns its length and re-arms the
 * receiver; the data is in cfg.rx_buf. Returns -1 if nothing is ready.
 * Any receive error is reported once through rx_status and cleared here. */
int isotp_rx_take(isotp_link_t *l);

/* STmin raw byte -> microseconds. Reserved values map to 127 ms as the
 * standard requires. */
uint32_t isotp_stmin_to_us(uint8_t raw);

const char *isotp_status_str(isotp_status_t s);

#ifdef __cplusplus
}
#endif

#endif /* ISOTP_H */
