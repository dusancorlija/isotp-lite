/*
 * vbus.h - discrete-time CAN / CAN FD bus simulator for host tests.
 *
 * Models what matters for a transport protocol:
 *   - per-node TX mailboxes of limited depth (send() fails when full),
 *   - bitwise arbitration: lowest identifier wins when the bus goes idle,
 *   - frame duration from the bit rate, with worst-case bit stuffing; on a
 *     CAN FD bus every frame is an FD frame with bit-rate switching,
 *   - fault injection: a hook may drop any frame on the wire,
 *   - a candump-style trace.
 */
#ifndef VBUS_H
#define VBUS_H

#include <stdint.h>
#include <stdio.h>

#include "isotp.h"

#define VBUS_MAX_NODES 4
#define VBUS_MAILBOX 3

typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
} vbus_frame_t;

typedef struct vbus vbus_t;

typedef struct {
    vbus_t *bus;
    int index;
    isotp_link_t *link;
    uint32_t rx_id;
    vbus_frame_t mailbox[VBUS_MAILBOX];
    int pending;
} vbus_node_t;

/* Return non-zero to drop the frame (it still occupies the bus). */
typedef int (*vbus_drop_fn)(void *user, const vbus_frame_t *f, uint32_t seq);

struct vbus {
    uint32_t bitrate;
    uint32_t data_bitrate;  /* CAN FD data phase; 0 = classic CAN bus */
    uint32_t now_us;
    uint32_t tick_us;
    uint32_t poll_us;
    vbus_node_t nodes[VBUS_MAX_NODES];
    int n_nodes;

    int busy_node;          /* node currently transmitting, -1 if idle */
    uint32_t busy_until_us;

    vbus_drop_fn drop;
    void *drop_user;
    FILE *trace;

    uint32_t frames;        /* frames completed on the wire */
    uint64_t busy_us;       /* total time the bus was occupied */
};

void vbus_init(vbus_t *b, uint32_t bitrate);
/* CAN FD bus: arbitration at bitrate, data phase at data_bitrate (BRS). */
void vbus_init_fd(vbus_t *b, uint32_t bitrate, uint32_t data_bitrate);

/* Registers a link; fills in cfg.send / cfg.user. Call before isotp_init. */
vbus_node_t *vbus_attach(vbus_t *b, isotp_config_t *cfg, uint32_t rx_id);
void vbus_bind(vbus_node_t *n, isotp_link_t *link);

void vbus_step(vbus_t *b);
void vbus_run_for(vbus_t *b, uint32_t duration_us);

/* Classic CAN, 11-bit ID, worst-case stuffing (Davis et al., 2007). */
uint32_t vbus_frame_bits(uint8_t len);

/* Time on the wire of one frame including intermission, in microseconds
 * (rounded up), classic or FD depending on the bus. */
uint32_t vbus_frame_us(const vbus_t *b, uint8_t len);

#endif /* VBUS_H */
