#include "vbus.h"

#include <string.h>

uint32_t vbus_frame_bits(uint8_t len)
{
    const uint32_t g = 34u + 8u * len; /* bits exposed to stuffing */
    return 8u * len + 47u + (g - 1u) / 4u;
}

static int node_send(void *user, uint32_t id, const uint8_t *data, uint8_t len)
{
    vbus_node_t *n = user;
    if (n->pending >= VBUS_MAILBOX)
        return -1;
    vbus_frame_t *f = &n->mailbox[n->pending++];
    f->id = id;
    f->len = len;
    memcpy(f->data, data, len);
    return 0;
}

void vbus_init(vbus_t *b, uint32_t bitrate)
{
    memset(b, 0, sizeof(*b));
    b->bitrate = bitrate;
    b->tick_us = 5;
    b->poll_us = 100;
    b->busy_node = -1;
}

vbus_node_t *vbus_attach(vbus_t *b, isotp_config_t *cfg, uint32_t rx_id)
{
    vbus_node_t *n = &b->nodes[b->n_nodes];
    n->bus = b;
    n->index = b->n_nodes++;
    n->rx_id = rx_id;
    cfg->send = node_send;
    cfg->user = n;
    return n;
}

void vbus_bind(vbus_node_t *n, isotp_link_t *link) { n->link = link; }

static void trace_frame(vbus_t *b, const vbus_frame_t *f, int dropped)
{
    if (!b->trace)
        return;
    fprintf(b->trace, "(%4u.%06u) vcan0 %03X#", b->now_us / 1000000u, b->now_us % 1000000u, f->id);
    for (int i = 0; i < f->len; i++)
        fprintf(b->trace, "%02X", f->data[i]);
    fprintf(b->trace, "%s\n", dropped ? "   <- dropped" : "");
}

static void finish_frame(vbus_t *b)
{
    vbus_node_t *src = &b->nodes[b->busy_node];
    const vbus_frame_t f = src->mailbox[0];
    memmove(&src->mailbox[0], &src->mailbox[1], sizeof(f) * (size_t)(src->pending - 1));
    src->pending--;
    b->busy_node = -1;

    const int dropped = b->drop && b->drop(b->drop_user, &f, b->frames);
    b->frames++;
    trace_frame(b, &f, dropped);
    if (dropped)
        return;
    for (int i = 0; i < b->n_nodes; i++) {
        vbus_node_t *n = &b->nodes[i];
        if (i != src->index && n->link && n->rx_id == f.id)
            isotp_on_frame(n->link, f.data, f.len, b->now_us);
    }
}

static void arbitrate(vbus_t *b)
{
    int winner = -1;
    for (int i = 0; i < b->n_nodes; i++) {
        const vbus_node_t *n = &b->nodes[i];
        if (n->pending && (winner < 0 || n->mailbox[0].id < b->nodes[winner].mailbox[0].id))
            winner = i;
    }
    if (winner < 0)
        return;
    const uint32_t bits = vbus_frame_bits(b->nodes[winner].mailbox[0].len) + 3u; /* + intermission */
    const uint32_t dur = (bits * 1000000u + b->bitrate - 1u) / b->bitrate;
    b->busy_node = winner;
    b->busy_until_us = b->now_us + dur;
    b->busy_us += dur;
}

void vbus_step(vbus_t *b)
{
    b->now_us += b->tick_us;
    if (b->busy_node >= 0 && (int32_t)(b->now_us - b->busy_until_us) >= 0)
        finish_frame(b);
    if (b->now_us % b->poll_us < b->tick_us)
        for (int i = 0; i < b->n_nodes; i++)
            if (b->nodes[i].link)
                isotp_poll(b->nodes[i].link, b->now_us);
    if (b->busy_node < 0)
        arbitrate(b);
}

void vbus_run_for(vbus_t *b, uint32_t duration_us)
{
    const uint32_t end = b->now_us + duration_us;
    while ((int32_t)(end - b->now_us) > 0)
        vbus_step(b);
}
