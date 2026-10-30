#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "isotp.h"
#include "vbus.h"

#define TESTER_ID 0x7E0
#define ECU_ID 0x7E8

/* ── harness: two links on a simulated 500 kbit/s bus ───────────────────── */

typedef struct {
    vbus_t bus;
    isotp_link_t tester, ecu;
    uint8_t tester_rx[ISOTP_MAX_LEN], ecu_rx[ISOTP_MAX_LEN];
} pair_t;

static void pair_init(pair_t *p, uint8_t bs, uint8_t stmin, uint16_t ecu_cap)
{
    vbus_init(&p->bus, 500000);
    isotp_config_t a = { .tx_id = TESTER_ID, .rx_id = ECU_ID, .rx_buf = p->tester_rx,
                         .rx_cap = ISOTP_MAX_LEN, .padding = 0xCC };
    isotp_config_t b = { .tx_id = ECU_ID, .rx_id = TESTER_ID, .rx_buf = p->ecu_rx,
                         .rx_cap = ecu_cap, .block_size = bs, .st_min = stmin, .padding = 0xCC };
    vbus_node_t *na = vbus_attach(&p->bus, &a, ECU_ID);
    vbus_node_t *nb = vbus_attach(&p->bus, &b, TESTER_ID);
    isotp_init(&p->tester, &a);
    isotp_init(&p->ecu, &b);
    vbus_bind(na, &p->tester);
    vbus_bind(nb, &p->ecu);
}

/* Runs until the transfer settles or the deadline passes. */
static void run_until_idle(pair_t *p, uint32_t max_us)
{
    const uint32_t end = p->bus.now_us + max_us;
    while ((int32_t)(end - p->bus.now_us) > 0) {
        vbus_step(&p->bus);
        if (isotp_tx_status(&p->tester) != ISOTP_BUSY && isotp_rx_status(&p->ecu) != ISOTP_BUSY &&
            p->bus.busy_node < 0 && !p->bus.nodes[0].pending && !p->bus.nodes[1].pending)
            break;
    }
}

static uint8_t msg[ISOTP_MAX_LEN];

static void fill(uint16_t len, unsigned seed)
{
    srand(seed);
    for (uint16_t i = 0; i < len; i++) msg[i] = (uint8_t)rand();
}

static int transfer(uint16_t len, uint8_t bs, uint8_t stmin)
{
    pair_t p;
    pair_init(&p, bs, stmin, ISOTP_MAX_LEN);
    fill(len, len * 31u + bs);
    isotp_send(&p.tester, msg, len, p.bus.now_us);
    run_until_idle(&p, 10000000);
    const int got = isotp_rx_take(&p.ecu);
    return isotp_tx_status(&p.tester) == ISOTP_DONE && got == len && memcmp(p.ecu_rx, msg, len) == 0;
}

/* ── tests ───────────────────────────────────────────────────────────────── */

static void stmin_encoding(void)
{
    CHECK_EQ(isotp_stmin_to_us(0x00), 0);
    CHECK_EQ(isotp_stmin_to_us(0x7F), 127000);
    CHECK_EQ(isotp_stmin_to_us(0xF1), 100);
    CHECK_EQ(isotp_stmin_to_us(0xF9), 900);
    CHECK_EQ(isotp_stmin_to_us(0x80), 127000); /* reserved */
    CHECK_EQ(isotp_stmin_to_us(0xFA), 127000); /* reserved */
}

static void single_frames(void)
{
    for (uint16_t len = 1; len <= 7; len++)
        CHECK(transfer(len, 0, 0));
}

static void multi_frame_sizes(void)
{
    const uint16_t sizes[] = { 8, 9, 13, 14, 15, 62, 111, 112, 113, 1000, 4094, 4095 };
    for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++)
        CHECK(transfer(sizes[i], 0, 0));
}

static void block_size_and_stmin_matrix(void)
{
    const uint8_t bss[] = { 0, 1, 2, 8, 15 };
    const uint8_t sts[] = { 0x00, 0xF3, 0x01, 0x05 };
    for (size_t i = 0; i < sizeof(bss); i++)
        for (size_t j = 0; j < sizeof(sts); j++)
            CHECK(transfer(300, bss[i], sts[j]));
}

static void sequence_number_wraps(void)
{
    /* 4095 bytes = 1 FF + 585 CFs: SN wraps 0..15 thirty-six times. */
    CHECK(transfer(4095, 0, 0));
}

/* Inter-frame gap of consecutive frames on the wire honours STmin. */
static uint32_t last_cf_us, min_gap_us;
static int gap_probe(void *user, const vbus_frame_t *f, uint32_t seq)
{
    vbus_t *b = user;
    (void)seq;
    if (f->id == TESTER_ID && (f->data[0] >> 4) == 2) {
        if (last_cf_us && b->now_us - last_cf_us < min_gap_us) min_gap_us = b->now_us - last_cf_us;
        last_cf_us = b->now_us;
    }
    return 0;
}

static void stmin_is_respected(void)
{
    /* BS = 4 also checks the gap across block boundaries, where an FC sits
     * between two consecutive frames. */
    for (uint8_t bs = 0; bs <= 4; bs = (uint8_t)(bs + 4)) {
        pair_t p;
        pair_init(&p, bs, 0x02, ISOTP_MAX_LEN); /* 2 ms */
        p.bus.drop = gap_probe;
        p.bus.drop_user = &p.bus;
        last_cf_us = 0;
        min_gap_us = UINT32_MAX;
        fill(200, 1);
        isotp_send(&p.tester, msg, 200, p.bus.now_us);
        run_until_idle(&p, 5000000);
        CHECK_EQ(isotp_tx_status(&p.tester), ISOTP_DONE);
        CHECK(min_gap_us >= 2000);
    }
}

static void receiver_overflow(void)
{
    pair_t p;
    pair_init(&p, 0, 0, 100);
    fill(500, 2);
    isotp_send(&p.tester, msg, 500, p.bus.now_us);
    run_until_idle(&p, 1000000);
    CHECK_EQ(isotp_tx_status(&p.tester), ISOTP_ERR_OVERFLOW);
    CHECK_EQ(isotp_rx_status(&p.ecu), ISOTP_ERR_OVERFLOW);
    CHECK_EQ(isotp_rx_take(&p.ecu), -1);
    CHECK_EQ(isotp_rx_status(&p.ecu), ISOTP_IDLE);
}

static int drop_seq_n(void *user, const vbus_frame_t *f, uint32_t seq)
{
    (void)f;
    return seq == *(uint32_t *)user;
}

static void lost_flow_control_times_out(void)
{
    pair_t p;
    uint32_t drop = 1; /* frame 0 = FF, frame 1 = FC */
    pair_init(&p, 0, 0, ISOTP_MAX_LEN);
    p.bus.drop = drop_seq_n;
    p.bus.drop_user = &drop;
    fill(100, 3);
    isotp_send(&p.tester, msg, 100, p.bus.now_us);
    vbus_run_for(&p.bus, 999000);
    CHECK_EQ(isotp_tx_status(&p.tester), ISOTP_BUSY);
    vbus_run_for(&p.bus, 2000);
    CHECK_EQ(isotp_tx_status(&p.tester), ISOTP_ERR_TIMEOUT_BS);
}

static void lost_consecutive_frame_is_detected(void)
{
    pair_t p;
    uint32_t drop = 5; /* FF, FC, CF1, CF2, CF3 -> CF3 lost */
    pair_init(&p, 0, 0, ISOTP_MAX_LEN);
    p.bus.drop = drop_seq_n;
    p.bus.drop_user = &drop;
    fill(100, 4);
    isotp_send(&p.tester, msg, 100, p.bus.now_us);
    run_until_idle(&p, 2000000);
    CHECK_EQ(isotp_rx_status(&p.ecu), ISOTP_ERR_WRONG_SN);
}

static void lost_last_frame_times_out(void)
{
    pair_t p;
    uint32_t drop = 3; /* 20 bytes: FF, FC, CF1, CF2(last) */
    pair_init(&p, 0, 0, ISOTP_MAX_LEN);
    p.bus.drop = drop_seq_n;
    p.bus.drop_user = &drop;
    fill(20, 5);
    isotp_send(&p.tester, msg, 20, p.bus.now_us);
    vbus_run_for(&p.bus, 500000);
    CHECK_EQ(isotp_rx_status(&p.ecu), ISOTP_BUSY);
    vbus_run_for(&p.bus, 600000);
    CHECK_EQ(isotp_rx_status(&p.ecu), ISOTP_ERR_TIMEOUT_CR);
}

static void full_duplex(void)
{
    pair_t p;
    pair_init(&p, 4, 0, ISOTP_MAX_LEN);
    static uint8_t a[700], b[900];
    for (int i = 0; i < 700; i++) a[i] = (uint8_t)(i * 7);
    for (int i = 0; i < 900; i++) b[i] = (uint8_t)(i * 13 + 1);
    isotp_send(&p.tester, a, 700, p.bus.now_us);
    isotp_send(&p.ecu, b, 900, p.bus.now_us);
    vbus_run_for(&p.bus, 2000000);
    CHECK_EQ(isotp_tx_status(&p.tester), ISOTP_DONE);
    CHECK_EQ(isotp_tx_status(&p.ecu), ISOTP_DONE);
    CHECK_EQ(isotp_rx_take(&p.ecu), 700);
    CHECK_EQ(isotp_rx_take(&p.tester), 900);
    CHECK(memcmp(p.ecu_rx, a, 700) == 0);
    CHECK(memcmp(p.tester_rx, b, 900) == 0);
}

/* ── raw frame tests: craft the peer's frames by hand ───────────────────── */

typedef struct { uint8_t data[64][8]; uint8_t len[64]; int n; int full; } capture_t;

static int capture_send(void *user, uint32_t id, const uint8_t *d, uint8_t len)
{
    capture_t *c = user;
    (void)id;
    if (c->full) return -1;
    memcpy(c->data[c->n], d, len);
    c->len[c->n++] = len;
    return 0;
}

static void raw_link(isotp_link_t *l, capture_t *c, uint8_t *buf, uint16_t cap)
{
    memset(c, 0, sizeof(*c));
    isotp_config_t cfg = { .tx_id = 1, .rx_id = 2, .send = capture_send, .user = c,
                           .rx_buf = buf, .rx_cap = cap, .padding = ISOTP_NO_PADDING, .wft_max = 2 };
    isotp_init(l, &cfg);
}

static void flow_control_wait(void)
{
    isotp_link_t l;
    capture_t c;
    uint8_t buf[8];
    raw_link(&l, &c, buf, sizeof(buf));
    fill(30, 6);
    isotp_send(&l, msg, 30, 0);
    const uint8_t wait[] = { 0x31, 0, 0 }, cts[] = { 0x30, 0, 0 };
    isotp_on_frame(&l, wait, 3, 900000);
    isotp_poll(&l, 1500000); /* N_Bs restarted by WAIT: not expired */
    CHECK_EQ(isotp_tx_status(&l), ISOTP_BUSY);
    isotp_on_frame(&l, cts, 3, 1600000);
    CHECK_EQ(isotp_tx_status(&l), ISOTP_DONE);
    CHECK_EQ(c.n, 1 + 4); /* FF + 4 CFs */
    CHECK_EQ(c.len[4], 1 + 3); /* no padding: last CF is short */

    /* Exceeding wft_max aborts. */
    raw_link(&l, &c, buf, sizeof(buf));
    isotp_send(&l, msg, 30, 0);
    for (int i = 0; i < 3; i++) isotp_on_frame(&l, wait, 3, 1000);
    CHECK_EQ(isotp_tx_status(&l), ISOTP_ERR_WFT_OVRN);
}

static void mailbox_full_is_retried(void)
{
    isotp_link_t l;
    capture_t c;
    uint8_t buf[8];
    raw_link(&l, &c, buf, sizeof(buf));
    fill(30, 7);
    isotp_send(&l, msg, 30, 0);
    c.full = 1;
    const uint8_t cts[] = { 0x30, 0, 0 };
    isotp_on_frame(&l, cts, 3, 10);
    CHECK_EQ(c.n, 1);
    c.full = 0;
    isotp_poll(&l, 20);
    CHECK_EQ(c.n, 5);
    CHECK_EQ(isotp_tx_status(&l), ISOTP_DONE);
}

static void invalid_frames_are_ignored(void)
{
    isotp_link_t l;
    capture_t c;
    uint8_t buf[64];
    raw_link(&l, &c, buf, sizeof(buf));
    const uint8_t sf0[] = { 0x00, 1, 2 }, sf8[] = { 0x08, 1, 2, 3, 4, 5, 6, 7 };
    const uint8_t sf_short[] = { 0x05, 1, 2 }, ff_small[] = { 0x10, 0x05, 1, 2, 3, 4, 5, 6 };
    const uint8_t cf_stray[] = { 0x21, 1, 2, 3, 4, 5, 6, 7 }, fc_stray[] = { 0x30, 0, 0 };
    isotp_on_frame(&l, sf0, 3, 0);
    isotp_on_frame(&l, sf8, 8, 0);
    isotp_on_frame(&l, sf_short, 3, 0);
    isotp_on_frame(&l, ff_small, 8, 0);
    isotp_on_frame(&l, cf_stray, 8, 0);
    isotp_on_frame(&l, fc_stray, 3, 0);
    CHECK_EQ(isotp_rx_status(&l), ISOTP_IDLE);
    CHECK_EQ(c.n, 0);
    CHECK_EQ(isotp_send(&l, msg, 0, 0), ISOTP_ERR_INVALID);
    CHECK_EQ(isotp_send(&l, msg, 4096, 0), ISOTP_ERR_INVALID);
}

static void untaken_message_is_not_overwritten(void)
{
    isotp_link_t l;
    capture_t c;
    uint8_t buf[64];
    raw_link(&l, &c, buf, sizeof(buf));
    const uint8_t a[] = { 0x03, 'a', 'b', 'c' }, b[] = { 0x03, 'x', 'y', 'z' };
    isotp_on_frame(&l, a, 4, 0);
    isotp_on_frame(&l, b, 4, 0);
    CHECK_EQ(isotp_rx_take(&l), 3);
    CHECK(memcmp(buf, "abc", 3) == 0);
}

static void timer_wraparound(void)
{
    /* The microsecond clock wraps every 71.6 minutes. */
    isotp_link_t l;
    capture_t c;
    uint8_t buf[8];
    raw_link(&l, &c, buf, sizeof(buf));
    fill(30, 8);
    const uint32_t t0 = UINT32_MAX - 100000; /* 0.1 s before wrap */
    isotp_send(&l, msg, 30, t0);
    isotp_poll(&l, t0 + 500000);              /* wrapped, 0.5 s elapsed */
    CHECK_EQ(isotp_tx_status(&l), ISOTP_BUSY);
    isotp_poll(&l, t0 + 1000001);
    CHECK_EQ(isotp_tx_status(&l), ISOTP_ERR_TIMEOUT_BS);
}

static void fuzz_random_frames(void)
{
    /* Random garbage must never crash, overrun rx_buf or hang. Runs under
     * ASan/UBSan in CI. */
    isotp_link_t l;
    capture_t c;
    static uint8_t buf[100];
    raw_link(&l, &c, buf, sizeof(buf));
    srand(9);
    uint8_t f[8];
    for (int i = 0; i < 200000; i++) {
        const uint8_t len = (uint8_t)(rand() % 9);
        for (int k = 0; k < 8; k++) f[k] = (uint8_t)rand();
        if (len) f[0] = (uint8_t)((f[0] & 0x3F) | ((rand() % 4) << 4)); /* bias towards valid PCIs */
        isotp_on_frame(&l, f, len, (uint32_t)i * 100u);
        isotp_poll(&l, (uint32_t)i * 100u);
        if (c.n > 60) c.n = 0;
        if (rand() % 50 == 0) isotp_rx_take(&l);
    }
    CHECK(1);
}

int main(void)
{
    RUN(stmin_encoding);
    RUN(single_frames);
    RUN(multi_frame_sizes);
    RUN(block_size_and_stmin_matrix);
    RUN(sequence_number_wraps);
    RUN(stmin_is_respected);
    RUN(receiver_overflow);
    RUN(lost_flow_control_times_out);
    RUN(lost_consecutive_frame_is_detected);
    RUN(lost_last_frame_times_out);
    RUN(full_duplex);
    RUN(flow_control_wait);
    RUN(mailbox_full_is_retried);
    RUN(invalid_frames_are_ignored);
    RUN(untaken_message_is_not_overwritten);
    RUN(timer_wraparound);
    RUN(fuzz_random_frames);
    return CHECK_DONE();
}
