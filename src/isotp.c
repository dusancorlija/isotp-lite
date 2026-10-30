#include "isotp.h"

#include <string.h>

enum { PCI_SF = 0x0, PCI_FF = 0x1, PCI_CF = 0x2, PCI_FC = 0x3 };
enum { FS_CTS = 0, FS_WAIT = 1, FS_OVFLW = 2 };
enum { TX_IDLE, TX_WAIT_FC, TX_SEND_CF };
enum { RX_IDLE, RX_RECEIVING, RX_DONE };

/* Wrap-safe "now has reached t" for a free-running 32-bit microsecond clock. */
static bool reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

uint32_t isotp_stmin_to_us(uint8_t raw)
{
    if (raw <= 0x7F) return (uint32_t)raw * 1000u;
    if (raw >= 0xF1 && raw <= 0xF9) return (uint32_t)(raw - 0xF0) * 100u;
    return 127000u;
}

static int emit(isotp_link_t *l, uint8_t *f, uint8_t used)
{
    uint8_t len = used;
    if (l->cfg.padding >= 0) {
        memset(f + used, (uint8_t)l->cfg.padding, (size_t)(8 - used));
        len = 8;
    }
    return l->cfg.send(l->cfg.user, l->cfg.tx_id, f, len);
}

static int send_fc(isotp_link_t *l, uint8_t fs)
{
    uint8_t f[8] = { (uint8_t)(PCI_FC << 4 | fs), l->cfg.block_size, l->cfg.st_min };
    return emit(l, f, 3);
}

void isotp_init(isotp_link_t *l, const isotp_config_t *cfg)
{
    memset(l, 0, sizeof(*l));
    l->cfg = *cfg;
    if (!l->cfg.n_bs_ms) l->cfg.n_bs_ms = 1000;
    if (!l->cfg.n_cr_ms) l->cfg.n_cr_ms = 1000;
    if (!l->cfg.wft_max) l->cfg.wft_max = 8;
}

/* ── transmitter ─────────────────────────────────────────────────────────── */

static void tx_pump(isotp_link_t *l, uint32_t now)
{
    while (l->tx_state == TX_SEND_CF && reached(now, l->tx_due_us)) {
        uint8_t f[8];
        const uint16_t left = (uint16_t)(l->tx_len - l->tx_off);
        const uint8_t n = left < 7 ? (uint8_t)left : 7;
        f[0] = (uint8_t)(PCI_CF << 4 | l->tx_sn);
        memcpy(&f[1], l->tx_data + l->tx_off, n);
        if (emit(l, f, (uint8_t)(1 + n)) != 0)
            return; /* mailbox full: retry on next poll */

        l->tx_off = (uint16_t)(l->tx_off + n);
        l->tx_sn = (uint8_t)((l->tx_sn + 1) & 0x0F);
        l->tx_last_cf_us = now;
        if (l->tx_off >= l->tx_len) {
            l->tx_state = TX_IDLE;
            l->tx_status = ISOTP_DONE;
            return;
        }
        if (l->tx_bs && --l->tx_bs_left == 0) {
            l->tx_state = TX_WAIT_FC;
            l->tx_deadline_us = now + l->cfg.n_bs_ms * 1000u;
            return;
        }
        l->tx_due_us = now + l->tx_st_us;
        if (l->tx_st_us)
            return; /* honour STmin: next CF on a later poll */
    }
}

isotp_status_t isotp_send(isotp_link_t *l, const uint8_t *data, uint16_t len, uint32_t now_us)
{
    if (l->tx_state != TX_IDLE)
        return ISOTP_BUSY;
    if (len == 0 || len > ISOTP_MAX_LEN || (!data && len))
        return ISOTP_ERR_INVALID;

    uint8_t f[8];
    if (len <= 7) {
        f[0] = (uint8_t)(PCI_SF << 4 | len);
        memcpy(&f[1], data, len);
        if (emit(l, f, (uint8_t)(1 + len)) != 0)
            return ISOTP_BUSY; /* caller may retry */
        return l->tx_status = ISOTP_DONE;
    }

    f[0] = (uint8_t)(PCI_FF << 4 | (len >> 8));
    f[1] = (uint8_t)len;
    memcpy(&f[2], data, 6);
    if (emit(l, f, 8) != 0)
        return ISOTP_BUSY;

    l->tx_data = data;
    l->tx_len = len;
    l->tx_off = 6;
    l->tx_sn = 1;
    l->tx_wft = 0;
    l->tx_state = TX_WAIT_FC;
    l->tx_deadline_us = now_us + l->cfg.n_bs_ms * 1000u;
    return l->tx_status = ISOTP_BUSY;
}

static void on_fc(isotp_link_t *l, const uint8_t *d, uint8_t len, uint32_t now)
{
    if (l->tx_state != TX_WAIT_FC || len < 3)
        return;
    switch (d[0] & 0x0F) {
    case FS_CTS:
        l->tx_bs = d[1];
        l->tx_bs_left = d[1];
        l->tx_st_us = isotp_stmin_to_us(d[2]);
        l->tx_wft = 0;
        l->tx_state = TX_SEND_CF;
        /* STmin separates consecutive frames, also across a block
         * boundary; the very first CF after the FF may go immediately. */
        l->tx_due_us = (l->tx_sn == 1 && l->tx_off == 6) ? now : l->tx_last_cf_us + l->tx_st_us;
        if (reached(now, l->tx_due_us))
            l->tx_due_us = now;
        tx_pump(l, now);
        break;
    case FS_WAIT:
        if (++l->tx_wft > l->cfg.wft_max) {
            l->tx_state = TX_IDLE;
            l->tx_status = ISOTP_ERR_WFT_OVRN;
        } else {
            l->tx_deadline_us = now + l->cfg.n_bs_ms * 1000u;
        }
        break;
    case FS_OVFLW:
        l->tx_state = TX_IDLE;
        l->tx_status = ISOTP_ERR_OVERFLOW;
        break;
    default:
        break; /* reserved flow status: ignore */
    }
}

/* ── receiver ────────────────────────────────────────────────────────────── */

static void rx_fail(isotp_link_t *l, isotp_status_t why)
{
    l->rx_state = RX_IDLE;
    l->rx_status = why;
}

static void rx_flow(isotp_link_t *l, uint8_t fs)
{
    l->rx_fc_pending = send_fc(l, fs) ? (uint8_t)(fs + 1) : 0;
}

static void on_sf(isotp_link_t *l, const uint8_t *d, uint8_t len)
{
    const uint8_t n = d[0] & 0x0F;
    if (n == 0 || n > 7 || len < 1 + n)
        return;
    if (n > l->cfg.rx_cap) {
        rx_fail(l, ISOTP_ERR_OVERFLOW);
        return;
    }
    memcpy(l->cfg.rx_buf, &d[1], n);
    l->rx_len = n;
    l->rx_state = RX_DONE;
    l->rx_status = ISOTP_DONE;
}

static void on_ff(isotp_link_t *l, const uint8_t *d, uint8_t len, uint32_t now)
{
    if (len < 8)
        return;
    const uint16_t total = (uint16_t)((d[0] & 0x0F) << 8 | d[1]);
    if (total != 0 && total < 8)
        return; /* would have fitted a single frame: invalid */
    if (total == 0 || total > l->cfg.rx_cap) {
        /* total == 0 is the 32-bit length escape, not supported here */
        rx_flow(l, FS_OVFLW);
        rx_fail(l, ISOTP_ERR_OVERFLOW);
        return;
    }
    memcpy(l->cfg.rx_buf, &d[2], 6);
    l->rx_len = total;
    l->rx_off = 6;
    l->rx_sn = 1;
    l->rx_bs_count = 0;
    l->rx_state = RX_RECEIVING;
    l->rx_status = ISOTP_BUSY;
    l->rx_deadline_us = now + l->cfg.n_cr_ms * 1000u;
    rx_flow(l, FS_CTS);
}

static void on_cf(isotp_link_t *l, const uint8_t *d, uint8_t len, uint32_t now)
{
    if (l->rx_state != RX_RECEIVING)
        return;
    if ((d[0] & 0x0F) != l->rx_sn) {
        rx_fail(l, ISOTP_ERR_WRONG_SN);
        return;
    }

    const uint16_t left = (uint16_t)(l->rx_len - l->rx_off);
    const uint8_t n = left < 7 ? (uint8_t)left : 7;
    if (len < 1 + n)
        return; /* truncated frame: ignore, N_Cr will catch a real loss */

    memcpy(l->cfg.rx_buf + l->rx_off, &d[1], n);
    l->rx_off = (uint16_t)(l->rx_off + n);
    l->rx_sn = (uint8_t)((l->rx_sn + 1) & 0x0F);
    l->rx_deadline_us = now + l->cfg.n_cr_ms * 1000u;

    if (l->rx_off >= l->rx_len) {
        l->rx_state = RX_DONE;
        l->rx_status = ISOTP_DONE;
    } else if (l->cfg.block_size && ++l->rx_bs_count == l->cfg.block_size) {
        l->rx_bs_count = 0;
        rx_flow(l, FS_CTS);
    }
}

void isotp_on_frame(isotp_link_t *l, const uint8_t *data, uint8_t len, uint32_t now_us)
{
    if (len == 0 || len > 8)
        return;
    const uint8_t pci = data[0] >> 4;

    if (pci == PCI_FC) {
        on_fc(l, data, len, now_us);
        return;
    }
    /* An untaken message is never overwritten: the application reads
     * rx_buf after isotp_rx_take() without copying it. */
    if (l->rx_state == RX_DONE)
        return;

    switch (pci) {
    case PCI_SF: on_sf(l, data, len); break;          /* also aborts a reception in progress */
    case PCI_FF: on_ff(l, data, len, now_us); break;  /* likewise restarts reception */
    case PCI_CF: on_cf(l, data, len, now_us); break;
    default: break;
    }
}

void isotp_poll(isotp_link_t *l, uint32_t now_us)
{
    if (l->tx_state == TX_WAIT_FC && reached(now_us, l->tx_deadline_us)) {
        l->tx_state = TX_IDLE;
        l->tx_status = ISOTP_ERR_TIMEOUT_BS;
    }
    tx_pump(l, now_us);

    if (l->rx_fc_pending)
        rx_flow(l, (uint8_t)(l->rx_fc_pending - 1));
    if (l->rx_state == RX_RECEIVING && reached(now_us, l->rx_deadline_us))
        rx_fail(l, ISOTP_ERR_TIMEOUT_CR);
}

isotp_status_t isotp_tx_status(const isotp_link_t *l) { return l->tx_status; }
isotp_status_t isotp_rx_status(const isotp_link_t *l) { return l->rx_status; }

int isotp_rx_take(isotp_link_t *l)
{
    if (l->rx_state == RX_DONE) {
        l->rx_state = RX_IDLE;
        l->rx_status = ISOTP_IDLE;
        return l->rx_len;
    }
    if (l->rx_state == RX_IDLE && l->rx_status != ISOTP_IDLE)
        l->rx_status = ISOTP_IDLE; /* clear a reported error */
    return -1;
}

const char *isotp_status_str(isotp_status_t s)
{
    switch (s) {
    case ISOTP_IDLE: return "idle";
    case ISOTP_BUSY: return "busy";
    case ISOTP_DONE: return "done";
    case ISOTP_ERR_TIMEOUT_BS: return "timeout N_Bs";
    case ISOTP_ERR_TIMEOUT_CR: return "timeout N_Cr";
    case ISOTP_ERR_WRONG_SN: return "wrong sequence number";
    case ISOTP_ERR_OVERFLOW: return "buffer overflow";
    case ISOTP_ERR_WFT_OVRN: return "too many FC.WAIT";
    case ISOTP_ERR_INVALID: return "invalid argument";
    }
    return "?";
}
