/*
 * uds_download.c - a firmware download over UDS (ISO 14229) on top of
 * isotp-lite, tester and ECU on one simulated CAN bus.
 *
 *   10 02            DiagnosticSessionControl: programming session
 *   22 F1 90         ReadDataByIdentifier: VIN
 *   34 00 44 ...     RequestDownload (address, size) -> max block length
 *   36 nn <data>     TransferData, block sequence counter nn
 *   37               RequestTransferExit -> ECU returns CRC-32 of the image
 *
 * The ECU checks the session, the block sequence counter and the image
 * length, and answers with negative responses (7F) where UDS requires it.
 * The run prints the first frames as a candump-style trace and a summary.
 */
#include <stdio.h>
#include <string.h>

#include "isotp.h"
#include "vbus.h"

#define TESTER 0x7E0
#define ECU 0x7E8
#define IMAGE_SIZE 16384u
#define FLASH_BASE 0x08010000u

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }

/* ── ECU ─────────────────────────────────────────────────────────────────── */

static struct {
    uint8_t session;            /* 1 default, 2 programming */
    uint8_t flash[IMAGE_SIZE];
    uint32_t dl_size, dl_off;
    uint8_t bsc;                /* next expected block sequence counter */
    int downloading;
    uint8_t resp[16];
} ecu;

static uint16_t nrc(uint8_t sid, uint8_t code)
{
    ecu.resp[0] = 0x7F; ecu.resp[1] = sid; ecu.resp[2] = code;
    return 3;
}

static uint16_t ecu_handle(const uint8_t *rq, uint16_t n)
{
    uint8_t *r = ecu.resp;
    switch (rq[0]) {
    case 0x10:
        if (n != 2) return nrc(0x10, 0x13);
        ecu.session = rq[1];
        r[0] = 0x50; r[1] = rq[1];
        r[2] = 0x00; r[3] = 50;            /* P2server = 50 ms */
        r[4] = 0x01; r[5] = 0xF4;          /* P2*server = 500 * 10 ms */
        return 6;
    case 0x22:
        if (n != 3 || rq[1] != 0xF1 || rq[2] != 0x90) return nrc(0x22, 0x31);
        r[0] = 0x62; r[1] = 0xF1; r[2] = 0x90;
        memcpy(&r[3], "VSIM0000000000042", 13); /* 13 of 17 chars fit in resp[] */
        return 16;
    case 0x34: {
        if (ecu.session != 2) return nrc(0x34, 0x7F);         /* not in this session */
        if (n != 11 || rq[2] != 0x44) return nrc(0x34, 0x13);
        const uint32_t addr = be32(&rq[3]), size = be32(&rq[7]);
        if (addr != FLASH_BASE || size > IMAGE_SIZE) return nrc(0x34, 0x31);
        ecu.dl_size = size; ecu.dl_off = 0; ecu.bsc = 1; ecu.downloading = 1;
        r[0] = 0x74; r[1] = 0x20; r[2] = 0x0F; r[3] = 0xFF;  /* maxNumberOfBlockLength = 4095 */
        return 4;
    }
    case 0x36:
        if (!ecu.downloading) return nrc(0x36, 0x24);
        if (n < 2 || rq[1] != ecu.bsc) return nrc(0x36, 0x73);
        if (ecu.dl_off + (n - 2u) > ecu.dl_size) return nrc(0x36, 0x71);
        memcpy(&ecu.flash[ecu.dl_off], &rq[2], n - 2u);
        ecu.dl_off += n - 2u;
        r[0] = 0x76; r[1] = ecu.bsc++;     /* wraps 0xFF -> 0x00 by design */
        return 2;
    case 0x37:
        if (!ecu.downloading) return nrc(0x37, 0x24);
        if (ecu.dl_off != ecu.dl_size) return nrc(0x37, 0x70);
        ecu.downloading = 0;
        r[0] = 0x77;
        put32(&r[1], crc32(ecu.flash, ecu.dl_size));
        return 5;
    default:
        return nrc(rq[0], 0x11);
    }
}

/* ── tester ──────────────────────────────────────────────────────────────── */

static uint8_t image[IMAGE_SIZE];
static uint8_t req[ISOTP_MAX_LEN];

static uint16_t next_request(int step, uint32_t *off, uint8_t *bsc)
{
    switch (step) {
    case 0: req[0] = 0x22; req[1] = 0xF1; req[2] = 0x90; return 3;   /* VIN */
    case 1: req[0] = 0x34; req[1] = 0x00; req[2] = 0x44;             /* refused: default session */
            put32(&req[3], FLASH_BASE); put32(&req[7], IMAGE_SIZE); return 11;
    case 2: req[0] = 0x10; req[1] = 0x02; return 2;
    case 3: req[0] = 0x34; req[1] = 0x00; req[2] = 0x44;
            put32(&req[3], FLASH_BASE); put32(&req[7], IMAGE_SIZE); return 11;
    default:
        if (*off < IMAGE_SIZE) {
            const uint32_t chunk = IMAGE_SIZE - *off < ISOTP_MAX_LEN - 2 ? IMAGE_SIZE - *off : ISOTP_MAX_LEN - 2;
            req[0] = 0x36; req[1] = (*bsc)++;
            memcpy(&req[2], &image[*off], chunk);
            *off += chunk;
            return (uint16_t)(chunk + 2);
        }
        req[0] = 0x37;
        return 1;
    }
}

static void describe(const uint8_t *r, int n)
{
    printf("  <- ");
    for (int i = 0; i < n && i < 8; i++) printf("%02X ", r[i]);
    if (r[0] == 0x7F) printf("  negative response, NRC 0x%02X", r[2]);
    else if (r[0] == 0x62) printf("  VIN prefix \"%.13s\"", (const char *)&r[3]);
    else if (r[0] == 0x74) printf("  max block length %u", (unsigned)(r[2] << 8 | r[3]));
    printf("\n");
}

int main(void)
{
    for (uint32_t i = 0; i < IMAGE_SIZE; i++) image[i] = (uint8_t)(i * 2654435761u >> 24);
    ecu.session = 1;

    vbus_t bus;
    isotp_link_t tl, el;
    static uint8_t trx[64], erx[ISOTP_MAX_LEN];
    vbus_init(&bus, 500000);
    isotp_config_t tc = { .tx_id = TESTER, .rx_buf = trx, .rx_cap = sizeof(trx), .padding = 0xCC };
    isotp_config_t ec = { .tx_id = ECU, .rx_buf = erx, .rx_cap = sizeof(erx), .padding = 0xCC,
                          .block_size = 16, .st_min = 0x00 };
    vbus_node_t *tn = vbus_attach(&bus, &tc, ECU), *en = vbus_attach(&bus, &ec, TESTER);
    isotp_init(&tl, &tc);
    isotp_init(&el, &ec);
    vbus_bind(tn, &tl);
    vbus_bind(en, &el);
    bus.trace = stdout;

    int step = 0, waiting = 0, done = 0;
    uint32_t off = 0, xfer_start = 0, ecu_crc = 0;
    uint8_t bsc = 1;

    printf("Trace (first frames):\n");
    while (!done && bus.now_us < 30000000u) {
        if (!waiting) {
            if (step == 4) xfer_start = bus.now_us;
            const uint16_t n = next_request(step, &off, &bsc);
            printf("  -> service 0x%02X, %u bytes\n", req[0], n);
            isotp_send(&tl, req, n, bus.now_us);
            waiting = 1;
        }
        vbus_step(&bus);
        if (bus.frames > 40) bus.trace = NULL;

        const int rn = isotp_rx_take(&el);
        if (rn > 0) {
            const uint16_t len = ecu_handle(erx, (uint16_t)rn);
            isotp_send(&el, ecu.resp, len, bus.now_us);
        }
        const int tn_len = isotp_rx_take(&tl);
        if (tn_len > 0) {
            if (trx[0] != 0x76 || step < 5) describe(trx, tn_len);
            if (trx[0] == 0x77) {
                ecu_crc = be32(&trx[1]);
                done = 1;
            }
            waiting = 0;
            step++;
        }
    }

    const double secs = (bus.now_us - xfer_start) / 1e6;
    const uint32_t local = crc32(image, IMAGE_SIZE);
    printf("\nDownloaded %u bytes in %.3f s (%.1f kbit/s goodput), %u CAN frames.\n",
           IMAGE_SIZE, secs, IMAGE_SIZE * 8 / secs / 1000, bus.frames);
    printf("CRC-32 tester 0x%08X, ECU 0x%08X: %s\n", local, ecu_crc, local == ecu_crc ? "match" : "MISMATCH");
    return local == ecu_crc ? 0 : 1;
}
