/*
 * throughput.c - what block size and STmin cost you.
 *
 * Sends a 4095-byte message tester -> ECU over a simulated 500 kbit/s bus
 * for a grid of receiver settings and prints transfer time, goodput and
 * the bus load the transfer causes (what is left for other traffic), then
 * the same message over CAN FD with different TX_DL and data bit rates.
 */
#include <stdio.h>
#include <string.h>

#include "isotp.h"
#include "vbus.h"

static uint8_t msg[ISOTP_MAX_LEN], rx_t[ISOTP_MAX_LEN], rx_e[ISOTP_MAX_LEN];

/* data_bitrate 0 = classic CAN bus. */
static void measure_dl(uint8_t bs, uint8_t st, int pad, uint8_t tx_dl, uint32_t data_bitrate, double *ms,
                       double *kbps, double *load)
{
    vbus_t bus;
    isotp_link_t tester, ecu;
    vbus_init_fd(&bus, 500000, data_bitrate);
    isotp_config_t a = { .tx_id = 0x7E0, .rx_buf = rx_t, .rx_cap = sizeof(rx_t), .tx_dl = tx_dl,
                         .padding = pad ? 0xCC : ISOTP_NO_PADDING };
    isotp_config_t b = { .tx_id = 0x7E8, .rx_buf = rx_e, .rx_cap = sizeof(rx_e), .block_size = bs,
                         .st_min = st, .padding = pad ? 0xCC : ISOTP_NO_PADDING };
    vbus_node_t *na = vbus_attach(&bus, &a, 0x7E8), *nb = vbus_attach(&bus, &b, 0x7E0);
    isotp_init(&tester, &a);
    isotp_init(&ecu, &b);
    vbus_bind(na, &tester);
    vbus_bind(nb, &ecu);

    const uint32_t t0 = bus.now_us;
    isotp_send(&tester, msg, sizeof(msg), t0);
    while (isotp_rx_status(&ecu) != ISOTP_DONE && bus.now_us - t0 < 60000000u)
        vbus_step(&bus);
    const double dt = (double)(bus.now_us - t0);

    *ms = dt / 1000.0;
    *kbps = sizeof(msg) * 8.0 / dt * 1000.0;
    *load = 100.0 * (double)bus.busy_us / dt; /* bus load caused by the transfer */
}

static void measure(uint8_t bs, uint8_t st, int pad, double *ms, double *kbps, double *load)
{
    measure_dl(bs, st, pad, 8, 0, ms, kbps, load);
}

int main(void)
{
    for (size_t i = 0; i < sizeof(msg); i++) msg[i] = (uint8_t)i;

    const struct { uint8_t raw; const char *name; } st[] = {
        { 0x00, "0" }, { 0xF5, "0.5 ms" }, { 0x01, "1 ms" }, { 0x05, "5 ms" },
    };
    const uint8_t bs[] = { 0, 8, 1 };

    printf("4095-byte message, classic CAN 500 kbit/s, frames padded to 8 bytes.\n\n");
    printf("| Receiver BS | Receiver STmin | Transfer time | Goodput | Bus load |\n");
    printf("|---:|---:|---:|---:|---:|\n");
    for (size_t i = 0; i < sizeof(bs); i++)
        for (size_t j = 0; j < sizeof(st) / sizeof(st[0]); j++) {
            double ms, kbps, load;
            measure(bs[i], st[j].raw, 1, &ms, &kbps, &load);
            printf("| %u | %s | %.1f ms | %.1f kbit/s | %.1f %% |\n", bs[i], st[j].name, ms, kbps, load);
        }

    double ms0, k0, e0, ms1, k1, e1;
    measure(0, 0, 1, &ms0, &k0, &e0);
    measure(0, 0, 0, &ms1, &k1, &e1);
    printf("\nPadding off saves only the final short frame: %.2f ms -> %.2f ms.\n", ms0, ms1);

    const struct { uint8_t dl; uint32_t dbr; const char *bus; } fd[] = {
        { 8, 0, "classic 500k" },  { 8, 2000000, "FD 500k/2M" },  { 16, 2000000, "FD 500k/2M" },
        { 64, 2000000, "FD 500k/2M" }, { 64, 5000000, "FD 500k/5M" },
    };
    printf("\n4095-byte message, receiver BS 0, STmin 0.\n\n");
    printf("| Bus | TX_DL | Frames | Transfer time | Goodput | Bus load |\n");
    printf("|---|---:|---:|---:|---:|---:|\n");
    for (size_t i = 0; i < sizeof(fd) / sizeof(fd[0]); i++) {
        double ms, kbps, load;
        measure_dl(0, 0, 1, fd[i].dl, fd[i].dbr, &ms, &kbps, &load);
        const unsigned ff = fd[i].dl - 2u, cf = fd[i].dl - 1u;
        const unsigned frames = 2u + (sizeof(msg) - ff + cf - 1u) / cf; /* FF + FC + CFs */
        printf("| %s | %u | %u | %.1f ms | %.1f kbit/s | %.1f %% |\n", fd[i].bus, fd[i].dl, frames, ms, kbps,
               load);
    }
    return 0;
}
