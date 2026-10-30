/*
 * throughput.c - what block size and STmin cost you.
 *
 * Sends a 4095-byte message tester -> ECU over a simulated 500 kbit/s bus
 * for a grid of receiver settings and prints transfer time, goodput and
 * the bus load the transfer causes (what is left for other traffic).
 */
#include <stdio.h>
#include <string.h>

#include "isotp.h"
#include "vbus.h"

static uint8_t msg[ISOTP_MAX_LEN], rx_t[ISOTP_MAX_LEN], rx_e[ISOTP_MAX_LEN];

static void measure(uint8_t bs, uint8_t st, int pad, double *ms, double *kbps, double *load)
{
    vbus_t bus;
    isotp_link_t tester, ecu;
    vbus_init(&bus, 500000);
    isotp_config_t a = { .tx_id = 0x7E0, .rx_buf = rx_t, .rx_cap = sizeof(rx_t),
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
    return 0;
}
