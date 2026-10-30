/*
 * trace.c - writes a candump-style trace of one transfer.
 *
 *   ./trace <bytes> <receiver BS> <receiver STmin raw> > trace.log
 *   python3 tools/timeline.py trace.log docs/timeline.svg
 */
#include <stdio.h>
#include <stdlib.h>

#include "isotp.h"
#include "vbus.h"

int main(int argc, char **argv)
{
    const uint16_t len = (uint16_t)(argc > 1 ? atoi(argv[1]) : 100);
    const uint8_t bs = (uint8_t)(argc > 2 ? atoi(argv[2]) : 4);
    const uint8_t st = (uint8_t)(argc > 3 ? strtol(argv[3], NULL, 0) : 1);

    static uint8_t msg[ISOTP_MAX_LEN], rt[ISOTP_MAX_LEN], re[ISOTP_MAX_LEN];
    for (uint16_t i = 0; i < len; i++) msg[i] = (uint8_t)i;

    vbus_t bus;
    isotp_link_t tester, ecu;
    vbus_init(&bus, 500000);
    isotp_config_t a = { .tx_id = 0x7E0, .rx_buf = rt, .rx_cap = sizeof(rt), .padding = 0xCC };
    isotp_config_t b = { .tx_id = 0x7E8, .rx_buf = re, .rx_cap = sizeof(re), .padding = 0xCC,
                         .block_size = bs, .st_min = st };
    vbus_node_t *na = vbus_attach(&bus, &a, 0x7E8), *nb = vbus_attach(&bus, &b, 0x7E0);
    isotp_init(&tester, &a);
    isotp_init(&ecu, &b);
    vbus_bind(na, &tester);
    vbus_bind(nb, &ecu);
    bus.trace = stdout;

    isotp_send(&tester, msg, len, bus.now_us);
    while (isotp_rx_status(&ecu) != ISOTP_DONE && bus.now_us < 10000000u)
        vbus_step(&bus);
    return isotp_rx_take(&ecu) == len ? 0 : 1;
}
