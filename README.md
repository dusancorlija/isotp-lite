# isotp-lite

**ISO 15765-2 (ISO-TP) transport layer for classic CAN and CAN FD, with a bus simulator to prove it.**

ISO-TP is the protocol under UDS diagnostics and ECU flashing. It splits messages of up to 4095 bytes into CAN frames of 8 bytes (classic CAN) or up to 64 bytes (CAN FD), with flow control between sender and receiver. This implementation is about 350 lines of C99. It has no heap, no globals and no OS dependency. On a Cortex-M4 it compiles to **1.5 KB of code and 84 bytes of RAM per link** (`-Os`, measured in CI).

[![ci](https://github.com/dusancorlija/isotp-lite/actions/workflows/ci.yml/badge.svg)](https://github.com/dusancorlija/isotp-lite/actions/workflows/ci.yml)

![Timeline of a 100-byte transfer with block size 4 and STmin 1 ms](docs/timeline.svg)

## Features

- Single / first / consecutive / flow-control frames, messages of 1 to 4095 bytes
- CAN FD per ISO 15765-2:2016: configurable TX_DL (8 … 64), single frames up to 62 bytes with the SF_DL escape, RX_DL taken from each first frame, mandatory padding to valid CAN FD lengths
- Block size and STmin, in both the millisecond and the 100 µs ranges; reserved values handled per the standard
- STmin honoured across block boundaries, not just inside a block
- FC.WAIT with a configurable limit, FC.OVFLW, N_Bs and N_Cr timeouts
- Sequence-number checking with wrap-around, optional frame padding
- Full duplex: one link sends and receives at the same time
- A full TX mailbox is not an error: the frame is retried on the next poll
- Wrap-safe 32-bit microsecond clock (it wraps every 71.6 minutes)

## The simulator

`sim/vbus.c` is a discrete-time model of a classic CAN bus. It is what makes the timing behaviour testable on a laptop:

- per-node TX mailboxes of limited depth,
- bitwise arbitration (the lowest ID wins when the bus goes idle),
- frame duration from the bit rate, with worst-case bit stuffing, for classic CAN and for CAN FD with bit-rate switching,
- a fault-injection hook that can drop any frame on the wire,
- a candump-compatible trace (`tools/timeline.py` turns it into the chart above).

## What block size and STmin cost

4095-byte message at 500 kbit/s (`examples/throughput.c`):

| Receiver BS | Receiver STmin | Transfer time | Goodput | Bus load |
|---:|---:|---:|---:|---:|
| 0 | 0 | 164.4 ms | 199.3 kbit/s | 98.6 % |
| 0 | 0.5 ms | 292.9 ms | 111.9 kbit/s | 55.3 % |
| 0 | 1 ms | 584.9 ms | 56.0 kbit/s | 27.7 % |
| 0 | 5 ms | 2920.9 ms | 11.2 kbit/s | 5.5 % |
| 8 | 0 | 184.8 ms | 177.3 kbit/s | 98.6 % |
| 8 | 1 ms | 584.9 ms | 56.0 kbit/s | 31.1 % |
| 1 | 0 | 327.9 ms | 99.9 kbit/s | 98.6 % |
| 1 | 5 ms | 2920.9 ms | 11.2 kbit/s | 11.1 % |

How to read it:

- With STmin = 0 and no block limit, a single transfer saturates the bus. Every other ECU's periodic traffic now competes with the download.
- STmin is the knob that trades speed for bus capacity. Block size mainly protects a slow receiver's buffer; once STmin dominates, BS costs almost nothing.
- BS = 1 halves throughput, because every consecutive frame waits for a flow-control round trip.
- Padding to 8 bytes only lengthens the final frame (0.2 ms per message here).

## CAN FD

Set `tx_dl` in the config to send CAN FD frames; leave it 0 (or 8) for classic CAN. Receiving needs no setting: the receiver takes RX_DL from each first frame, so one link talks to classic and FD peers alike.

```c
isotp_config_t cfg = { /* ... */ .tx_dl = 64 };
```

What changes on the wire (normal addressing):

| Frame | Classic CAN (TX_DL 8) | CAN FD (TX_DL 12 … 64) |
|---|---|---|
| Single frame, 1–7 bytes | `0L` + data | same, CAN_DL ≤ 8 |
| Single frame, 8 … TX_DL−2 bytes | — | `00 LL` + data, padded to the next CAN FD length |
| First frame | `1L LL` + 6 bytes | `1L LL` + TX_DL−2 bytes |
| Consecutive frame | `2N` + 7 bytes | `2N` + TX_DL−1 bytes, all but the last exactly TX_DL long |
| Flow control | `3S BS ST` | unchanged |

The `send` callback then gets lengths of 12, 16, 20, 24, 32, 48 or 64 and must send those as CAN FD frames. Frames longer than 8 bytes are always padded (with `padding`, or 0xCC if padding is off). The receiver ignores frames that break the 2016 rules: invalid CAN FD lengths, an escaped SF_DL that would have fitted a classic single frame, a first frame shorter than FF_DLmin, and consecutive frames longer than RX_DL or not using all of it.

A classic-only build can define `ISOTP_MAX_DL=8`: frame buffers on the stack shrink from 64 to 8 bytes and FD frames are ignored.

What CAN FD buys for a 4095-byte message (`examples/throughput.c`, receiver BS 0 and STmin 0):

| Bus | TX_DL | Frames | Transfer time | Goodput |
|---|---:|---:|---:|---:|
| classic 500 kbit/s | 8 | 587 | 164.4 ms | 199 kbit/s |
| FD 500 kbit/s / 2 Mbit/s | 8 | 587 | 73.4 ms | 446 kbit/s |
| FD 500 kbit/s / 2 Mbit/s | 16 | 275 | 45.3 ms | 723 kbit/s |
| FD 500 kbit/s / 2 Mbit/s | 64 | 67 | 26.9 ms | 1218 kbit/s |
| FD 500 kbit/s / 5 Mbit/s | 64 | 67 | 13.5 ms | 2423 kbit/s |

The data-phase bit rate helps with any frame size, but the arbitration phase and the frame trailer still run at 500 kbit/s. 64-byte frames spread that fixed cost over eight times more payload, which is where most of the remaining gain comes from.

## Firmware download over UDS

`examples/uds_download.c` puts a small UDS server (ISO 14229) on the simulated ECU and drives it from a tester:

```
  -> service 0x22, 3 bytes
(   0.000285) vcan0 7E0#0322F190CCCCCCCC
(   0.000570) vcan0 7E8#101062F190565349
(   0.000850) vcan0 7E0#300000CCCCCCCCCC
(   0.001130) vcan0 7E8#214D303030303030
(   0.001410) vcan0 7E8#22303030CCCCCCCC
  <- 62 F1 90 56 53 49 4D 30   VIN prefix "VSIM000000000"
  -> service 0x34, 11 bytes
  ...
  <- 7F 34 7F   negative response, NRC 0x7F    (download refused in default session)
  -> service 0x10, 2 bytes                       (switch to programming session)
  ...
Downloaded 16384 bytes in 0.701 s (187.0 kbit/s goodput), 2518 CAN frames.
CRC-32 tester 0xAF1F4A91, ECU 0xAF1F4A91: match
```

The ECU checks the session, the block sequence counter (with 0xFF → 0x00 wrap) and the image length, and returns the matching negative response codes.

## Usage

```c
#include "isotp.h"

static uint8_t rx_buf[ISOTP_MAX_LEN];
static isotp_link_t diag;

static int can_tx(void *user, uint32_t id, const uint8_t *data, uint8_t len)
{
    return hal_can_send(id, data, len) == HAL_OK ? 0 : -1; /* -1 = mailbox full, retried later */
}

void diag_init(void)
{
    isotp_config_t cfg = {
        .tx_id = 0x7E8, .rx_id = 0x7E0,
        .send = can_tx,
        .rx_buf = rx_buf, .rx_cap = sizeof(rx_buf),
        .block_size = 8, .st_min = 0x00, .padding = 0xCC,
    };
    isotp_init(&diag, &cfg);
}

void can_rx_isr(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (id == 0x7E0)
        isotp_on_frame(&diag, data, len, micros());
}

void every_1ms(void)
{
    isotp_poll(&diag, micros());
    const int n = isotp_rx_take(&diag);
    if (n > 0)
        uds_handle(rx_buf, (uint16_t)n);
}
```

If `isotp_on_frame` runs in an interrupt and `isotp_poll` in a task, guard them with the same lock or feed received frames through a queue. A link is not reentrant.

## Tests

`tests/test_isotp.c` covers, among others:

- every single-frame length and multi-frame boundary sizes (8, 13, 14, 15, 111–113, 4094, 4095)
- a block-size × STmin matrix, and measured on-wire gaps ≥ STmin, also across block boundaries
- receiver overflow → FC.OVFLW → sender aborts
- a lost flow control (N_Bs timeout), a lost consecutive frame (sequence error), a lost last frame (N_Cr timeout)
- FC.WAIT handling and the WAIT limit, a full TX mailbox, invalid and stray frames, clock wrap-around
- simultaneous transfers in both directions
- 200 000 random frames under ASan + UBSan
- CAN FD: every single-frame length for each TX_DL, multi-frame boundaries per TX_DL, exact frame layout and padding, classic and FD peers in both directions at once, FD overflow, and frames that break the 2016 length rules
- 200 000 random classic and FD frames under ASan + UBSan

CI builds with GCC and Clang (`-Wall -Wextra -Wpedantic -Wconversion -Werror`) under sanitizers. It also cross-compiles for Cortex-M0+, M4 and M7:

| Target | Code (`-Os`) | RAM per link |
|---|---:|---:|
| Cortex-M0+ | 1558 B | 84 B |
| Cortex-M4 | 1460 B | 84 B |
| Cortex-M7 | 1458 B | 84 B |

With `-DISOTP_MAX_DL=8` (classic CAN only) the code is about 120 bytes smaller.

## Build

```bash
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure
./build/throughput
./build/uds_download
./build/trace 100 4 0x01 > trace.log && python3 tools/timeline.py trace.log timeline.svg
```

## Scope

Supported: normal addressing, classic CAN and CAN FD, messages up to 4095 bytes. Not supported: extended / mixed addressing, and messages over 4095 bytes. A first frame with the 32-bit length escape is answered with FC.OVFLW when it announces more than 4095 bytes, and ignored otherwise, as ISO 15765-2:2016 requires.

## License

MIT
