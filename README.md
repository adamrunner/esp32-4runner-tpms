# esp32-4runner-tpms

Receive the 2018 4Runner's TPMS sensors directly at 315 MHz with a CC1101 + ESP32-S3.

The CC1101 runs in async 2-FSK RX mode: GDO0 outputs the demodulated bitstream, GDO2 outputs
carrier sense. An ISR timestamps GDO0 edges (only while carrier sense is high), and the run
lengths are sliced into bits for decoders ported from rtl_433:

| Decoder  | rtl_433 source     | Half-bit | Notes                         |
|----------|--------------------|----------|-------------------------------|
| PMV-107J | `tpms_pmv107j.c`   | 100 us   | 315 MHz Pacific, likely match |
| Toyota   | `tpms_toyota.c`    | 52 us    | PMV-C210                      |

## Wiring (ESP32-S3 DevKitC, all on the left header)

| CC1101 pin | ESP32-S3 |
|------------|----------|
| 1 GND      | GND      |
| 2 VCC      | 3V3 (not 5V) |
| 3 GDO0     | GPIO4    |
| 4 CSN      | GPIO10   |
| 5 SCK      | GPIO12   |
| 6 MOSI     | GPIO11   |
| 7 MISO/GDO1| GPIO13   |
| 8 GDO2     | GPIO5    |

## Build / flash

```fish
. ~/esp/v5.5/esp-idf/export.fish
idf.py -p /dev/cu.usbmodem2101 build flash monitor
```

Console commands (type + Enter): `s` status, `m` start/cancel mapping, `c` clear mapping,
`l<pos>` learn one wheel (e.g. `lFL`), `a<pos> <id>` assign a sensor by hand (e.g. `aFL 0a1b2c3d`),
`i<id> <psi>` inject a fake reading, `t<n>` carrier-sense threshold (-7..7), `g` toggle CS
gating, `r` toggle raw row dump, `j` toggle JSON output, `f<hz>` set frequency, `x[n]` bench TX test, `h` help.

## Output and tire mapping

The node emits JSON lines (`tire`, `tpms` snapshot every 10 s, `map` events); see
[docs/message-schema.md](docs/message-schema.md).

Mapping sensors to wheels (BOOT button = GPIO0, feedback on the onboard RGB LED):

1. Drive a few minutes so all four sensors have been heard.
2. Press BOOT. The LED blinks blue once: driver front.
3. Let ~2-3 PSI out of that tire until the LED shows a long green, then move on:
   2 blinks = driver rear, 3 = passenger rear, 4 = passenger front.
4. Three green flashes = mapping saved to flash. Re-inflate. Press BOOT mid-run to
   cancel (three red flashes); the old mapping is kept.

A dim cyan blip means a TPMS packet was decoded.

## Phone web UI (no laptop)

Power the board from any USB power bank or phone. It brings up a Wi-Fi access point
**`4runner-tpms`** (password `tpms4runner`; change both in `main/web.h`). Join it and open
**http://192.168.4.1**. The AP hands out no default gateway or DNS, so the phone keeps using cellular for internet while joined. It may say "No Internet Connection" for this network; that's expected.

- **Learn one wheel**: tap *Learn* on a wheel, then fill or let air out of that tire. The first
  sensor not already saved on another wheel whose pressure moves 1.5 PSI either way gets saved to
  flash right away. Do the wheels one at a time, in any order. Each one is saved as soon as it's
  found. Console equivalent: `lFL` (`l` alone cancels).
- **Sensors heard** lists every sensor with live PSI, its change since first heard, and when it
  was last heard. Each card has *→ FL/FR/RL/RR* buttons to save that sensor by hand.

Changing a tire's pressure makes its sensor transmit within seconds, even when parked.

**Log** (top right, or http://192.168.4.1/log) shows the raw console output: the last 32 KB
of `ESP_LOG` and JSON lines, kept in RAM. It has a filter and a command box (`s`, `d`, `r`, ...)
that runs console commands, with their output shown in the log.

## Host test

```fish
cd test && cc -I../main -o /tmp/host_test host_test.c ../main/tpms_decode.c && /tmp/host_test
```

## rtl_433 capture (ground truth)

```fish
cd captures
rtl_433 -f 315M -M level -M time:iso -F kv -F json:tpms.jsonl -S unknown
```

## License

GPL-2.0-or-later (see `LICENSE`). The TPMS decoders are ported from
[rtl_433](https://github.com/merbanan/rtl_433), which is GPL-2.0-or-later.
