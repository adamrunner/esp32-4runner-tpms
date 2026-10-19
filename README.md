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
`i<id> <psi>` inject a fake reading, `t<n>` carrier-sense threshold (-7..7), `g` toggle CS
gating, `r` toggle raw row dump, `f<hz>` set frequency, `x[n]` bench TX test, `h` help.

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

## Host test

```fish
cd test && cc -I../main -o /tmp/host_test host_test.c ../main/tpms_decode.c && /tmp/host_test
```

## rtl_433 capture (ground truth)

```fish
cd captures
rtl_433 -f 315M -M level -M time:iso -F kv -F json:tpms.jsonl -S unknown
```
