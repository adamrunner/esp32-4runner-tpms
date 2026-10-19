# TPMS node message schema

The node writes one JSON object per line to its serial console (USB today; RS485,
ESP-NOW or UART later). Lines that don't start with `{` are human-readable logs
and should be ignored by consumers. Every object has a `type`.

Positions use the driver's-side convention for a US (left-hand-drive) truck:

| pos | wheel |
|-----|-------|
| `FL` | driver front |
| `FR` | passenger front |
| `RL` | driver rear |
| `RR` | passenger rear |

## Tire fields

Shared by `tire` messages and each entry of a `tpms` snapshot.

| field | type | notes |
|-------|------|-------|
| `pos` | `"FL"`/`"FR"`/`"RL"`/`"RR"` or `null` | `null` = sensor not mapped (e.g. a passing car) |
| `id` | 8 hex chars or `null` | sensor ID; `null` only for an unmapped position in a snapshot |
| `model` | string or `null` | decoder that matched: `"PMV-107J"` (this truck) or `"Toyota"` |
| `psi` | number or `null` | 0.1 PSI precision; the PMV-107J sensor resolves ~0.36 PSI |
| `kpa` | number or `null` | same reading in kPa |
| `temp_c` | number or `null` | whole °C |
| `battery_ok` | bool or `null` | `null` when the protocol doesn't report it |
| `rapid_change` | bool or `null` | sensor's fast-pressure-change flag. **Not reliable on its own**: this truck's sensors set it routinely while driving |
| `failed` | bool or `null` | sensor self-test failure |
| `age_s` | int or `null` | seconds since the last packet from this sensor; `null` = not heard since boot |
| `stale` | bool | `true` if `age_s` > 600 or never heard. Parked sensors go quiet, so stale ≠ fault |

## `tire`: a new reading

Emitted once per sensor transmission (repeats within 2 s are suppressed), for
every sensor heard, mapped or not. Sensors transmit roughly every 64 s while
driving.

```json
{"type":"tire","pos":"RL","id":"0c3d4e5f","model":"PMV-107J","psi":33.5,"kpa":230.6,"temp_c":18,"battery_ok":true,"rapid_change":false,"failed":false,"age_s":0,"stale":false}
```

## `tpms`: periodic snapshot

Every 10 s, always all four positions in `FL, FR, RL, RR` order. This is what a
display or logger should normally consume.

```json
{"type":"tpms","uptime_s":600,"mapping":false,"mapped":true,"tires":[
  {"pos":"FL","id":"0a1b2c3d","model":"PMV-107J","psi":33.5,"kpa":230.6,"temp_c":19,"battery_ok":true,"rapid_change":true,"failed":false,"age_s":12,"stale":false},
  {"pos":"FR","id":"0b2c3d4e", "...": "..."},
  {"pos":"RL","id":"0c3d4e5f", "...": "..."},
  {"pos":"RR","id":"0d4e5f60", "...": "..."}]}
```

- `mapping`: a mapping run is in progress.
- `mapped`: all four positions have a sensor ID assigned.

## `map`: mapping procedure events

| event | extra fields | meaning |
|-------|--------------|---------|
| `start` | `order`, `candidates`, `drop_psi` | mapping started; `candidates` = sensors heard in the last 15 min |
| `warning` | `msg` | fewer than 4 candidates: newly heard sensors will be accepted |
| `wait` | `step` (1-4), `pos` | let air out of this tire now |
| `assigned` | `step`, `pos`, `id`, `drop_psi` | a sensor dropped ≥ `drop_psi` and was assigned |
| `done` | `map` (`{"FL":id,...}`) | all four assigned and saved to flash |
| `cancelled` | `step`, `pos` | button pressed during mapping; previous mapping kept |
| `timeout` | `step`, `pos` | no qualifying drop within 10 min; previous mapping kept |
| `cleared` | | saved mapping erased (console `c`) |

## Tunables (`main/tires.h`)

| define | default | |
|--------|---------|---|
| `MAP_ORDER` | FL, RL, RR, FR | order the tires are deflated in |
| `MAP_DROP_PSI` | 1.5 | drop below baseline that assigns a sensor |
| `MAP_STEP_TIMEOUT_S` | 600 | per-step timeout |
| `MAP_CANDIDATE_AGE_S` | 900 | how recently a sensor must have been heard to get a baseline at start |
| `TIRE_STALE_S` | 600 | `stale` threshold |
| `SNAPSHOT_INTERVAL_S` | 10 | `tpms` snapshot period |
