# BeeGreen Watering System: MQTT Topics

All topics are under `<deviceId>/<suffix>`. Payloads are wrapped as `{"payload":"<value>","timestamp":"YYYY-MM-DD HH:MM:SS"}` unless noted.

## Topic matrix
| topic suffix | Sub/Pub | retained | payload / expectation |
| --- | --- | --- | --- |
| `pump_trigger` | Sub | no | integer seconds; `0` stop, `>0` run N sec |
| `set_schedule` | Sub | no | `index:HH:MM:duration:days:enabled` , index: 0-9 |
| `get_schedules` | Sub | no | empty payload to request all schedules |
| `update_firmware_url` | Sub | no | URL string to binary; runs OTA if pump idle |
| `restart` | Sub | no | any payload; device restarts |
| `reset_settings` | Sub | no | any payload; enter config portal |
| `calibrate` | Sub | no | runs a fixed 10s pump calibration to auto-set current threshold |
| `pump_status` | Pub | yes | `on/off` in payload field with timestamp |
| `heartbeat` | Pub | yes | CSV: `tempC,humidity,currentmA` (present fields only) |
| `next_schedule_due` | Pub | yes | timestamp or empty |
| `get_schedules_response` | Pub | no | JSON array strings: `idx:hr:min:dur:dow` (enabled only) |
| `tank_empty` | Pub | yes | `1` when dry, `0` after start |
| `power_status` | Pub | yes | `off:<ts>,on:<ts>` or `no power failure detected` |
| `status` | Pub | yes | `online` (retained) / LWT `offline` |
| `version` | Pub | yes | firmware version string on connect |

## Payload formats & examples

- Pump control `pump_trigger`  
  Payload: integer seconds. Examples: `300` (run 5 min), `0` (stop).

- Scheduler write `set_schedule`  
  Payload: `index:hour:minute:duration:daysOfWeek:enabled`  
  Example: `1:20:30:90:127:1` (slot 1, 20:30, 90s, every day, enabled).

- Scheduler dump `get_schedules_response`  
  Payload inside `payload` field is a JSON array of strings. Each string: `index:hour:minute:duration:daysOfWeek` for enabled entries. Example array: `["0:8:30:60:127","1:20:30:90:31"]`.

- Next alarm `next_schedule_due`  
  Payload: timestamp `YYYY-MM-DD HH:MM:SS` or empty when none.

- Heartbeat `heartbeat`  
  Payload: CSV of available readings in order: `tempC,humidity,currentmA` (fields omitted if sensor absent). Example: `27.45,58.10,123.00`.

## Appendix: days-of-week bitmask
Sum the day values you need:

| Day | Value |
| :-- | :-- |
| Sun | 1 |
| Mon | 2 |
| Tue | 4 |
| Wed | 8 |
| Thu | 16 |
| Fri | 32 |
| Sat | 64 |

Common sums: everyday `127`; weekdays `62`; weekends `65`. Example: disable Tue only → `127 - 4 = 123`.

