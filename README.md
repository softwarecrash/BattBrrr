# BattBrrr (ESP32 + MQTT)

Production-ready battery heater controller for ESP32 (Arduino framework).
Includes DS18B20 sensor management, PID or hysteresis control, safety layer,
Wi-Fi/AP setup, Web UI, OTA, and MQTT integration.

## Quick Start
1. Build and flash with PlatformIO (see `platformio.ini`).
2. On first boot, the device starts an AP named `BattBrrr-<MAC>`.
3. Connect to the AP and open `http://192.168.4.1/wifisetup`.
4. Configure Wi-Fi, then open `http://<device-ip>/config`.
5. Set OneWire pin, heater output pin, targets, and safety limits.
6. Assign sensor roles (battery_primary is required).

## UI Notes
- Config save shows a status line plus a toast on success/failure.
- OTA page shows GitHub update availability and progress only.

## How To
- Rescan sensors: Status page -> `Rescan Sensors`.
- Reset faults: Status page -> `Reset Fault`.
- Output test: Status page -> `Output Test` (respects safety limits).
- Export config: Config page -> `Export Config`.
- Import config: Config page -> `Import Config`.
- Manual OTA: OTA page -> upload `.bin`.
- GitHub OTA: OTA page -> `Check Release` then `Update`.

## Modes
`IDLE`, `CHARGE`, `DISCHARGE`, optional `FROST_PROTECT`, optional `MANUAL`.
Faults force `FAULT` and shut the heater off until reset.

## Fault Codes
`OVER_TEMP`, `SENSOR_PRIMARY_FAIL`, `PLAUSIBILITY_FAIL`, `STUCK_ON_NO_HEAT`,
`THERMAL_RUNAWAY`, `MQTT_TIMEOUT`, `CONFIG_INVALID`.

## MQTT Topic Table
Base topic: `mqttBaseTopic` (default `battbrrr`).

| Direction | Topic | Payload | Notes |
|---|---|---|---|
| Publish | `<base>/heater/state` | JSON | Live status, temps, faults, mode, output, wifi/mqtt, uptime |
| Publish | `<base>/heater/event` | JSON | `{type, detail, ts_ms}` |
| Publish | `<base>/heater/autotune/state` | JSON | Autotune phase, progress, class, rate |
| Publish | `<base>/heater/autotune/progress` | JSON | Progress + current values |
| Publish | `<base>/heater/autotune/result` | JSON | PID result + quality |
| Subscribe | `<base>/heater/cmd/enable` | `true/false` or `1/0` | Enable controller |
| Subscribe | `<base>/heater/cmd/mode` | `IDLE/CHARGE/DISCHARGE/FROST_PROTECT/MANUAL` or `0..4` | Set mode |
| Subscribe | `<base>/heater/cmd/target_idle` | float | Target in C |
| Subscribe | `<base>/heater/cmd/target_charge` | float | Target in C |
| Subscribe | `<base>/heater/cmd/target_discharge` | float | Target in C |
| Subscribe | `<base>/heater/cmd/target_frost` | float | Target in C |
| Subscribe | `<base>/heater/cmd/max_temp` | float | Max temp cutoff |
| Subscribe | `<base>/heater/cmd/max_output` | float | Max output % |
| Subscribe | `<base>/heater/cmd/reset_fault` | any | Request fault reset |
| Subscribe | `<base>/heater/cmd/output_test` | JSON | `{pct, duration_s}` |
| Subscribe | `<base>/heater/cmd/autotune_start` | JSON | `{auto_save, aggressiveness, max_duration_s}` |
| Subscribe | `<base>/heater/cmd/autotune_abort` | any | Abort autotune |
| Subscribe | `<base>/heater/cmd/autotune_commit` | any | Save autotune result |

### BMS Inputs
Configured via UI:
- `bmsStateTopic` -> maps `charge/discharge/idle` to modes
- `bmsTempTopic` -> optional temperature fallback
- Optional JSON paths (dot notation): `bmsStatePath`, `bmsTempPath`
- Timeout: `bmsTimeoutS`

## GitHub OTA Configuration
GitHub OTA uses a build-time URL (no UI settings). Set it in `platformio.ini`:
- `OTA_GH_RELEASE_URL` -> GitHub API release endpoint
- Optional: `OTA_GH_ASSET_PATTERN` (default `*.bin`)

Example:
- `https://api.github.com/repos/<owner>/<repo>/releases/latest`

## Build
Use PlatformIO:
```
pio run -e esp32c3
```

## Notes
- No blocking `delay()` loops.
- Safety always wins: faults disable output until reset and safe.
