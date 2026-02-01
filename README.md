# BattBrrr (Battery Heater Controller)

## Features
- DS18B20 scanning, roles, offsets, and error tracking (primary/secondary/ambient)
- PID or hysteresis control with min on/off protection
- Fully-configurable GPIO mapping (outputs + optional inputs)
- Safety layer with hard cutoffs, plausibility checks, stuck-on detection, and thermal runaway protection
- MQTT status/events and command topics (plus BMS mode/temperature inputs)
- Web UI for status, configuration, tools, and OTA
- Manual OTA upload + GitHub OTA release updater
- PID Autotune (minutes-scale, non-blocking, conservative by default)
- No blocking delay loops

## Hardware
- ESP32 (tested on Wemos D1 mini ESP32)
- 1..N DS18B20 sensors on a single OneWire pin
- Heater output: PWM or windowed (relay/SSR)
- Optional inputs: enable/mode/manual override

## Quick Start
1. Build and flash with PlatformIO (see Build section).
2. On first boot, the device starts an AP named `BattBrrr-<MAC>`.
3. Connect to the AP and open `http://192.168.4.1/`.
4. Configure Wi-Fi, then open `http://<device-ip>/`.
5. Set OneWire pin, heater output pin/mode, targets, and safety limits.
6. Assign sensor roles (battery_primary is required).

## Web UI
- Status: live temps, mode, target, output, faults, Wi-Fi/MQTT, tools
- Config: all settings, conditional sections, import/export
- OTA: manual upload and GitHub release update
- PID Autotune: start/abort, progress, result, save

## Control Modes
- `IDLE`, `CHARGE`, `DISCHARGE`, optional `FROST_PROTECT`, optional `MANUAL`
- Any fault forces `FAULT` and disables heater output until reset and safe

## Safety (always on)
- Over-temp hard cutoff (latched)
- Primary sensor invalid (latched)
- Primary/secondary plausibility check (latched)
- Stuck-on / no-heat detection (latched)
- Thermal runaway detection (latched)
- Config invalid -> heater off

## PID Autotune
- Fully automatic, minutes-scale safe for slow thermal systems
- Probe phase classifies system as FAST/MEDIUM/SLOW
- Relay autotune with robust peak detection
- Aggressiveness presets: conservative / normal / aggressive
- Optional auto-save on completion

## OTA
### Manual OTA
Upload a compiled `.ota` from the OTA page. Progress and automatic reboot on success.

## MQTT
Base topic: `mqttBaseTopic` (default `battbrrr`).

| Direction | Topic | Payload | Notes |
|---|---|---|---|
| Publish | `<base>/heater/state` | JSON | temps, roles, mode, enabled, target, output, faults, wifi/mqtt, uptime |
| Publish | `<base>/heater/event` | JSON | `{type, detail, ts_ms}` |
| Publish | `<base>/heater/autotune/state` | JSON | phase, progress, class, rate |
| Publish | `<base>/heater/autotune/progress` | JSON | progress + current values |
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

### BMS Inputs (MQTT)
Configured via UI:
- `bmsStateTopic` -> maps `charge/discharge/idle` to modes
- `bmsTempTopic` -> optional temperature fallback
- Optional JSON paths (dot notation): `bmsStatePath`, `bmsTempPath`
- Timeout: `bmsTimeoutS`

## GPIO Notes
- Heater output pin must be a valid ESP32 output pin
- OneWire pin must be a valid output-capable GPIO
- Inputs can be disabled by setting pin to `-1`
- Invalid GPIO configs trigger `CONFIG_INVALID` fault



## Troubleshooting
- Sensor primary fail: assign a primary sensor and rescan
- Invalid config: check GPIO assignments and target limits
- No sensors: verify OneWire pin, power, pull-up resistor, and rescan
- Control temp shows `~`: UI/backend holding last known value during brief dropouts

## Notes
- Safety always wins: any fault disables output until reset and safe.
