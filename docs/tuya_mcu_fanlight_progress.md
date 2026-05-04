# Tuya MCU FanLight Progress

Date: 2026-05-04
Branch: tuya-mcu-fan-light
Scope: current branch status after build fixes and cloud-schema DP alignment

## Current status

Status: in progress, buildable

Latest validated build:

`make -C src/telink ztu_fanlight`

Result: success, binary generated at `build/telink/bin/ztu_fanlight.bin`.

## Completed work

### Build and integration

- Telink toolchain path issue resolved by using shared install and project link.
- `ztu_fanlight` target now compiles and links successfully.
- Fanlight mode compatibility globals were added for builds that exclude `config_parser.c`.

### Zigbee control path

- Fan Control attribute writes are wired into the global dispatcher.
- Fan direction (DP4) is exposed as a writable custom attribute on Fan Control cluster (EP1).
- Fan direction writes from coordinator now map back to Tuya DP4.

### Tuya DP mapping alignment (official cloud schema)

- DP10 brightness updated to official range/step: `0..100`, `step=2`.
- Light-on behavior: when requested brightness maps to 0 while ON, firmware sends DP10=2.
- DP9 remains the OFF control path.
- DP11 color temperature treated as continuous `0..100`, `step=2` (not 3-state).
- DP11 direction is compile-time switchable via `TUYA_DP11_WARM_AT_100`.

## Known unimplemented / intentionally deferred

- DP2 fan mode (`nature`, `sleep`) recognized and logged, not yet mapped to ZCL behavior.
- DP102/DP103 timer/countdown remains intentionally ignored.

## Open risks / next validation

- Physical verification needed for DP11 warm/cool direction on real hardware.
- Coordinator-side UX validation needed for custom fan direction attribute (Z2M/ZHA exposure).
- No automated tests yet for DP<->ZCL conversion edge cases.

## Suggested next steps

1. Verify DP11 direction physically and lock default for `TUYA_DP11_WARM_AT_100`.
2. Add optional handling for DP2 (`nature`, `sleep`) as a custom mode attribute/state.
3. Run end-to-end pairing/control checks on both ZHA and Zigbee2MQTT.
4. Add unit tests for DP10/DP11 quantization and boundary mapping.