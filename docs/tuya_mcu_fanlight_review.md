# Tuya MCU FanLight Branch Review

Date: 2026-05-04
Branch: tuya-mcu-fan-light
Scope reviewed: current branch changes, updated after fixing Fan Control attribute dispatch

## Review findings

### 1. Resolved: Fan Control attribute writes are now wired into the global dispatcher

The branch now routes cluster `0x0202` writes through the existing global attribute-change callback path.

- `src/zigbee/general_commands.c` now forwards `ZCL_CLUSTER_FAN_CONTROL` writes.
- `src/zigbee/fan_cluster.h` exposes a trampoline declaration matching the pattern used by other clusters.
- `src/zigbee/fan_cluster.c` now provides the trampoline wrapper used by the dispatcher.

Result: coordinator writes to `FanMode` should now reach the MCU bridge callback path instead of being dropped at the dispatcher layer.

### 2. Medium: The documented endpoint topology does not match the registered clusters

`src/fanlight/fanlight_init.c` documents Identify on both endpoints and implies OTA in the cluster sizing comments for endpoint 2, but the implementation only registers:

- Endpoint 1: Basic, OTA, Fan Control
- Endpoint 2: Basic, On/Off, Level, Color Control

That mismatch will make bring-up and coordinator-side inspection harder, and any expectation of Identify support on the light endpoint is currently unmet.

Recommended fix: either register the missing Identify cluster(s) or update the comments and sizing notes to reflect the actual topology.

### 3. Low: The Tuya UART driver comments and implementation disagree on interrupt usage

`src/base_components/tuya_mcu.c` says receive is handled by polling with no interrupt plumbing, but `tuya_mcu_init()` still enables UART RX IRQ and keeps an unused ring-buffer skeleton in the file.

This is not a proven runtime failure from static review alone, but it is an integration risk because the code path is ambiguous and harder to reason about during hardware debugging.

Recommended fix: either remove the IRQ/ring-buffer remnants and keep the driver purely polled, or finish the interrupt-backed implementation.

## Summary of current changes

The branch adds the first cut of a dedicated Tuya MCU fan/light device path for Telink:

- New device entry in `device_db.yaml` for `MODULE_ZTU_FANLIGHT`
- New Telink build target in `src/telink/Makefile` (`make ztu_fanlight`)
- New UART Tuya MCU transport in `src/base_components/tuya_mcu.c`
- New fan/light app entrypoints in `src/fanlight/`
- New Zigbee clusters for Fan Control and Color Temperature Light in `src/zigbee/`
- New Telink custom ZCL registration for Fan Control and Color Control in `src/telink/custom_zcl/`
- Telink ZCL registration hook updates in `src/telink/hal/zigbee_zcl.c`

## Progress assessment

Status: in progress

Completed:

- Device metadata and image type were added.
- A dedicated build flow for the fan/light target was added.
- Basic Tuya UART frame handling exists.
- MCU to Zigbee state mapping exists for fan on/off, speed, light on/off, brightness, and color temperature.
- Zigbee to MCU command mapping exists for fan mode, on/off, level, and color temperature commands.
- Fan Control attribute writes are wired into the global attribute-change path.

Not complete yet:

- The documented endpoint model is not yet aligned with the actual cluster registration.
- No successful local build was possible in this environment because the Telink compiler is missing.
- No hardware validation, pairing validation, or coordinator validation was observed.
- No automated tests were added for the new mapping logic.

## Validation notes

Static review only.

Attempted build command:

`make -C src/telink ztu_fanlight`

Build could not run here because `../../telink_tools/toolchain/tc32/bin/tc32-elf-gcc` is not available in the current environment.

## Suggested next steps

1. Align the endpoint comments with the actual cluster model, or add the missing Identify cluster(s).
2. Build with the Telink toolchain present and fix any compile issues.
3. Validate pairing and control from both Zigbee2MQTT and ZHA.
4. Add targeted tests for DP-to-ZCL and ZCL-to-DP mapping where feasible.