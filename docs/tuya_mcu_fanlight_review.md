# Tuya MCU FanLight Branch Review

Date: 2026-05-04
Branch: tuya-mcu-fan-light
Scope reviewed: current local working tree changes in this branch

## Review findings

### 1. High: Fan control writes from the coordinator will be ignored

The new fan implementation is built around writable `FanMode` attribute writes, but the global attribute-write dispatcher never forwards cluster `0x0202` writes to the new fan cluster callback.

- `src/zigbee/fan_cluster.c` defines `fan_cluster_callback_attr_write()` and exposes `ZCL_ATTR_FAN_MODE` as `ATTR_WRITABLE`.
- `src/zigbee/general_commands.c` only routes writes for Basic, Switch Config, Cover Switch Config, On/Off, Window Covering, and Poll Control.
- Result: Z2M/ZHA writes to Fan Control will update nothing on the MCU side, so fan speed/mode control is effectively broken.

Recommended fix: add Fan Control dispatch in `zigbee_on_attr_change()` and expose the callback in the fan cluster header with the same trampoline pattern used by other clusters.

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

Not complete yet:

- Fan Control attribute writes are not wired into the global attribute-change path.
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

1. Wire Fan Control writes through `src/zigbee/general_commands.c`.
2. Align the endpoint comments with the actual cluster model, or add the missing Identify cluster(s).
3. Build with the Telink toolchain present and fix any compile issues.
4. Validate pairing and control from both Zigbee2MQTT and ZHA.
5. Add targeted tests for DP-to-ZCL and ZCL-to-DP mapping where feasible.